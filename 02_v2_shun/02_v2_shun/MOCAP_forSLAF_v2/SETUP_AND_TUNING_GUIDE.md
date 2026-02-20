# v2オブザーバ型SLAF実機実装 - セットアップとチューニングガイド

## ファイル構成の説明

### メインファイル
- **`mocap_slaf_main.py`**: v2オブザーバ型SLAF制御のメインプログラム（★使用★）
- **`mocap_slaf_main_v2.py`**: 古いMOCAP制御プログラム（SLAF不使用、無関係）
- **`mocap_slaf_main_broken.py`**: 壊れたバックアップファイル（無視）

### 制御器
- **`slaf_observer_controller.py`**: v2オブザーバ型SLAF制御器（★v2専用★）
- **`slaf_pid_controller.py`**: v1 PID階層型SLAF制御器（v2では**未使用**）

### その他のモジュール（v1と共通）
- `virtual_leader.py`: 仮想リーダー管理
- `weight_matrices.py`: Bearing重み行列計算
- `mocap_stream.py`: MOCAP連携
- `custom_tello.py`: Telloドローン制御
- `keyboard_control.py`: キーボード入力
- `csv_logger.py`: CSVログ記録

---

## 初期位置の設定（v1準拠）

### 3つの位置の概念

#### 1. **初期位置** (`p_actual(0)`)
- **取得方法**: Tキー押下時に**MOCAPから現在位置を一瞬取得**
- **用途**: 実位置の初期値

#### 2. **初期目標位置** (`p_star(0)`)
- **取得方法**: Tキー押下時に**MOCAPから現在位置を一瞬取得**（初期位置と同じ）
- **用途**: 目標軌道の初期値
- **更新**: Hモード開始後、時変的に更新される

#### 3. **初期推定位置** (`p_hat(0)`)
- **取得方法**: Tキー押下時に**初期目標位置をコピー**（推定誤差零の仮定）
- **用途**: 推定器の初期値
- **理論**: `p_hat(0) = p_actual(0) = p_star(0)`（推定誤差零）

### Tキー押下時の処理フロー（v1準拠）

```python
# 1. MOCAPから現在位置を取得（一瞬のみ）
for follower_id in [3, 4]:
    mocap_pos = ms.get_rigid_body_position(rigid_id)
    x = mocap_pos.get('x', 0.0)
    z = mocap_pos.get('z', 0.0)
    
    # 2. 目標位置を現在位置に設定
    follower_target_positions[follower_id] = np.array([x, z])
    follower_target_velocities[follower_id] = np.array([0.0, 0.0])
    
    # 3. 推定器を初期化（推定誤差零）
    initial_position = follower_target_positions[follower_id].copy()  # p_hat(0) = p_star(0)
    initial_velocity = follower_target_velocities[follower_id].copy()  # v_hat(0) = 0
    controller.initialize_state(initial_position, initial_velocity)
```

### ★重要★ 固定値は使わない

- **v1準拠**: `DEFAULT_FOLLOWER_POSITIONS`やハードコードされた固定値は**使用しない**
- **動的取得**: Tキー押下時に**必ずMOCAPから現在位置を取得**
- **理由**: 実機の真の位置から開始することで、初期推定誤差を零にできる

---

## ゲイン調整方法

### デフォルトゲイン（シミュレーションと同じ）

ファイル: **`slaf_observer_controller.py`** の `__init__()` メソッド

```python
# オブザーバゲイン
self.K_obs = 10.0   # 速度推定誤差の収束速度

# 制御器ゲイン
self.K_p = 10.0     # 位置制御ゲイン（固有振動数 ω_n = √K_p ≈ 3.16 rad/s）
self.K_v = 10.0     # 速度制御ゲイン（減衰比 ζ = K_v/(2√K_p) ≈ 1.58）

# 補正項のゲイン
self.w_xi = 5.0     # ξ項のゲイン
self.w_psi = 1.0    # ψ項のゲイン

# 共線回避パラメータ
self.gamma = 1.0
self.lambda_max = 0.99
```

### ゲイン調整の3つの方法

#### 方法1: ソースコード直接編集（推奨）

**ファイル**: `slaf_observer_controller.py` (行43-58付近)

```python
class SLAFObserverController:
    def __init__(self, follower_id, neighbors, dt=0.1):
        # === オブザーバ型パラメータ ===
        self.K_obs = 10.0   # ← ここを変更
        self.K_p = 10.0     # ← ここを変更
        self.K_v = 10.0     # ← ここを変更
        self.w_xi = 5.0     # ← ここを変更
        self.w_psi = 1.0    # ← ここを変更
```

**変更後**: プログラムを再起動

#### 方法2: `set_all_gains()`で一括設定（実行時）

**ファイル**: `mocap_slaf_main.py` の `initialize_slaf_system()` 関数に追加

```python
def initialize_slaf_system():
    # ... (既存の初期化コード) ...
    
    # ゲインを一括設定（全フォロワー共通）
    slaf_manager.set_all_gains(
        K_obs=15.0,    # オブザーバゲイン
        K_p=12.0,      # 位置制御ゲイン
        K_v=12.0,      # 速度制御ゲイン
        w_xi=8.0,      # ξゲイン
        w_psi=1.5      # ψゲイン
    )
    print("カスタムゲインを設定しました")
```

#### 方法3: 個別フォロワーのゲイン設定（上級者向け）

```python
# フォロワー3のみゲイン変更
controller_3 = slaf_manager.get_controller(3)
controller_3.set_gains(K_obs=20.0, K_p=15.0, K_v=15.0)

# フォロワー4のみゲイン変更
controller_4 = slaf_manager.get_controller(4)
controller_4.set_gains(K_obs=8.0, K_p=8.0, K_v=8.0)
```

---

## ゲインチューニング指針

### 1. オブザーバゲイン `K_obs`

**役割**: 速度推定誤差の収束速度

```
˙v_hat = u + K_obs(v - v_hat)
```

- **大きい** → 速い収束、振動的、ノイズ敏感
- **小さい** → 遅い収束、滑らか、ノイズ鈍感
- **推奨範囲**: 5.0 ~ 20.0
- **初期値**: 10.0（シミュレーションと同じ）

**調整例**:
- 振動が激しい → `K_obs = 5.0` に下げる
- 応答が遅い → `K_obs = 15.0` に上げる

### 2. 位置制御ゲイン `K_p`

**役割**: 位置誤差のフィードバック強度

```
u = a* - K_p(p_hat - p*) - K_v(v - v*) + w_psi * ψ
```

固有振動数: **ω_n = √K_p**

- **ω_n = 2 rad/s** → `K_p = 4`（ゆっくり）
- **ω_n = 3 rad/s** → `K_p = 9`（中程度）
- **ω_n = 4 rad/s** → `K_p = 16`（速い）
- **ω_n = 5 rad/s** → `K_p = 25`（非常に速い）

**推奨範囲**: 4 ~ 25
**初期値**: 10.0（ω_n ≈ 3.16 rad/s）

### 3. 速度制御ゲイン `K_v`

**役割**: 速度誤差のフィードバック強度（減衰）

減衰比: **ζ = K_v / (2√K_p)**

- **ζ < 0.7**: 不足減衰（振動的）
- **ζ = 0.7**: 適度な減衰
- **ζ = 1.0**: 臨界減衰（振動なし、最速収束）
- **ζ > 1.0**: 過減衰（振動なし、やや遅い）

**推奨ζ**: 0.7 ~ 1.5
**初期値**: `K_v = 10.0`（K_p=10.0のとき、ζ ≈ 1.58）

**調整例**:
- `K_p = 10` のとき、臨界減衰 `ζ = 1.0` → `K_v = 2√10 ≈ 6.3`
- `K_p = 16` のとき、臨界減衰 `ζ = 1.0` → `K_v = 2√16 = 8.0`

### 4. 補正ゲイン `w_xi`

**役割**: 幾何学的補正項ξの強度

```
˙p_hat = v + w_xi * ξ
```

- **大きい** → 推定誤差の収束が速い
- **小さい** → 推定誤差の収束が遅い
- **推奨範囲**: 1.0 ~ 10.0
- **初期値**: 5.0（シミュレーションと同じ）

### 5. 補正ゲイン `w_psi`

**役割**: オクルージョン時の共線回避項ψの強度

```
u = a* - K_p(p_hat - p*) - K_v(v - v*) + w_psi * ψ
```

- **大きい** → 共線回避動作が強い（大きく避ける）
- **小さい** → 共線回避動作が弱い（小さく避ける）
- **推奨範囲**: 0.5 ~ 2.0
- **初期値**: 1.0（シミュレーションと同じ）

---

## トラブルシューティング

### 問題1: 制御が振動する

**原因**: ゲインが高すぎる

**対策**:
1. `K_obs` を下げる（例: 10.0 → 5.0）
2. `K_p` を下げる（例: 10.0 → 8.0）
3. `K_v` を上げる（減衰を増加、例: 10.0 → 12.0）

### 問題2: 応答が遅い

**原因**: ゲインが低すぎる

**対策**:
1. `K_p` を上げる（例: 10.0 → 16.0）
2. `K_v` を調整して臨界減衰を維持（例: `K_v = 2√16 = 8.0`）

### 問題3: 推定誤差が収束しない

**原因**: `w_xi` または `K_obs` が小さい

**対策**:
1. `w_xi` を上げる（例: 5.0 → 8.0）
2. `K_obs` を上げる（例: 10.0 → 15.0）

### 問題4: 定常偏差が残る

**原因**: v2はI制御がないため、定常偏差が残る可能性あり

**対策**:
1. 目標位置設計を見直す
2. `K_p` を上げる（定常偏差を小さくする）
3. v1（PID階層型）の使用を検討

### 問題5: オクルージョン時に共線から逃げない

**原因**: `w_psi` が小さい

**対策**:
1. `w_psi` を上げる（例: 1.0 → 1.5）
2. `gamma` を調整（例: 1.0 → 0.8）

---

## 実機テストの手順

### 1. 初期テスト（デフォルトゲイン）

```bash
cd /home/initial/02_v2_shun/02_v2_shun/MOCAP_forSLAF_v2
python mocap_slaf_main.py
```

1. **Q**: 離陸
2. **T**: SLAF制御モード開始（現在位置を取得）
3. **H**: Hモード開始（目標軌道生成）
4. データログ確認（`slaf_results/control_log_*.csv`）

### 2. ゲインチューニング

1. 振動の有無を確認
2. 追跡誤差の収束速度を確認
3. 必要に応じてゲイン調整（`slaf_observer_controller.py`を編集）
4. プログラム再起動

### 3. オクルージョンテスト

1. **T**: SLAF制御モード開始
2. **H**: Hモード開始
3. **O**: オクルージョンON（フォロワー4）
4. 共線回避動作を確認
5. **P**: オクルージョンOFF

### 4. 性能評価（v1との比較）

- 収束速度
- 定常偏差
- 振動の有無
- オクルージョン時の挙動

---

## よくある質問

### Q1: `mocap_slaf_main_v2.py` は何ですか？

**A**: 古いMOCAP制御プログラム（SLAF制御不使用）です。**v2とは無関係**です。無視してください。

### Q2: ゲイン調整後、プログラムを再起動する必要がありますか？

**A**: はい。`slaf_observer_controller.py`を編集した場合、**プログラムを再起動**してください。

### Q3: v1とv2でゲインの意味は同じですか？

**A**: **異なります**。

| ゲイン | v1 (PID階層型) | v2 (オブザーバ型) |
|--------|----------------|-------------------|
| 推定器 | `k_p, k_v, k_i` | `K_obs` |
| 制御器 | `k_cp, k_cv` | `K_p, K_v` |
| 補正 | `xi_gain` | `w_xi, w_psi` |

### Q4: 初期位置が固定値になっていないか心配です

**A**: 大丈夫です。**Tキー押下時にMOCAPから現在位置を動的に取得**します。

```python
# mocap_slaf_main.py 行339-355
if MOCAP_CONNECTED:
    for follower_id in [3, 4]:
        mocap_pos = ms.get_rigid_body_position(rigid_id)  # ← 動的取得
        x = mocap_pos.get('x', 0.0)
        z = mocap_pos.get('z', 0.0)
        follower_target_positions[follower_id] = np.array([x, z])
```

### Q5: v1とv2を同時に実行できますか？

**A**: できません（同じドローンを使用するため）。別々のディレクトリで**どちらか一方のみ**を実行してください。

---

## まとめ

### ファイル構成
- **メインプログラム**: `mocap_slaf_main.py`（v2専用）
- **制御器**: `slaf_observer_controller.py`（v2専用）
- **無関係**: `mocap_slaf_main_v2.py`（古いMOCAP制御、無視）

### 初期位置設定（v1準拠）
- **Tキー押下時**: MOCAPから現在位置を取得
- **3つの位置**: 初期位置、初期目標位置、初期推定位置（全て同じ値）
- **推定誤差零**: `p_hat(0) = p_actual(0) = p_star(0)`

### ゲイン調整
- **場所**: `slaf_observer_controller.py` の `__init__()`（行43-58）
- **デフォルト**: K_obs=10, K_p=10, K_v=10, w_xi=5, w_psi=1
- **変更後**: プログラム再起動

### 推奨設定
- **固有振動数**: ω_n = 2~5 rad/s（K_p = 4~25）
- **減衰比**: ζ = 0.7~1.5（臨界減衰付近）
- **オブザーバ**: K_obs = 5~20

---

**作成日**: 2025年12月11日  
**対象**: v2オブザーバ型SLAF実機実装  
**理論ベース**: v4_observer.tex, sim_v2/system_dynamics.m
