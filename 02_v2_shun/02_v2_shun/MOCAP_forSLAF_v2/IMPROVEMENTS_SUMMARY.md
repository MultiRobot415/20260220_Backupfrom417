# 改善実装まとめ - CSVログ追加とTモード再設計

**日時**: 2025-11-27  
**改善内容**: 
1. CSVログに重み行列のノルム、パターン、ゲイン値を追加
2. CSVログ出力エラーの修正
3. Tモードの再設計：固定初期位置 → 現在MOCAP位置を目標として設定
4. k_cv=0の設計を削除し、k_cv=0.1を維持

---

## 1. CSVログへの追加項目

### 追加項目

以下の項目をCSVログ（`control_log_*.csv`）に追加しました：

1. **`observer_weight_norm`**: 重み行列のノルム
   - 計算式: `||L|| = sqrt(||L_p||^2 + ||L_v||^2)`
   - `L_p = [[k_p, 0], [0, k_p]]`, `L_v = [[k_v, 0], [0, k_v]]`
   - `||L_p|| = k_p * sqrt(2)`, `||L_v|| = k_v * sqrt(2)`
   - 現在値: `sqrt((1.0*sqrt(2))^2 + (1.0*sqrt(2))^2) = 2.0`

2. **`observer_weight_pattern`**: 重み行列パターン
   - 現在: `"fixed"` （固定重み行列）
   - 将来的に適応型に拡張可能

3. **`k_p`**: 位置ゲイン（推定器）
   - 現在値: `1.0`

4. **`k_v`**: 速度ゲイン（推定器）
   - 現在値: `1.0`

5. **`k_cv`**: 制御器速度ゲイン（動的変更）
   - Tモード: `0.0`
   - Hモード: `0.1` (k_cv_nominal)

### CSVヘッダー（更新後）

```csv
timestamp,drone_id,role,mode,
x,y,z,
x_hat,y_hat,z_hat,
vx,vy,vz,
vx_hat,vy_hat,vz_hat,
target_x,target_y,target_z,
error_x,error_y,error_z,
rc_lr,rc_fb,rc_ud,rc_yaw,
trust,
xi_x,xi_y,xi_z,
psi_x,psi_y,psi_z,
tau,
is_collinear,
is_occluded,
tracking_error,estimation_error,
observer_weight_norm,observer_weight_pattern,k_p,k_v,k_cv
```

### 実装箇所

#### 1. `slaf_pid_controller.py`

```python
def get_state(self):
    # 重み行列（推定器ゲイン）のノルム計算
    L_p_norm = self.k_p * np.sqrt(2)
    L_v_norm = self.k_v * np.sqrt(2)
    observer_weight_norm = np.sqrt(L_p_norm**2 + L_v_norm**2)
    
    # 重み行列パターン（現在は固定重み行列を使用）
    observer_weight_pattern = "fixed"
    
    return {
        # ... 既存の状態変数 ...
        'observer_weight_norm': observer_weight_norm,
        'observer_weight_pattern': observer_weight_pattern,
        'k_p': self.k_p,
        'k_v': self.k_v,
        'k_cv': self.k_cv
    }
```

#### 2. `csv_logger.py`

CSVヘッダーに追加：
```python
control_csv_writer.writerow([
    # ... 既存のヘッダー ...
    'observer_weight_norm',
    'observer_weight_pattern',
    'k_p', 'k_v', 'k_cv'
])
```

ログ記録関数に追加：
```python
def log_slaf_control_data(log_data):
    row = [
        # ... 既存のデータ ...
        log_data.get('observer_weight_norm', 0.0),
        log_data.get('observer_weight_pattern', 'fixed'),
        log_data.get('k_p', 0.0),
        log_data.get('k_v', 0.0),
        log_data.get('k_cv', 0.0)
    ]
    control_csv_writer.writerow(row)
```

#### 3. `mocap_slaf_main.py`

ログデータに追加：
```python
log_data = {
    # ... 既存のデータ ...
    'observer_weight_norm': state['observer_weight_norm'],
    'observer_weight_pattern': state['observer_weight_pattern'],
    'k_p': state['k_p'],
    'k_v': state['k_v'],
    'k_cv': state['k_cv']
}
log_control_data(log_data)
```

---

## 2. CSVログ出力エラーの修正

### 問題

`csv_logger.py`の`log_slaf_control_data()`関数で未定義変数エラーが発生していました：

```python
# 問題のコード
row = [
    timestamp,  # ❌ 未定義
    drone_id,   # ❌ 未定義
    ...
]
```

### 修正

```python
# データ取得を追加
timestamp = log_data.get('timestamp', time.time())
drone_id = log_data.get('drone_id', 0)
follower_id = log_data.get('follower_id', 3)
mode = log_data.get('mode', 'slaf')

pos_2d = log_data['position']
pos_hat_2d = log_data['position_hat']
target_2d = log_data['target_position']
vel_2d = log_data['velocity']
vel_hat_2d = log_data['velocity_hat']
xi_2d = log_data['xi']
psi_2d = log_data['psi']
rc_command = log_data['rc_command']
tau = log_data.get('tau', 0.0)
is_collinear = log_data.get('is_collinear', False)
is_occluded = log_data.get('is_occluded', False)
tracking_error = log_data.get('tracking_error', 0.0)
estimation_error = log_data.get('estimation_error', 0.0)

# 行を構築
row = [
    timestamp, drone_id, f"follower_{follower_id}", mode,
    pos_2d[0], 1.0, pos_2d[1],  # 実位置
    ...
]
```

---

## 3. Tモードの再設計

### 変更前（削除された設計）

```python
# 固定初期位置への移動
follower_target_positions = {
    3: np.array([1.0, 0.8]),  # ドローン1
    4: np.array([0.0, 0.8])   # ドローン2
}

# 速度ゲインをゼロに設定
for follower_id in [3, 4]:
    controller.set_gains(k_cv=0.0)
```

**問題点**:
- 固定位置への移動は挙動が不安定
- k_cv=0の設計は制御性能を低下させる

---

### 変更後（新設計）

```python
# Tキー押下時の現在位置を目標として設定（一回のみ取得）
if MOCAP_CONNECTED:
    for follower_id in [3, 4]:
        mocap_pos = ms.get_rigid_body_position(rigid_id)
        if mocap_pos:
            x = mocap_pos.get('x', 0.0)
            z = mocap_pos.get('z', 0.0)
            follower_target_positions[follower_id] = np.array([x, z])
            follower_target_velocities[follower_id] = np.array([0.0, 0.0])
            
# 推定器を現在位置で初期化（推定誤差ゼロ）
slaf_manager.initialize_followers(initial_positions, initial_velocities)

# k_cvは0.1のまま維持（変更なし）
```

**利点**:
- 現在位置を目標とするため、ドローンはほぼ静止
- 制御則: `u = -k_cp*(p̂ - p*) - k_cv*(v - v*) + 0`
- 初期状態: `p̂ ≈ p* ≈ p_mocap`, `v ≈ v* ≈ 0` → `u ≈ 0`
- k_cv=0.1を維持することで、Hモード移行時の制御性能を確保

---

### 制御則の挙動解析

#### Tモード開始時

**状態**:
- 目標位置: `p* = p_mocap(t_T)`（Tキー押下時のMOCAP位置）
- 目標速度: `v* = [0, 0]`
- 目標加速度: `a* = [0, 0]`
- 推定位置: `p̂(0) = p_mocap(t_T)`（推定誤差ゼロ）
- 推定速度: `v̂(0) = [0, 0]`

**制御入力**:
```
u = -k_cp*(p̂ - p*) - k_cv*(v - v*) + a*
  = -k_cp*(p̂ - p_mocap) - k_cv*v + 0
```

**初期時刻**: 
```
p̂ ≈ p_mocap, v ≈ 0
→ u ≈ -k_cp*0 - k_cv*0 = 0
```
→ **ドローンはほぼ静止**

---

#### Hモード開始時

**状態**:
- 目標位置: `p*` は加速度積分により更新
- 目標速度: `v* = v* + a* * dt`（積分で更新）
- 目標加速度: `a* = [0, -0.08]` m/s²

**制御入力**:
```
u = -k_cp*(p̂ - p*) - k_cv*(v - v*) + a*
```

**k_cv=0.1を維持することで**:
- 速度項 `-k_cv*(v - v*)` が正しく機能
- 目標軌道への追従性が向上

---

### 動作フロー

#### Tモード開始

```
1. Tキー押下
2. 現在のMOCAP位置を取得 → p* として設定
3. 目標速度を[0, 0]に設定
4. 推定器を現在位置で初期化（推定誤差ゼロ）
5. k_cv = 0.1のまま維持
6. SLAF制御開始
```

出力例：
```
フォロワー（実機ドローン）目標位置を現在位置に設定:
  MOCAP接続 - 現在の真の位置・速度を目標として設定
  フォロワー3（ドローン1）: 目標位置=[0.854, 0.923], 目標速度=[0.000, 0.000]
  フォロワー4（ドローン2）: 目標位置=[0.043, 0.897], 目標速度=[0.000, 0.000]
  Hモードをリセット
目標位置・速度設定完了（現在位置ベース）

推定器初期化中（Assumption: 初期推定誤差零）...
  フォロワー3（ドローン1）: p_hat(0)=[0.854, 0.923], v_hat(0)=[0.000, 0.000]
  フォロワー4（ドローン2）: p_hat(0)=[0.043, 0.897], v_hat(0)=[0.000, 0.000]
  推定器を現在位置で初期化しました（推定誤差ゼロ）
```

#### Hモード開始

```
1. Hキー押下
2. h_mode_active = True
3. 目標速度を初期化 [0.0, 0.0]
4. k_cv = 0.1のまま維持
5. Hモード更新開始（加速度積分 → 速度、速度積分 → 位置）
```

出力例：
```
============================================================
Hキー検出 - Hモード開始
目標加速度: [0.000, -0.080] m/s^2
[DEBUG] h_mode_active=True, last_h_mode_update_time=...
============================================================
```

### 効果

#### Tモード時

- 制御則: `u = -k_cp*(p̂ - p*) - k_cv*(v - v*)`
- 安定性: ✅ `p̂ ≈ p*`, `v ≈ v* ≈ 0` → `u ≈ 0`
- 収束性: ✅ ドローンはほぼ静止

#### Hモード時

- 制御則: `u = -k_cp*(p̂ - p*) - k_cv*(v - v*) + a*`
- 安定性: ✅ 速度項が正しく機能（v*が更新される）
- 追従性: ✅ 加速度積分による軌道追従

---

## CSVログでの確認方法

### 1. 重み行列のノルム

```python
import pandas as pd
df = pd.read_csv('slaf_results/control_log_*.csv')

# 重み行列のノルム確認
print(df['observer_weight_norm'].unique())
# 出力: [2.0]  (k_p=1.0, k_v=1.0の場合)
```

### 2. 重み行列パターン

```python
print(df['observer_weight_pattern'].unique())
# 出力: ['fixed']
```

### 3. ゲイン値の動的変化

```python
# Tモード → Hモードでk_cvが変化することを確認
df_tmode = df[df['mode'] == 'slaf'].iloc[:100]  # Tモード開始直後
df_hmode = df[df['mode'] == 'slaf'].iloc[200:]  # Hモード開始後

print(f"Tモード時 k_cv: {df_tmode['k_cv'].mean()}")  # 0.0
print(f"Hモード時 k_cv: {df_hmode['k_cv'].mean()}")  # 0.1
```

### 4. タイムスタンプごとのゲイン値

```python
# ゲイン値の時系列変化
import matplotlib.pyplot as plt

plt.figure(figsize=(12, 4))
plt.plot(df['timestamp'], df['k_cv'], label='k_cv')
plt.axhline(y=0.0, color='r', linestyle='--', label='T mode')
plt.axhline(y=0.1, color='g', linestyle='--', label='H mode')
plt.xlabel('Time')
plt.ylabel('k_cv')
plt.legend()
plt.title('Control Velocity Gain over Time')
plt.show()
```

---

## 期待される動作

### Tモード

```
[SLAF制御] 飛行: True | Hモード: OFF | 時刻: ...
ドローン1（フォロワー3）:
  実際位置=[0.95, 0.82]
  推定位置=[0.95, 0.82]
  目標位置=[1.00, 0.80]
  制御入力=[-0.0500, -0.0200]  ← 位置制御のみ（速度項なし）
  RC指令=[lr:-1, fb:-2]
```

### Hモード

```
[SLAF制御] 飛行: True | Hモード: ON | 時刻: ...
  目標速度: [0.000, -0.160] m/s

ドローン1（フォロワー3）:
  実際位置=[0.98, 0.75]
  推定位置=[0.98, 0.75]
  目標位置=[1.00, 0.73]
  制御入力=[0.0200, -0.0400]  ← 位置+速度+加速度制御
  RC指令=[lr:-2, fb:1]
```

---

## まとめ

### 実装完了確認

✅ CSVログに重み行列のノルム追加  
✅ CSVログに重み行列パターン追加  
✅ CSVログにゲイン値（k_p, k_v, k_cv）追加  
✅ CSVログ出力エラー修正  
✅ Tモードの再設計：固定初期位置 → 現在MOCAP位置  
✅ k_cv=0の設計削除（k_cv=0.1を維持）  
✅ `initialize_followers()`に速度引数追加  
✅ 構文チェック成功  

**実装完了・テスト準備完了** 🎉

詳細な設計変更については`T_MODE_REDESIGN.md`を参照してください。slaf_pid_controller.pyにk_cv_nominal追加  

### ユーザー要求対応

1. **CSVログ追加**: ✅ 完了
   - 重み行列のノルム
   - 選ばれている重み行列のパターン

2. **Tモード不安定性対策**: ✅ 完了
   - 速度ゲインを動的調整
   - 複雑な実装を避け、シンプルに解決

---

**作成日**: 2025-11-27  
**最終更新**: 2025-11-27（重み行列とξの計算に関する重大なバグ修正を追加）  
**ステータス**: 実装完了、テスト準備完了

---

## **重要な追加修正（2025-11-27 4:54pm）**

### **重み行列とξの計算に関する重大なバグ修正**

論文とMATLABコードの詳細検証により、**2つの重大な実装エラー**を発見し修正しました。

#### **問題1：重み行列の計算に推定値を使用していた**

**間違った実装**：
```python
# ❌ 推定位置から重み行列を計算（間違い）
Hij, Hik, is_collinear = calculate_weight_matrices_for_agent(p_i_hat, p_j_hat, p_k_hat)
```

**論文Remark 5の要求**：
> "The weight matrices can be obtained by **bearing measurements**"

**修正後**：
```python
# ✅ 観測値（実位置、MOCAP測定）から重み行列を計算
Hij, Hik, is_collinear = calculate_weight_matrices_for_agent(p_i_actual, p_j_actual, p_k_actual)
```

#### **問題2：ξの計算式が論文Eq. (20)と異なっていた**

**間違った実装**：
```python
# ❌ 簡略版（不正確）
xi = Hij @ (g_ij_hat - g_ij_star) + Hik @ (g_ik_hat - g_ik_star)
```

**論文Eq. (20)の正しい式**：
```
ξijk = H^T_ii * Hij * (p̂_j - p̂_i) + H^T_ii * Hik * (p̂_k - p̂_i)
```
ここで `H_ii = H_ij + H_ik`

**修正後**：
```python
# ✅ 論文Eq. (20)に準拠
Hii = Hij + Hik
p_rel_ij_hat = p_j_hat - p_i_hat
p_rel_ik_hat = p_k_hat - p_i_hat
xi = Hii.T @ Hij @ p_rel_ij_hat + Hii.T @ Hik @ p_rel_ik_hat
```

#### **修正ファイル**

1. **`weight_matrices.py`**:
   - `calculate_xi_correction()`の引数を拡張（実位置を追加）
   - 重み行列を観測値から計算
   - ξの計算式を論文Eq. (20)に修正

2. **`slaf_pid_controller.py`**:
   - `calculate_xi()`の引数を拡張（`neighbor_positions_actual`を追加）
   - 実位置を重み行列の計算に使用

#### **理論的根拠**

- **重み行列H**：**センサ観測（bearing測定）**から得られる
  - シミュレーション：真の位置 = センサ観測
  - 実機：MOCAP測定値 = センサ観測

- **ξの計算**：**推定位置**を使用（変更なし）

- **制御入力への影響**：
  1. 観測値 → 重み行列H
  2. 重み行列H + 推定位置 → ξ
  3. ξ → 推定器の更新
  4. 推定位置 → 制御入力

つまり、**重み行列Hは間接的に制御入力に影響**します。

詳細は `WEIGHT_MATRIX_XI_FIX.md` を参照してください。

---
