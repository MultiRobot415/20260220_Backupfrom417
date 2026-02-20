# 実装更新サマリー（2025/11/24）

## 問題の報告
ユーザーから以下の問題が報告されました：
1. **Tキーを離すと制御入力が入り続ける** → 機体が吹き飛ぶ
2. **推定位置の確認が必要** → 推定位置/速度がおかしい可能性

## 実装した変更

### 1. 制御モードの変更
#### 変更前（誤った実装）
```python
# Tキーを押している間だけ制御入力を送信
if t_pressed:
    send_rc_control(control_lr, control_fb, control_ud, control_yaw)
else:
    # 何も送信しない（15秒後に停止）
```
**問題**: キーを離した後も制御ループが動作していた

#### 変更後（正しい実装）
```python
# Tキーでモード切替後、常に観測・推定・制御を実行
if control_mode == "slaf":
    # 1. 観測（MOCAP位置取得）
    p_actual = get_mocap_position()
    
    # 2. 推定（PID型オブザーバー）
    control_inputs = slaf_manager.update_followers(mocap_positions, leader_states_2d)
    
    # 3. 制御コマンド送信（常に実行）
    send_rc_control(control_lr, control_fb, control_ud, control_yaw)
```

### 2. 目標位置の更新
#### G/B/V/Nキーによる目標位置移動
```python
# キーを押している間、目標位置が連続的に移動
if "g" in pressed_keys:
    virtual_leaders.update_reference_leader_target(dx=0.05)  # 前進
if "b" in pressed_keys:
    virtual_leaders.update_reference_leader_target(dx=-0.05)  # 後退
if "v" in pressed_keys:
    virtual_leaders.update_reference_leader_target(dz=-0.05)  # 左
if "n" in pressed_keys:
    virtual_leaders.update_reference_leader_target(dz=0.05)  # 右
```

#### キーを離した時の挙動
1. **目標位置は固定** → `p* = const`
2. **制御は継続** → 観測・推定・制御は常に実行
3. **収束条件**:
   ```
   ||p̂ - p*|| < ε  （推定位置が目標に到達）
   AND
   ||v|| < δ        （実速度がゼロに近い）
   ⇒ u ≈ 0 （制御入力がゼロになり停止）
   ```

### 3. CSVログ出力の強化
#### 追加した列
```csv
# control_log_YYYYMMDD_HHMMSS.csv
timestamp, drone_id, role, mode,
x, y, z,              # 実位置（MOCAP）
x_hat, y_hat, z_hat,  # 推定位置（オブザーバー）← 新規追加
vx, vy, vz,           # 実速度（数値微分）← 新規追加
vx_hat, vy_hat, vz_hat,  # 推定速度（オブザーバー）← 新規追加
target_x, target_y, target_z,
error_x, error_y, error_z,
rc_lr, rc_fb, rc_ud, rc_yaw,
trust,
obs_error_x, obs_error_y, obs_error_z,
obs_state_x, obs_state_y, obs_state_z,
tracking_error,      # 追跡誤差ノルム ← 新規追加
estimation_error     # 推定誤差ノルム ← 新規追加
```

### 4. デバッグ出力の追加
#### コンソール出力（2秒ごと）
```
ドローン1: 実際位置=[0.12, 0.45], 推定位置=[0.13, 0.46], 目標位置=[0.00, 0.00], 制御値=[lr:-65, fb:60]
```

## SLAF制御の演算処理

### 観測（Observation）
```python
# MOCAP位置取得
mocap_pos = ms.get_rigid_body_position(rigid_id)
p_actual = np.array([mocap_pos['x'], mocap_pos['z']])  # 2D
```

### 推定（Estimation）- PID階層型オブザーバー
```python
# 積分状態の更新（Eq. 39）
z_dot = self.p_hat - self.p_star
self.z_integral += z_dot * dt

# 推定位置の更新（Eq. 40）
self.p_hat += self.v_hat * dt

# 推定速度の更新（Eq. 5）
v_hat_dot = (
    - k_p * (self.p_hat - self.p_star)      # P項（位置誤差）
    - k_v * (self.v_hat - self.v_actual)    # D項（速度フィードバック）
    - k_i * self.z_integral                 # I項（積分状態）
    + self.a_star                            # フィードフォワード
    + xi_gain * xi                           # 幾何学的補正項
)
self.v_hat += v_hat_dot * dt
```

### 制御（Control）
```python
# 制御入力の計算（Eq. 6）
u = (
    - k_cp * (self.p_hat - self.p_star)     # 位置制御項（推定位置使用）
    - k_cv * (self.v_actual - self.v_star)  # 速度制御項
    + self.a_star                            # フィードフォワード
    + psi                                    # 共線回避項（現在未使用）
)

# RC値に変換
control_lr = int(np.clip(u[1] * 50.0, -MAX_SPEED, MAX_SPEED))
control_fb = int(np.clip(u[0] * 50.0, -MAX_SPEED, MAX_SPEED))

# 送信
drone.send_rc_control(control_lr, control_fb, 0, 0)
```

## 収束判定の理論

### 理想的な収束条件
```
||p̂_i - p_i^*|| < ε = 0.1m  （推定位置誤差）
AND
||v_i|| < δ = 0.05m/s         （実速度）
⇒ u_i ≈ 0 ⇒ 停止
```

### 止まらない場合の原因と対策

#### 1. 推定位置のずれ
**症状**: `||p̂ - p|| > 0.1m`（推定誤差が大きい）

**確認方法**:
```bash
# CSVログから推定誤差を確認
import pandas as pd
df = pd.read_csv('slaf_results/control_log_*.csv')
print(df['estimation_error'].describe())
```

**対策**:
- 推定器ゲイン調整: `k_p`, `k_v`, `k_i`
- `k_v`を大きくする → 速度フィードバック強化

#### 2. 実速度の推定誤差
**症状**: 数値微分のノイズが大きい

**確認方法**:
```python
# 実速度と推定速度の差
print((df['vx_hat'] - df['vx']).std())
```

**対策**:
- サンプリング周期の見直し
- ローパスフィルタの追加

#### 3. 目標位置の振動
**症状**: `p*`が振動している

**確認方法**:
```python
# 目標位置の変化を確認
print(df['target_x'].diff().abs().max())
```

**対策**:
- `TARGET_STEP_SIZE`を小さくする（0.05m → 0.02m）
- キー入力の遅延処理

## 使用方法

### 基本操作
1. **Q**: 離陸
2. **T**: SLAF制御モード開始（観測・推定・制御を常に実行）
3. **G/B/V/N**: 目標位置移動（押している間、連続的に移動）
4. **キーを離す**: 目標位置が固定され、その場で収束
5. **E**: 着陸

### デバッグ手順
1. **実行**: `python3 mocap_slaf_main.py`
2. **CSVログ確認**: `slaf_results/control_log_*.csv`
3. **推定誤差チェック**: `estimation_error` 列を確認
4. **追跡誤差チェック**: `tracking_error` 列を確認
5. **推定位置vs実位置**: `x_hat` vs `x`, `z_hat` vs `z` をプロット

## 更新されたファイル

### メインプログラム
- `mocap_slaf_main.py`
  - SLAF制御モードの実装変更
  - G/B/V/Nキーによる目標位置移動
  - 常時観測・推定・制御の実行

### CSV出力
- `csv_logger.py`
  - ヘッダーに推定位置・速度を追加
  - `log_slaf_control_data()`関数の更新

### ドキュメント
- `README.md`: 制御モードの説明更新、操作方法更新
- `CONTROL_ALGORITHM.md`: 演算処理の詳細説明（新規作成）
- `UPDATE_SUMMARY_20251124.md`: このファイル（新規作成）

## 参考資料
- `ref/sim_PID_v1/system_dynamics.m`: 理論的な制御式
- `src/MOCAP_for2TELLOs/src2/mocap_for_2tellos.py`: 参考実装

## 今後の課題
1. 推定器ゲインの最適化
2. 速度推定の改善（ローパスフィルタ追加）
3. 共線回避項（ψ）の実装
4. 実機での収束性能の検証
