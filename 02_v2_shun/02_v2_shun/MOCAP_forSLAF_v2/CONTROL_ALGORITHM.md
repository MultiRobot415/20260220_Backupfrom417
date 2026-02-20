# SLAF制御アルゴリズムの詳細

## 概要
このドキュメントでは、PID階層型SLAF制御システムの演算処理の詳細を説明します。

## 制御アーキテクチャ

### 全体構造
```
仮想リーダー（固定位置）
    ↓ 相対測定
フォロワー（実機ドローン）
    ↓ 観測・推定・制御
制御コマンド送信
```

## 1. 観測（Observation）

### MOCAP位置取得
```python
mocap_pos = ms.get_rigid_body_position(rigid_id)
x = mocap_pos.get('x', 0.0)
z = mocap_pos.get('z', 0.0)
p_actual = np.array([x, z])  # 2D位置（x, z平面）
```

**処理内容：**
- モーションキャプチャシステムから剛体の位置を取得
- 3D座標（x, y, z）から2D座標（x, z）に変換
- y軸（高度）は別途管理

## 2. 推定（Estimation）- PID階層型オブザーバー

### 2.1 推定器の状態変数
```python
# slaf_pid_controller.py の SLAFPIDController クラス
self.p_hat = np.zeros(2)      # 推定位置 [x̂, ẑ]
self.v_hat = np.zeros(2)      # 推定速度 [v̂x, v̂z]
self.z_integral = np.zeros(2) # 積分状態 [z_x, z_z]
```

### 2.2 推定器の動作（毎制御周期で実行）

#### ステップ1: 速度推定（数値微分）
```python
# 実速度の推定（位置の差分から）
if self.prev_time is not None:
    dt = current_time - self.prev_time
    self.v_actual = (p_actual - self.prev_position) / dt
else:
    self.v_actual = np.zeros(2)
```

#### ステップ2: PID型推定器の更新
```python
# 積分状態の更新（Eq. 39）
z_dot = self.p_hat - self.p_star
self.z_integral += z_dot * self.dt

# 推定位置の更新（Eq. 40）
p_hat_dot = self.v_hat
self.p_hat += p_hat_dot * self.dt

# 推定速度の更新（Eq. 5）
v_hat_dot = (
    - self.k_p * (self.p_hat - self.p_star)      # P項（位置誤差）
    - self.k_v * (self.v_hat - self.v_actual)    # D項（速度フィードバック）
    - self.k_i * self.z_integral                 # I項（積分状態）
    + self.a_star                                # フィードフォワード
    + self.xi_gain * xi                          # 幾何学的補正項
)
self.v_hat += v_hat_dot * self.dt
```

**理論的背景（ref/sim_PID_v1/system_dynamics.m）：**
- **P項**: 目標位置との誤差を補正
- **D項**: 実速度との差を補正（外乱抑制）
- **I項**: 定常偏差を除去
- **フィードフォワード**: 目標加速度を予測
- **幾何学的補正項**: ベアリング誤差を補正

### 2.3 幾何学的補正項（ξ）の計算

```python
def calculate_xi(self, neighbor_positions, leader_positions):
    """
    ベアリングベース協調制御の補正項を計算
    
    Args:
        neighbor_positions: 隣接フォロワーの位置 {id: [x, z]}
        leader_positions: リーダーの位置 [leader1_2d, leader2_2d]
    
    Returns:
        xi: 幾何学的補正項 [ξ_x, ξ_z]
    """
    # ベアリング誤差の計算
    # g_ij = (p_j - p_i) / ||p_j - p_i||  (単位ベクトル)
    # xi = Σ W_ij * (g_ij - g_ij*)  (重み付き補正)
```

**物理的意味：**
- 隣接エージェントとの相対位置（ベアリング）を目標値に近づける
- フォーメーション形成と維持に寄与

## 3. 制御（Control）

### 3.1 制御入力の計算（Eq. 6）
```python
def calculate_control_input(self):
    """
    制御入力の計算
    
    u_i = -k_cp(p̂_i - p_i^*) - k_cv(v_i - v_i^*) + a_i^* + ψ_i
    """
    u = (
        - self.k_cp * (self.p_hat - self.p_star)  # 位置制御項（推定位置使用）
        - self.k_cv * (self.v_actual - self.v_star)  # 速度制御項
        + self.a_star  # フィードフォワード
        + psi  # 共線回避項（現在は未使用）
    )
    return u
```

**重要なポイント：**
- **推定位置（p̂）を使用**: 実位置（p）ではなく推定位置を使う
- **これにより**: ノイズの影響を低減し、滑らかな制御を実現
- **目標到達条件**: `||p̂_i - p_i^*|| < ε` かつ `||v_i|| < δ` で停止

### 3.2 RC値への変換
```python
# 制御入力 u [m/s^2] を RC値 [-100, 100] に変換
velocity_gain = 50.0
control_lr = int(np.clip(u_2d[1] * velocity_gain, -MAX_SPEED, MAX_SPEED))
control_fb = int(np.clip(u_2d[0] * velocity_gain, -MAX_SPEED, MAX_SPEED))
```

### 3.3 コマンド送信
```python
# Telloへ送信
drone.send_rc_control(control_lr, control_fb, control_ud, control_yaw)
# send_rc_control(left_right, forward_backward, up_down, yaw)
```

## 4. 目標位置の更新

### 4.1 仮想リーダーの目標位置（固定 + キー操作）
```python
# G/B/V/Nキーで目標位置を移動
if "g" in pressed_keys:  # 前進
    virtual_leaders.update_reference_leader_target(dx=0.05)
if "b" in pressed_keys:  # 後退
    virtual_leaders.update_reference_leader_target(dx=-0.05)
```

### 4.2 フォロワーの目標位置（フォーメーションオフセット適用）
```python
# フォロワーの目標位置 = リーダー目標位置 + オフセット
formation_offsets = {
    3: np.array([0.5, 0.0]),  # フォロワー3: リーダー1の後ろ0.5m
    4: np.array([0.5, 0.0])   # フォロワー4: リーダー2の後ろ0.5m
}
target_pos = leader_target_2d + formation_offsets[follower_id]
```

## 5. 収束判定（理論）

### キーを離した時の挙動
1. **目標位置は変化しない**: G/B/V/Nキーを離すと `p_i^*` は固定
2. **制御は継続実行**: SLAF制御は常に動作
3. **収束条件**:
   ```
   ||p̂_i - p_i^*|| < ε  （推定位置誤差が小さい）
   AND
   ||v_i|| < δ          （実速度が小さい）
   ```
4. **制御入力がゼロに**: 上記条件を満たすと `u ≈ 0` となり停止

### 止まらない場合の原因
1. **推定位置（p̂）がずれている**: 
   - PID推定器のゲインが不適切
   - ノイズの影響
2. **実速度（v）の推定誤差**:
   - 数値微分のノイズ
   - サンプリング周期の問題
3. **目標位置の振動**:
   - キー入力の遅延
   - フォーメーションオフセットの誤差

## 6. CSVログ出力

### ログデータ（1秒ごと）
```python
log_data = {
    'timestamp': time.time(),
    'drone_id': tello_id,
    'follower_id': follower_id,
    'mode': 'slaf',
    'position': state['p_actual'],        # 実位置（MOCAP）
    'position_hat': state['p_hat'],       # 推定位置（オブザーバー）
    'target_position': state['p_star'],   # 目標位置
    'velocity': state['v_actual'],        # 実速度（数値微分）
    'velocity_hat': state['v_hat'],       # 推定速度（オブザーバー）
    'control_input': u_2d,                # 制御入力 [u_x, u_z]
    'rc_command': [lr, fb, ud, yaw],      # RC値
    'tracking_error': ||p_actual - p_star||,     # 追跡誤差
    'estimation_error': ||p_hat - p_actual||     # 推定誤差
}
```

### CSVファイル形式
```
slaf_results/control_log_YYYYMMDD_HHMMSS.csv
```

**重要な列：**
- `x`, `z`: 実位置（MOCAP）
- `x_hat`, `z_hat`: 推定位置（オブザーバー）
- `vx`, `vz`: 実速度
- `vx_hat`, `vz_hat`: 推定速度
- `target_x`, `target_z`: 目標位置
- `tracking_error`: 追跡誤差ノルム
- `estimation_error`: 推定誤差ノルム

## 7. デバッグ方法

### 推定位置の確認
```bash
# CSVファイルから推定位置と実位置を比較
python -c "
import pandas as pd
df = pd.read_csv('slaf_results/control_log_*.csv')
print('実位置vs推定位置の差:')
print((df['x_hat'] - df['x']).describe())
print((df['z_hat'] - df['z']).describe())
"
```

### リアルタイムデバッグ（2秒ごとに表示）
```
ドローン1: 実際位置=[0.12, 0.45], 推定位置=[0.13, 0.46], 目標位置=[0.00, 0.00], 制御値=[lr:-65, fb:60]
```

**確認項目：**
1. 推定位置と実位置の差が大きくないか（< 0.1m）
2. 目標位置に近づいているか
3. 制御値が飽和していないか（|RC| < 50）

## パラメータ調整

### 推定器ゲイン（slaf_pid_controller.py）
```python
self.k_p = 3.0   # P項ゲイン（位置誤差）
self.k_v = 2.0   # D項ゲイン（速度フィードバック）
self.k_i = 0.5   # I項ゲイン（積分）
```

**調整指針：**
- `k_p`を大きく → 応答速度向上、振動リスク
- `k_v`を大きく → ダンピング向上、遅くなる
- `k_i`を大きく → 定常偏差除去、振動リスク

### 制御器ゲイン
```python
self.k_cp = 2.0  # 位置制御ゲイン
self.k_cv = 1.0  # 速度制御ゲイン
```

**調整指針：**
- `k_cp`を大きく → 追従性向上、振動リスク
- `k_cv`を大きく → ダンピング向上
