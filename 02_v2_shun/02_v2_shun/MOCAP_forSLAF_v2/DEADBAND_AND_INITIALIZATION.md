# 不感帯処理・初期化・座標系対応の説明

## 1. 不感帯処理（Deadband Processing）

### 概要
推定位置と目標位置がほぼ一致している時に、微小な制御入力が出力され続けることで機体が流れる現象を防ぐための処理。

### 実装（src/MOCAP_for2TELLOsに合わせて実装）

```python
# slaf_pid_controller.py
class SLAFPIDController:
    def __init__(self, ...):
        # 不感帯パラメータ
        self.deadband_x = 0.0  # x方向（前後）の不感帯 (m)
        self.deadband_z = 0.0  # z方向（左右）の不感帯 (m)
    
    def calculate_control_input(self, psi=None):
        # 位置誤差を計算
        position_error = self.p_hat - self.p_star
        
        # 不感帯処理：不感帯内の誤差はゼロにする
        if abs(position_error[0]) < self.deadband_x:
            position_error[0] = 0.0
        if abs(position_error[1]) < self.deadband_z:
            position_error[1] = 0.0
        
        # 制御入力計算（不感帯適用済み誤差を使用）
        u = - self.k_cp * position_error - self.k_cv * (self.v_actual - self.v_star) + ...
```

### 使用方法

#### デフォルト設定（不感帯なし）
```python
# デフォルトは deadband_x = 0.0, deadband_z = 0.0
# つまり、不感帯処理は無効（全ての誤差が制御に反映される）
```

#### 不感帯を設定する場合
```python
# 例: 前後方向0.05m、左右方向0.05mの不感帯を設定
slaf_manager.follower_controllers[3].set_deadbands(deadband_x=0.05, deadband_z=0.05)
slaf_manager.follower_controllers[4].set_deadbands(deadband_x=0.05, deadband_z=0.05)
```

### 推奨値
- **初期テスト**: `deadband_x = 0.0, deadband_z = 0.0`（不感帯なし）
  - まず不感帯なしで動作確認し、機体の流れ具合を確認
- **微調整後**: `deadband_x = 0.03〜0.05, deadband_z = 0.03〜0.05`
  - 機体が流れる場合、不感帯を徐々に増やす
  - 大きすぎると目標位置への収束が遅くなるため注意

---

## 2. 初期位置・初期推定値の設計

### 理論的背景
PID階層型SLAF制御では、推定器の初期誤差をゼロと仮定：
- **p̂_i(0) = p_i(0)**: 初期推定位置 = 初期実位置（MOCAP測定値）
- **v̂_i(0) = v_i(0)**: 初期推定速度 = 初期実速度（通常ゼロ）
- **z_i(0) = 0**: 積分項の初期値はゼロ

### 実装
```python
# slaf_pid_controller.py
def initialize_state(self, p_initial, v_initial=None):
    """
    初期状態を設定（Assumption: 初期推定誤差零）
    
    Args:
        p_initial: 初期位置 [x, z]（MOCAPから取得）
        v_initial: 初期速度 [vx, vz]（Noneの場合はゼロ）
    """
    # 実位置・実速度をMOCAP測定値で初期化
    self.p_actual = np.array(p_initial, dtype=float)
    self.v_actual = np.array(v_initial, dtype=float) if v_initial is not None else np.zeros(2)
    
    # Assumption: 初期推定誤差ゼロ
    self.p_hat = self.p_actual.copy()  # p̂_i(0) = p_i(0)
    self.v_hat = self.v_actual.copy()  # v̂_i(0) = v_i(0)
    
    # 積分項をゼロで初期化
    self.z_integral = np.zeros(2)      # z_i(0) = 0
```

### 初期化のタイミング
```python
# mocap_slaf_main.py - Tキー押下時
if "t" in pressed_keys:
    # ... 仮想リーダーの目標位置を設定 ...
    
    # フォロワーの推定器を初期化
    # 注意：実装では、推定器は自動的にゼロから開始される
    # MOCAPから取得した位置で初期化する場合は、以下を実行：
    # slaf_manager.initialize_followers_with_positions(mocap_positions)
```

### 重要な注意点
1. **初期化なしの場合**: 推定値はゼロからスタート → 大きな制御入力が発生し、不安定になる可能性
2. **MOCAP位置で初期化**: 推定誤差ゼロからスタート → 安定した制御開始
3. **現在の実装**: ゼロ初期化（プロトタイプに合わせて）

---

## 3. MOCAP座標系とコードの対応

### 座標系の定義

| 座標軸 | MOCAP | コード（SLAF） | Tello RC |
|--------|-------|----------------|----------|
| **X軸** | 前後方向 | `p_actual[0]`, `p_hat[0]` | `forward_backward` |
| **Y軸** | 高度（上下） | *(使用しない、一定と仮定)* | `up_down` |
| **Z軸** | 左右方向 | `p_actual[1]`, `p_hat[1]` | `left_right` |

### MOCAP → コード
```python
# mocap_slaf_main.py
mocap_pos = ms.get_rigid_body_position(rigid_id)
if mocap_pos:
    x = mocap_pos.get('x', 0.0)  # 前後方向
    z = mocap_pos.get('z', 0.0)  # 左右方向
    # y軸（高度）は使用しない（一定高度を仮定）
    mocap_positions[follower_id] = np.array([x, z])
```

### コード → Tello RC
```python
# mocap_slaf_main.py
# 制御値をRC値に変換
velocity_gain = 50.0
control_lr = int(np.clip(u_2d[1] * velocity_gain, -MAX_SPEED, MAX_SPEED))  # z方向→左右
control_fb = int(np.clip(u_2d[0] * velocity_gain, -MAX_SPEED, MAX_SPEED))  # x方向→前後
control_ud = 0  # 高度は一定
control_yaw = 0

# Tello SDKへ送信
drone.send_rc_control(control_lr, control_fb, control_ud, control_yaw)
# send_rc_control(left_right, forward_backward, up_down, yaw)
```

### src/MOCAP_for2TELLOsとの対応
✅ **同じMOCAPシステムを使用**しているため、座標軸の意味は同じ
✅ **同じTello SDKを使用**しているため、RC値の対応も同じ
✅ **SLAF版は2次元（x-z平面）**、MOCAP_for2TELLOsは3次元（x-y-z）

---

## 4. CSVログでの確認方法

### 不感帯処理の効果確認
```csv
# control_log_*.csv
timestamp,x,x_hat,target_x,rc_fb,...
1234.5,0.51,0.50,0.50,2,...  # 不感帯なし: 誤差0.01m → rc_fb=2
1234.5,0.51,0.50,0.50,0,...  # 不感帯0.05m: 誤差0.01m < 0.05m → rc_fb=0
```

**確認ポイント**:
- 推定位置と目標位置がほぼ一致している時（誤差 < 不感帯）
- RC値（`rc_lr`, `rc_fb`）がゼロになっているか

### 初期化の確認
```csv
# Tキー押下直後の最初のログ
timestamp,x,x_hat,target_x,...
1234.0,0.68,0.68,0.50,...  # x_hat = x（初期推定誤差ゼロ）
```

**確認ポイント**:
- Tキー押下直後の`x_hat`が`x`（MOCAP位置）と一致しているか
- または`x_hat = 0.0`でゼロ初期化されているか

---

## 5. トラブルシューティング

### Q1. 機体が流れ続ける（ホバリングしない）
**原因**: 不感帯が小さすぎて、微小な誤差が常に制御入力を生成している
**対策**: 不感帯を徐々に増やす（0.03m → 0.05m → 0.08m）

### Q2. 目標位置への収束が遅い
**原因**: 不感帯が大きすぎる
**対策**: 不感帯を徐々に減らす

### Q3. Tキー押下直後に機体が暴走する
**原因**: 初期推定値がゼロで、MOCAP位置と大きく乖離している
**対策**: MOCAP位置で推定器を初期化する（`initialize_state(mocap_position)`を実行）

### Q4. 座標が逆転している
**原因**: MOCAP座標とTello RC座標の対応ミス
**確認**: 
1. ログで`x`, `z`と`rc_fb`, `rc_lr`の符号を確認
2. `control_fb = u_2d[0]`（x方向）、`control_lr = u_2d[1]`（z方向）が正しいか確認

---

## 6. 参考: MOCAP_for2TELLOsとの比較

| 項目 | MOCAP_for2TELLOs | SLAF版 |
|------|------------------|---------|
| **不感帯** | あり（`position_control.py`） | 追加実装（デフォルト0.0） |
| **座標系** | 3次元（x, y, z） | 2次元（x, z）※y軸は一定 |
| **初期化** | コントローラー内で実行 | 推定器はゼロ初期化（プロトタイプに合わせて） |
| **RC変換** | `gain * 100` | `gain * 50.0` |

---

## 更新履歴
- **2025-11-26**: 不感帯処理を追加実装、座標系対応を文書化
