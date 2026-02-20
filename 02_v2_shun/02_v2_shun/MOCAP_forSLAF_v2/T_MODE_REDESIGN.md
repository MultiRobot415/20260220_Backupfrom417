# Tモード再設計まとめ

**日時**: 2025-11-27  
**変更内容**: 
1. Tモードの初期位置設定を削除し、現在のMOCAP位置・速度を目標として設定
2. k_cv=0の設計を削除し、k_cv=0.1を維持
3. CSVログ出力エラーを修正

---

## 1. Tモードの設計変更

### **変更前（削除された設計）**

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

### **変更後（新設計）**

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

## 2. 制御則の挙動解析

### **Tモード開始時**

#### 状態

- 目標位置: `p* = p_mocap(t_T)`（Tキー押下時のMOCAP位置）
- 目標速度: `v* = [0, 0]`
- 目標加速度: `a* = [0, 0]`
- 推定位置: `p̂(0) = p_mocap(t_T)`（推定誤差ゼロ）
- 推定速度: `v̂(0) = [0, 0]`

#### 制御入力

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

### **Hモード開始時**

#### 状態

- 目標位置: `p*` は加速度積分により更新
- 目標速度: `v* = v* + a* * dt`（積分で更新）
- 目標加速度: `a* = [0, -0.08]` m/s²

#### 制御入力

```
u = -k_cp*(p̂ - p*) - k_cv*(v - v*) + a*
```

**k_cv=0.1を維持することで**:
- 速度項 `-k_cv*(v - v*)` が正しく機能
- 目標軌道への追従性が向上

---

## 3. CSVログ出力エラーの修正

### **問題**

`csv_logger.py`の`log_slaf_control_data()`関数で未定義変数エラー：

```python
# 行を構築
row = [
    timestamp,  # ❌ 未定義
    drone_id,   # ❌ 未定義
    ...
]
```

### **修正**

```python
# データ取得
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
    timestamp,
    drone_id,
    f"follower_{follower_id}",
    mode,
    pos_2d[0], 1.0, pos_2d[1],  # 実位置（3D）
    pos_hat_2d[0], 1.0, pos_hat_2d[1],  # 推定位置（3D）
    ...
]
```

---

## 4. 実装ファイルの変更

### **`mocap_slaf_main.py`**

#### Tキー押下時の処理

```python
# 現在のMOCAP位置・速度を目標として設定（一回のみ取得）
follower_target_positions = {}
follower_target_velocities = {}

if MOCAP_CONNECTED:
    print("  MOCAP接続 - 現在の真の位置・速度を目標として設定")
    for follower_id in [3, 4]:
        tello_id = follower_to_drone_map[follower_id]
        rigid_id = RIGID_BODY_IDS[tello_id]
        
        mocap_pos = ms.get_rigid_body_position(rigid_id)
        if mocap_pos:
            x = mocap_pos.get('x', 0.0)
            z = mocap_pos.get('z', 0.0)
            follower_target_positions[follower_id] = np.array([x, z])
            follower_target_velocities[follower_id] = np.array([0.0, 0.0])
            
            print(f"  フォロワー{follower_id}（ドローン{tello_id+1}）: 目標位置=[{x:.3f}, {z:.3f}], 目標速度=[0.000, 0.000]")
```

#### 推定器初期化

```python
# 推定器を現在位置で初期化（推定誤差ゼロ）
initial_positions = {}
initial_velocities = {}

for follower_id in [3, 4]:
    initial_positions[follower_id] = follower_target_positions[follower_id].copy()
    initial_velocities[follower_id] = follower_target_velocities[follower_id].copy()
    print(f"  フォロワー{follower_id}: p_hat(0)=[{initial_positions[follower_id][0]:.3f}, {initial_positions[follower_id][1]:.3f}], v_hat(0)=[{initial_velocities[follower_id][0]:.3f}, {initial_velocities[follower_id][1]:.3f}]")

slaf_manager.initialize_followers(initial_positions, initial_velocities)
```

#### Hキー押下時の処理

```python
# Hモード時：速度ゲインはそのまま（k_cv=0.1を維持）
# k_cv調整処理は削除
```

---

### **`slaf_pid_controller.py`**

#### `initialize_followers()`メソッドの拡張

```python
def initialize_followers(self, follower_positions, follower_velocities=None):
    """
    フォロワーの推定器を初期化（Assumption: 初期推定誤差零）
    
    Args:
        follower_positions: {follower_id: np.array([x, z]), ...}
        follower_velocities: {follower_id: np.array([vx, vz]), ...} (オプション)
    """
    if follower_velocities is None:
        follower_velocities = {}
    
    for follower_id, controller in self.follower_controllers.items():
        p_initial = follower_positions.get(follower_id, None)
        v_initial = follower_velocities.get(follower_id, None)
        
        if p_initial is not None:
            controller.initialize_state(p_initial, v_initial)
            logger.info(f"フォロワー{follower_id}推定器初期化: p_initial=[...], v_initial=[...]")
```

---

### **`csv_logger.py`**

#### `log_slaf_control_data()`関数の修正

```python
def log_slaf_control_data(log_data):
    global control_csv_writer, control_log_file
    
    try:
        # データ取得（log_dataから全ての必要な変数を取得）
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
            pos_hat_2d[0], 1.0, pos_hat_2d[1],  # 推定位置
            vel_2d[0], 0.0, vel_2d[1],  # 実速度
            vel_hat_2d[0], 0.0, vel_hat_2d[1],  # 推定速度
            target_2d[0], 1.0, target_2d[1],  # 目標位置
            pos_2d[0] - target_2d[0], 0.0, pos_2d[1] - target_2d[1],  # 誤差
            rc_command[0], rc_command[1], rc_command[2], rc_command[3],  # RC
            1.0 if not is_collinear else 0.5,  # trust
            xi_2d[0], 0.0, xi_2d[1],  # xi
            psi_2d[0], 0.0, psi_2d[1],  # psi
            tau, 1 if is_collinear else 0, 1 if is_occluded else 0,
            tracking_error, estimation_error,
            log_data.get('observer_weight_norm', 0.0),
            log_data.get('observer_weight_pattern', 'fixed'),
            log_data.get('k_p', 0.0),
            log_data.get('k_v', 0.0),
            log_data.get('k_cv', 0.0)
        ]
        
        control_csv_writer.writerow(row)
        control_log_file.flush()
```

---

## 5. 期待される動作

### **Tモード開始**

```
============================================================
Tキー検出 - SLAF制御モード開始
============================================================

仮想リーダー（固定位置 - 相対測定基準点）:
  リーダー1: [0.0, 1, -0.5]
  リーダー2: [0.0, 1, 0.5]

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

SLAF制御モード
  H: Hモード開始（目標軌道生成）
  J: Hモード停止
============================================================
```

### **制御ループ実行**

```
[SLAF制御] 飛行: True | Hモード: OFF | 時刻: ...

ドローン1（フォロワー3）:
  実際位置=[0.85, 0.92]
  推定位置=[0.85, 0.92]
  目標位置=[0.85, 0.92]
  制御入力=[0.0000, 0.0000]  ← ほぼゼロ
  RC指令=[lr:0, fb:0]
```

### **Hモード開始**

```
============================================================
Hキー検出 - Hモード開始
目標加速度: [0.000, -0.080] m/s^2
[DEBUG] h_mode_active=True, last_h_mode_update_time=...
============================================================
```

### **Hモード実行**

```
[SLAF制御] 飛行: True | Hモード: ON | 時刻: ...
  目標速度: [0.000, -0.160] m/s

ドローン1（フォロワー3）:
  実際位置=[0.85, 0.88]
  推定位置=[0.85, 0.88]
  目標位置=[0.85, 0.86]
  制御入力=[0.0000, -0.0400]
  RC指令=[lr:-2, fb:0]
```

---

## 6. CSVログ確認

```python
import pandas as pd
df = pd.read_csv('slaf_results/control_log_*.csv')

# 目標位置の確認（Tモード時は現在位置と一致）
print(df[['timestamp', 'x', 'target_x', 'x_hat']].head(10))

# 制御入力の確認（Tモード時はほぼゼロ）
print(df[['timestamp', 'error_x', 'error_z']].head(10))

# 重み行列のノルムの確認
print(df['observer_weight_norm'].unique())  # [2.0]

# ゲイン値の確認（k_cv=0.1を維持）
print(df['k_cv'].unique())  # [0.1]
```

---

## まとめ

### **実装完了項目**

✅ Tモードの設計変更：固定初期位置 → 現在位置  
✅ 目標位置・速度の一回取得実装  
✅ k_cv=0の設計削除（k_cv=0.1を維持）  
✅ CSVログ出力エラー修正  
✅ `initialize_followers()`に速度引数追加  
✅ 推定誤差ゼロの初期化維持  

### **期待される効果**

- **Tモード**: ドローンがほぼ静止（`u ≈ 0`）
- **Hモード**: スムーズに軌道追従開始
- **CSVログ**: 正しくデータ記録
- **制御性能**: k_cv=0.1維持により向上

---

**作成日**: 2025-11-27  
**ステータス**: 実装完了、テスト準備完了
