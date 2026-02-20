# エラー修正まとめ - UnboundLocalError対応

**日時**: 2025-11-27  
**修正内容**: Hモード起動時のUnboundLocalErrorと制御入力の問題を修正

---

## エラー内容

### 1. UnboundLocalError

```
Traceback (most recent call last):
  File "mocap_slaf_main.py", line 500, in control_drones_thread
    print(f"  目標速度: [{target_velocity_2d[0]:.3f}, {target_velocity_2d[1]:.3f}] m/s")
UnboundLocalError: local variable 'target_velocity_2d' referenced before assignment
```

**発生タイミング**: Hモード起動時、2秒ごとのステータス表示

### 2. Tモード時の制御入力

- ユーザー報告：「Tモードにしたときも制御入力が入っていないように見える」

---

## 原因分析

### 1. UnboundLocalErrorの原因

**問題のコード**:
```python
def control_drones_thread():
    """ドローン制御スレッド"""
    global should_stop, slaf_mode_enabled, is_flying
    # target_velocity_2dがglobal宣言されていない！
    
    while not should_stop:
        # ...
        if h_mode_active:
            print(f"  目標速度: [{target_velocity_2d[0]:.3f}, ...]")  # ❌ エラー
        
        # Hモード更新
        global last_h_mode_update_time, follower_target_positions  # ❌ 遅すぎる
        if h_mode_active and last_h_mode_update_time is not None:
            target_velocity_2d += target_acceleration_2d * dt  # ❌ ここで代入するとローカル変数になる
```

**問題点**:
1. `target_velocity_2d`が関数の最初で`global`宣言されていない
2. ステータス表示部分で`target_velocity_2d`を参照
3. その後のHモード更新で`target_velocity_2d +=`で代入
4. Pythonは代入がある変数をローカル変数と判断
5. 参照時点ではまだ代入前 → `UnboundLocalError`

**Pythonの変数スコープ規則**:
- 関数内で変数に代入(`=`, `+=`など)があると、その変数はローカル変数と判断される
- ローカル変数は関数の最初から存在するとみなされる
- `global`宣言がない場合、参照時に未定義エラー

### 2. 制御入力が入らない可能性

**考えられる原因**:
1. MOCAP未接続時の位置データ取得
2. 目標位置が実際位置と同じ（推定誤差ゼロ初期化）
3. 制御ゲインの問題

---

## 修正内容

### 1. グローバル変数宣言の修正

**修正前**:
```python
def control_drones_thread():
    global should_stop, slaf_mode_enabled, is_flying
    # target_velocity_2dなどが宣言されていない
    
    # ... 途中で ...
    global last_h_mode_update_time, follower_target_positions  # ❌ 遅すぎる
```

**修正後**:
```python
def control_drones_thread():
    global should_stop, slaf_mode_enabled, is_flying
    global h_mode_active, target_velocity_2d, target_acceleration_2d, last_h_mode_update_time, follower_target_positions
    # ✅ 関数の最初で全てのHモード関連グローバル変数を宣言
    
    # ... Hモード更新部分 ...
    if h_mode_active and last_h_mode_update_time is not None:
        # ✅ 途中のglobal宣言を削除（既に宣言済み）
        target_velocity_2d += target_acceleration_2d * dt
```

### 2. Tキー押下時のHモードリセット

**修正前**:
```python
# Tキー押下時
global follower_target_positions
follower_target_positions = {...}
# Hモードはリセットされない
```

**修正後**:
```python
# Tキー押下時
global follower_target_positions, target_velocity_2d, h_mode_active, last_h_mode_update_time
follower_target_positions = {
    3: FOLLOWER_INITIAL_TARGETS[3].copy(),
    4: FOLLOWER_INITIAL_TARGETS[4].copy()
}
# Hモードをリセット
h_mode_active = False
target_velocity_2d = np.array([0.0, 0.0])
last_h_mode_update_time = None
print("  Hモードをリセット")
```

### 3. デバッグ出力の追加

#### 制御入力のデバッグ

```python
# SLAF制御更新（観測・推定・制御）
control_inputs = slaf_manager.update_followers(mocap_positions, leader_states_2d)

# デバッグ：制御入力確認（最初の5回のみ）
if not hasattr(control_drones_thread, 'control_debug_count'):
    control_drones_thread.control_debug_count = 0
if control_drones_thread.control_debug_count < 5:
    print(f"[DEBUG] 制御入力: {control_inputs}")
    control_drones_thread.control_debug_count += 1
```

#### 詳細なステータス表示

**修正前**:
```python
print(f"ドローン{tello_id+1}: 実際位置=[...], 推定位置=[...], 目標位置=[...], 制御値=[lr:{control_lr}, fb:{control_fb}]")
```

**修正後**:
```python
print(f"ドローン{tello_id+1}（フォロワー{follower_id}）:")
print(f"  実際位置=[{state['p_actual'][0]:.2f}, {state['p_actual'][1]:.2f}]")
print(f"  推定位置=[{state['p_hat'][0]:.2f}, {state['p_hat'][1]:.2f}]")
print(f"  目標位置=[{state['p_star'][0]:.2f}, {state['p_star'][1]:.2f}]")
print(f"  制御入力=[{u_2d[0]:.4f}, {u_2d[1]:.4f}]")  # ← 追加
print(f"  RC指令=[lr:{control_lr}, fb:{control_fb}]")
```

---

## 修正後の動作フロー

### Tキー押下時

```
1. フォロワー目標位置を初期値に設定
   - フォロワー3: [1.0, 0.8]
   - フォロワー4: [0.0, 0.8]

2. Hモードをリセット
   - h_mode_active = False
   - target_velocity_2d = [0.0, 0.0]
   - last_h_mode_update_time = None

3. 推定器を初期化（推定誤差ゼロ）
   - MOCAP接続: MOCAP位置で初期化
   - MOCAP未接続: 目標位置[1.0, 0.8], [0.0, 0.8]で初期化

4. SLAF制御モード開始
```

### Hキー押下時

```
1. h_mode_active = True
2. target_velocity_2d = [0.0, 0.0]
3. last_h_mode_update_time = 現在時刻

出力:
============================================================
Hキー検出 - Hモード開始
目標加速度: [0.000, -0.080] m/s^2
[DEBUG] h_mode_active=True, last_h_mode_update_time=...
============================================================
```

### 制御ループ（0.1秒ごと）

```
1. Hモード更新（h_mode_active=Trueの場合）
   dt = 現在時刻 - last_h_mode_update_time
   target_velocity_2d += [0.0, -0.08] * dt
   follower_target_positions[3] += target_velocity_2d * dt
   follower_target_positions[4] += target_velocity_2d * dt

2. 仮想リーダー更新（固定位置）

3. フォロワー目標設定
   follower_targets[3] = {
       'position': follower_target_positions[3],
       'velocity': target_velocity_2d if h_mode_active else [0, 0],
       'acceleration': target_acceleration_2d if h_mode_active else [0, 0]
   }

4. MOCAP位置取得（MOCAP接続時）または推定位置使用（未接続時）

5. SLAF制御更新
   control_inputs = slaf_manager.update_followers(mocap_positions, leader_states_2d)

6. RC指令生成と送信
   control_lr = int(u_2d[1] * 50.0)  # z方向→左右
   control_fb = int(u_2d[0] * 50.0)  # x方向→前後
   drone.send_rc_control(control_lr, control_fb, 0, 0)
```

### ステータス表示（2秒ごと）

```
[SLAF制御] 飛行: True | Hモード: ON | 時刻: 1764224140.00
  目標速度: [0.000, -0.160] m/s

ドローン1（フォロワー3）:
  実際位置=[0.84, 0.91]
  推定位置=[0.84, 0.91]
  目標位置=[1.00, 0.79]
  制御入力=[0.0320, 0.0240]
  RC指令=[lr:1, fb:1]

ドローン2（フォロワー4）:
  実際位置=[0.06, 0.89]
  推定位置=[0.06, 0.89]
  目標位置=[0.00, 0.79]
  制御入力=[-0.0120, 0.0200]
  RC指令=[lr:1, fb:-0]
```

---

## src/CBF_for2TELLOsとの比較

### src/CBF_for2TELLOs

```python
# メインループ
global target_moving, last_target_update_time, target_positions

if target_moving:
    if last_target_update_time is not None:
        dt = current_time - last_target_update_time
        for i in range(len(target_positions)):
            target_positions[i][0] += target_move_velocity[0] * dt
            target_positions[i][1] += target_move_velocity[1] * dt
            target_positions[i][2] += target_move_velocity[2] * dt
        for i, ctrl in enumerate(controllers):
            ctrl.set_target_position(*target_positions[i])
    last_target_update_time = current_time
```

**特徴**:
- `target_moving`フラグで制御
- 固定速度`target_move_velocity`で移動
- 各ドローンの目標位置を独立更新

### MOCAP_forSLAF（修正後）

```python
# メインループ
global h_mode_active, target_velocity_2d, target_acceleration_2d, last_h_mode_update_time, follower_target_positions

if h_mode_active and last_h_mode_update_time is not None:
    dt = current_time - last_h_mode_update_time
    # 目標速度を更新（加速度から積分）
    target_velocity_2d += target_acceleration_2d * dt
    # 各フォロワーの目標位置を更新（速度から積分）
    for follower_id in [3, 4]:
        follower_target_positions[follower_id][0] += target_velocity_2d[0] * dt
        follower_target_positions[follower_id][1] += target_velocity_2d[1] * dt
if h_mode_active:
    last_h_mode_update_time = current_time
```

**差異**:
- `h_mode_active`フラグで制御（同じ概念）
- **速度は加速度から積分**（src/CBF_for2TELLOsは速度固定）
- 目標位置は速度から積分（同じ）
- 各フォロワーの目標位置を独立更新（同じ）

---

## 期待される結果

### Tモード起動時

```
============================================================
Tキー検出 - SLAF制御モード開始
============================================================

仮想リーダー（固定位置 - 相対測定基準点）:
  リーダー1: [0.0, 1, -0.5]
  リーダー2: [0.0, 1, 0.5]

フォロワー（実機ドローン）目標位置を初期位置に設定:
  フォロワー3（ドローン1）: 目標位置=[1.0, 0.8]
  フォロワー4（ドローン2）: 目標位置=[0.0, 0.8]
  Hモードをリセット
目標位置設定完了

推定器初期化中（Assumption: 初期推定誤差零）...
  MOCAP未接続 - 目標位置で推定器を初期化します
  フォロワー3（ドローン1）: p_initial=[1.000, 0.800]（目標位置）
  フォロワー4（ドローン2）: p_initial=[0.000, 0.800]（目標位置）
  推定器を目標位置で初期化しました（推定誤差ゼロ）

SLAF制御モード
  H: Hモード開始（目標軌道生成）
  J: Hモード停止
============================================================
```

### Hモード起動後

```
[DEBUG] Hモード更新: dt=0.100, v_new=[0.0000, -0.0080], dx=0.0000, dz=-0.0008
[DEBUG]   フォロワー3目標: [1.0000, 0.7992]
[DEBUG]   フォロワー4目標: [0.0000, 0.7992]

[DEBUG] 制御入力: {3: array([0.032, 0.024]), 4: array([-0.012, 0.020])}

[SLAF制御] 飛行: True | Hモード: ON | 時刻: 1764224140.50
  目標速度: [0.000, -0.040] m/s

ドローン1（フォロワー3）:
  実際位置=[1.00, 0.80]
  推定位置=[1.00, 0.80]
  目標位置=[1.00, 0.79]
  制御入力=[0.0000, 0.0200]
  RC指令=[lr:1, fb:0]
```

---

## チェックリスト

✅ UnboundLocalErrorを修正（グローバル変数宣言）  
✅ Tキー押下時にHモードをリセット  
✅ 制御入力のデバッグ出力を追加  
✅ 詳細なステータス表示を追加  
✅ src/CBF_for2TELLOsの設計を参考  
⏳ 実機テスト（ユーザー確認待ち）

---

**作成日**: 2025-11-27  
**ステータス**: エラー修正完了、デバッグ出力追加済み
