# Hモード実装の修正まとめ

**日時**: 2025-11-27  
**修正内容**: Hモードが動作しない問題の原因を特定し修正

---

## 問題点の特定

### 1. CSVログ確認結果

```csv
target_x,target_y,target_z
0.5,0.0,-0.5  ← 常に固定（変化なし）
0.5,0.0,-0.5
0.5,0.0,-0.5
...
```

**問題**:
- 目標位置が常に固定値(0.5, 0.0, -0.5)または(0.5, 0.0, 0.5)
- Hモードが常にOFF
- Hキー押下が検出されていない

### 2. 根本原因

#### **原因1: keyboard_control.pyにHキーとJキーがない**

```python
# keyboard_control.py (修正前)
keys_to_check = [
    'a', 'd', 'w', 's',
    'UP', 'DOWN', 'LEFT', 'RIGHT',
    'q', 'e',
    't', 'm',
    'g', 'b', 'v', 'n',  # ← これらは使わない
    'o', 'p',
    'f', 'r',
    'z',
    '1', '2', '0',
    'ESCAPE', 'SPACE'
]
# 'h'と'j'が含まれていない！
```

**結果**: Hキーを押しても`pressed_keys`リストに含まれないため、Hモード開始処理が実行されない

#### **原因2: MOCAP未接続時に制御ループが実行されない**

```python
# mocap_slaf_main.py (修正前)
if control_mode == "slaf" and is_flying and MOCAP_CONNECTED:  # ← MOCAP_CONNECTED=Falseなので実行されない
    # Hモード更新処理
    # 仮想リーダー更新処理
    # フォロワー制御処理
```

**結果**: MOCAP未接続時は制御ループ全体がスキップされるため、Hモード更新も実行されない

#### **原因3: フォーメーションオフセット**

```python
formation_offsets = {
    3: np.array([0.5, 0.0]),  # フォロワー3: リーダー1の後ろ0.5m
    4: np.array([0.5, 0.0])   # フォロワー4: リーダー2の後ろ0.5m
}

# リーダー1目標: [0.0, -0.5] + オフセット[0.5, 0.0] = フォロワー3目標: [0.5, -0.5]
# リーダー2目標: [0.0, 0.5] + オフセット[0.5, 0.0] = フォロワー4目標: [0.5, 0.5]
```

**結果**: CSVログの目標位置(0.5, -0.5)と(0.5, 0.5)は正しい（問題なし）

---

## 修正内容

### 1. keyboard_control.pyの修正

```python
# keyboard_control.py (修正後)
keys_to_check = [
    'a', 'd', 'w', 's',
    'UP', 'DOWN', 'LEFT', 'RIGHT',
    'q', 'e',
    't', 'm',
    'h', 'j',  # ← Hモード開始/停止を追加
    'o', 'p',
    'f', 'r',
    'z',
    '1', '2', '0',
    'ESCAPE', 'SPACE'
]
# 'g', 'b', 'v', 'n'を削除（使用しない）
```

### 2. mocap_slaf_main.pyの修正

#### 修正1: MOCAP未接続時も制御ループを実行

```python
# mocap_slaf_main.py (修正後)
if control_mode == "slaf" and is_flying:  # MOCAP_CONNECTEDの条件を削除
    # Hモード更新処理
    # 仮想リーダー更新処理
    # フォロワー制御処理
```

#### 修正2: MOCAP未接続時の位置取得処理を明示

```python
# MOCAP位置取得と制御
mocap_positions = {}
if MOCAP_CONNECTED:
    # MOCAP接続時：MOCAPから位置取得
    for tello_id in [0, 1]:
        # ... MOCAP位置取得 ...
else:
    # MOCAP未接続時：推定位置を使用
    for tello_id in [0, 1]:
        follower_id = drone_to_follower_map[tello_id]
        controller = slaf_manager.follower_controllers[follower_id]
        mocap_positions[follower_id] = controller.p_actual
```

#### 修正3: デバッグ出力を追加

```python
# Hキー検出
if "h" in pressed_keys:
    print(f"[DEBUG] Hキー検出: control_mode={control_mode}, virtual_leaders={virtual_leaders is not None}, h_mode_active={h_mode_active}")
    if control_mode == "slaf" and virtual_leaders:
        if not h_mode_active:
            print("Hキー検出 - Hモード開始")
            h_mode_active = True
            # ...
            print(f"[DEBUG] h_mode_active={h_mode_active}, last_h_mode_update_time={last_h_mode_update_time}")

# Hモード更新
if h_mode_active and last_h_mode_update_time is not None:
    dt = current_time - last_h_mode_update_time
    old_velocity = target_velocity_2d.copy()
    target_velocity_2d += target_acceleration_2d * dt
    dx = target_velocity_2d[0] * dt
    dz = target_velocity_2d[1] * dt
    virtual_leaders.update_reference_leader_target(dx=dx, dz=dz)
    
    # デバッグ出力（最初の5回のみ）
    if control_drones_thread.h_mode_debug_count < 5:
        print(f"[DEBUG] Hモード更新: dt={dt:.3f}, v_old=[{old_velocity[0]:.4f}, {old_velocity[1]:.4f}], v_new=[{target_velocity_2d[0]:.4f}, {target_velocity_2d[1]:.4f}], dx={dx:.4f}, dz={dz:.4f}")
        control_drones_thread.h_mode_debug_count += 1
```

### 3. Pygameウィンドウの説明テキスト更新

```python
text7 = font.render("H: H-Mode Start (Trajectory Gen) | J: H-Mode Stop", True, (0, 128, 0))
```

---

## 期待される動作

### テストシナリオ

```bash
python mocap_slaf_main.py
# Q（離陸） → T（SLAF開始） → H（Hモード開始） → 5秒待機 → J（停止） → E（着陸）
```

### 期待される出力

1. **Hキー押下時**:
   ```
   押されたキー: ['h']
   [DEBUG] Hキー検出: control_mode=slaf, virtual_leaders=True, h_mode_active=False
   ============================================================
   Hキー検出 - Hモード開始
   目標加速度: [0.000, -0.080] m/s^2
   [DEBUG] h_mode_active=True, last_h_mode_update_time=1764222950.12
   ============================================================
   ```

2. **Hモード更新時（制御ループ内）**:
   ```
   [DEBUG] Hモード更新: dt=0.100, v_old=[0.0000, 0.0000], v_new=[0.0000, -0.0080], dx=0.0000, dz=-0.0008
   [DEBUG] Hモード更新: dt=0.100, v_old=[0.0000, -0.0080], v_new=[0.0000, -0.0160], dx=0.0000, dz=-0.0016
   [DEBUG] Hモード更新: dt=0.100, v_old=[0.0000, -0.0160], v_new=[0.0000, -0.0240], dx=0.0000, dz=-0.0024
   [DEBUG] Hモード更新: dt=0.100, v_old=[0.0000, -0.0240], v_new=[0.0000, -0.0320], dx=0.0000, dz=-0.0032
   [DEBUG] Hモード更新: dt=0.100, v_old=[0.0000, -0.0320], v_new=[0.0000, -0.0400], dx=0.0000, dz=-0.0040
   ```

3. **ステータス表示（2秒ごと）**:
   ```
   [SLAF制御] 飛行: True | Hモード: ON | 時刻: 1764222950.50
     目標速度: [0.000, -0.040] m/s
   
   [SLAF制御] 飛行: True | Hモード: ON | 時刻: 1764222952.50
     目標速度: [0.000, -0.160] m/s
   
   [SLAF制御] 飛行: True | Hモード: ON | 時刻: 1764222954.50
     目標速度: [0.000, -0.280] m/s
   ```

4. **CSVログ確認**:
   ```csv
   timestamp,target_x,target_y,target_z
   1764222950.5,0.5,0.0,-0.500
   1764222951.0,0.5,0.0,-0.504  ← 目標位置が変化
   1764222951.5,0.5,0.0,-0.512  ← 加速度積分で変化量増加
   1764222952.0,0.5,0.0,-0.524
   1764222952.5,0.5,0.0,-0.540
   ```

5. **Jキー押下時**:
   ```
   押されたキー: ['j']
   ============================================================
   Jキー検出 - Hモード停止
   ============================================================
   ```

---

## 修正ファイル

1. **`keyboard_control.py`**:
   - `keys_to_check`リストに'h'と'j'を追加
   - 'g', 'b', 'v', 'n'を削除
   - Pygameウィンドウの説明テキスト更新

2. **`mocap_slaf_main.py`**:
   - MOCAP未接続時も制御ループを実行
   - MOCAP未接続時の位置取得処理を明示化
   - Hキー/Jキーのデバッグ出力を追加
   - Hモード更新のデバッグ出力を追加

---

## チェックリスト

✅ keyboard_control.pyに'h'と'j'を追加  
✅ MOCAP未接続時も制御ループを実行  
✅ デバッグ出力を追加  
✅ Pygameウィンドウの説明更新  
⏳ 実機テスト（ユーザー確認待ち）

---

**作成日**: 2025-11-27  
**ステータス**: 修正完了、テスト準備完了
