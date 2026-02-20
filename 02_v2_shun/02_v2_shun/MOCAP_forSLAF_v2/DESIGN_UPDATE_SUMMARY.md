# 設計変更まとめ - オフセット機能削除と初期位置設定

**日時**: 2025-11-27  
**変更内容**: 仮想リーダーからのオフセット機能を削除、各フォロワーの初期目標位置を独立設定

---

## 設計変更の理由

### ユーザー要求

1. **仮想リーダーからのオフセット機能を削除**
   - 目標位置はリーダーからのオフセットではなく、初期位置から速度・加速度積分で生成
   - 各フォロワーは独立した目標位置を持つ

2. **初期位置の設定（Tキー押下時）**
   - ドローン1（フォロワー3）: `[1.0, 1.0, 0.8]` (x, y, z) → 2Dでは `[1.0, 0.8]` (x, z)
   - ドローン2（フォロワー4）: `[0.0, 1.0, 0.8]` (x, y, z) → 2Dでは `[0.0, 0.8]` (x, z)

3. **初期推定位置**
   - 推定誤差ゼロ（論文準拠）
   - MOCAP接続時：MOCAP位置で初期化
   - MOCAP未接続時：目標位置で初期化

4. **仮想リーダー**
   - 引き続き固定（観測に使用）
   - 削除しない

---

## 修正前の設計（問題点）

### 目標位置の計算方法（修正前）

```python
# formation_offsetsを使用
formation_offsets = {
    3: np.array([0.5, 0.0]),  # リーダー1の後ろ0.5m
    4: np.array([0.5, 0.0])   # リーダー2の後ろ0.5m
}

for follower_id in [3, 4]:
    leader_idx = 0 if follower_id == 3 else 1
    leader_target_2d = leader_states_2d[leader_idx]['target_position']
    target_pos = leader_target_2d + formation_offsets[follower_id]  # オフセット適用
```

**問題**:
- リーダーの目標位置 + オフセット = フォロワー目標位置
- 各フォロワーの初期位置が反映されない
- Hモードで速度・加速度を積分しても、常にリーダーとのオフセットが維持される

### 初期位置の設定（修正前）

```python
# リーダーの目標位置を設定
virtual_leaders.set_reference_leader_target(
    x=VIRTUAL_LEADER_POSITIONS[0][0],  # 0.0
    y=VIRTUAL_LEADER_POSITIONS[0][1],  # 1.0
    z=VIRTUAL_LEADER_POSITIONS[0][2]   # -0.5
)

# フォロワーの目標位置は自動計算される（リーダー + オフセット）
# フォロワー3: [0.0, -0.5] + [0.5, 0.0] = [0.5, -0.5]
# フォロワー4: [0.0, 0.5] + [0.5, 0.0] = [0.5, 0.5]
```

**問題**:
- ユーザー希望の初期位置`[1.0, 0.8]`, `[0.0, 0.8]`にならない
- 固定されたオフセット計算

---

## 修正後の設計

### 1. グローバル変数の追加

```python
# フォロワー初期目標位置（2D: x, z）
# Tキー押下時に設定される初期位置
FOLLOWER_INITIAL_TARGETS = {
    3: np.array([1.0, 0.8]),  # ドローン1（フォロワー3）: [x=1.0m, z=0.8m]
    4: np.array([0.0, 0.8])   # ドローン2（フォロワー4）: [x=0.0m, z=0.8m]
}

# 各フォロワーの現在の目標位置（Hモードで更新される）
follower_target_positions = {
    3: np.array([1.0, 0.8]),
    4: np.array([0.0, 0.8])
}
```

### 2. Tキー押下時の処理（修正後）

```python
# フォロワー目標位置を初期位置に設定
print("\nフォロワー（実機ドローン）目標位置を初期位置に設定:")
global follower_target_positions
follower_target_positions = {
    3: FOLLOWER_INITIAL_TARGETS[3].copy(),  # ドローン1: [1.0, 0.8]
    4: FOLLOWER_INITIAL_TARGETS[4].copy()   # ドローン2: [0.0, 0.8]
}
print(f"  フォロワー3（ドローン1）: 目標位置=[{follower_target_positions[3][0]:.1f}, {follower_target_positions[3][1]:.1f}]")
print(f"  フォロワー4（ドローン2）: 目標位置=[{follower_target_positions[4][0]:.1f}, {follower_target_positions[4][1]:.1f}]")
print("目標位置設定完了")
```

**出力例**:
```
フォロワー（実機ドローン）目標位置を初期位置に設定:
  フォロワー3（ドローン1）: 目標位置=[1.0, 0.8]
  フォロワー4（ドローン2）: 目標位置=[0.0, 0.8]
目標位置設定完了
```

### 3. 推定器初期化（修正後）

#### MOCAP接続時

```python
if MOCAP_CONNECTED:
    # MOCAP接続時：MOCAP位置で初期化
    mocap_positions_init = {}
    for follower_id in [3, 4]:
        mocap_pos = ms.get_rigid_body_position(rigid_id)
        if mocap_pos:
            x = mocap_pos.get('x', 0.0)
            z = mocap_pos.get('z', 0.0)
            mocap_positions_init[follower_id] = np.array([x, z])
            print(f"  フォロワー{follower_id}: p_initial=[{x:.3f}, {z:.3f}]（MOCAP）")
    slaf_manager.initialize_followers(mocap_positions_init)
```

#### MOCAP未接続時

```python
else:
    # MOCAP未接続時：目標位置で初期化（推定誤差ゼロ）
    print("  MOCAP未接続 - 目標位置で推定器を初期化します")
    initial_positions_from_targets = {}
    for follower_id in [3, 4]:
        initial_positions_from_targets[follower_id] = follower_target_positions[follower_id].copy()
        print(f"  フォロワー{follower_id}: p_initial=[{initial_positions_from_targets[follower_id][0]:.3f}, {initial_positions_from_targets[follower_id][1]:.3f}]（目標位置）")
    slaf_manager.initialize_followers(initial_positions_from_targets)
    print("  推定器を目標位置で初期化しました（推定誤差ゼロ）")
```

### 4. フォロワー目標設定（修正後）

```python
# フォロワー目標設定（各フォロワー独立、オフセットなし）
follower_targets = {}

for follower_id in [3, 4]:
    # 各フォロワーの目標位置を使用（初期位置からHモードで更新）
    target_pos = follower_target_positions[follower_id]
    
    # Hモード時は目標速度・加速度を設定
    if h_mode_active:
        follower_targets[follower_id] = {
            'position': target_pos,
            'velocity': target_velocity_2d,
            'acceleration': target_acceleration_2d
        }
    else:
        follower_targets[follower_id] = {
            'position': target_pos,
            'velocity': np.zeros(2),
            'acceleration': np.zeros(2)
        }
```

**特徴**:
- `formation_offsets`を使用しない
- 各フォロワーの目標位置は`follower_target_positions`から取得
- リーダーの目標位置とは独立

### 5. Hモード更新（修正後）

```python
# Hモード：目標軌道更新（src/CBF_for2TELLOs準拠）
global last_h_mode_update_time, follower_target_positions
if h_mode_active and last_h_mode_update_time is not None:
    dt = current_time - last_h_mode_update_time
    # 目標速度を更新：v_star(t+dt) = v_star(t) + a_star * dt
    old_velocity = target_velocity_2d.copy()
    target_velocity_2d += target_acceleration_2d * dt
    # 目標位置を更新：p_star(t+dt) = p_star(t) + v_star * dt
    dx = target_velocity_2d[0] * dt
    dz = target_velocity_2d[1] * dt
    
    # 各フォロワーの目標位置を更新（同じ速度で移動）
    for follower_id in [3, 4]:
        follower_target_positions[follower_id][0] += dx
        follower_target_positions[follower_id][1] += dz
    
    # デバッグ出力
    print(f"[DEBUG] Hモード更新: dt={dt:.3f}, v_new=[{target_velocity_2d[0]:.4f}, {target_velocity_2d[1]:.4f}], dx={dx:.4f}, dz={dz:.4f}")
    print(f"[DEBUG]   フォロワー3目標: [{follower_target_positions[3][0]:.4f}, {follower_target_positions[3][1]:.4f}]")
    print(f"[DEBUG]   フォロワー4目標: [{follower_target_positions[4][0]:.4f}, {follower_target_positions[4][1]:.4f}]")
```

**動作**:
1. 目標速度を加速度から積分: `v_star += a_star * dt`
2. 移動量を計算: `dx = v_star[0] * dt`, `dz = v_star[1] * dt`
3. **各フォロワーの目標位置を独立に更新**: `p_star[follower_id] += [dx, dz]`

**結果**:
- フォロワー3の目標位置: `[1.0, 0.8]` → `[1.0, 0.8-Δz]` → `[1.0, 0.8-2Δz]` ...
- フォロワー4の目標位置: `[0.0, 0.8]` → `[0.0, 0.8-Δz]` → `[0.0, 0.8-2Δz]` ...
- **両方とも同じ速度で移動するが、初期位置の差は保たれる**

---

## 動作フロー

### Tキー押下時

```
1. フォロワー目標位置を初期値に設定
   - フォロワー3: [1.0, 0.8]
   - フォロワー4: [0.0, 0.8]

2. 推定器を初期化（推定誤差ゼロ）
   - MOCAP接続: MOCAP位置で初期化
   - MOCAP未接続: 目標位置で初期化

3. SLAF制御モード開始
```

### Hキー押下時

```
1. h_mode_active = True
2. target_velocity_2d = [0.0, 0.0]（初期速度ゼロ）
3. last_h_mode_update_time = 現在時刻
```

### Hモード更新（制御ループ内、0.1秒ごと）

```
1. dt計算
2. 目標速度更新: v_star += [0.0, -0.08] * 0.1 = [0.0, -0.008]
3. 移動量計算: dx = 0.0 * 0.1 = 0.0, dz = -0.008 * 0.1 = -0.0008
4. 各フォロワーの目標位置更新:
   - フォロワー3: [1.0, 0.8] → [1.0, 0.7992]
   - フォロワー4: [0.0, 0.8] → [0.0, 0.7992]
5. 次回のdt計算用に時刻を記録
```

### CSVログ期待値

```csv
timestamp,drone_id,target_x,target_y,target_z
...,0,...,1.0,0.0,0.8      # Tキー押下直後（フォロワー3 = ドローン0）
...,1,...,0.0,0.0,0.8      # Tキー押下直後（フォロワー4 = ドローン1）
...,0,...,1.0,0.0,0.7992   # Hモード開始後0.1秒
...,1,...,0.0,0.0,0.7992
...,0,...,1.0,0.0,0.7968   # Hモード開始後0.3秒
...,1,...,0.0,0.0,0.7968
```

**特徴**:
- `target_x`は変化しない（x方向の加速度ゼロ）
- `target_z`が減少（z方向の加速度-0.08 m/s^2）
- フォロワー3とフォロワー4の`target_z`は同じ（同じ速度で移動）
- フォロワー3とフォロワー4の`target_x`は異なる（初期位置の差）

---

## 仮想リーダーの役割

仮想リーダーは**引き続き固定位置で観測に使用**されます：

```python
VIRTUAL_LEADER_POSITIONS = [
    [0.0, 1, -0.5],  # リーダー1（固定）
    [0.0, 1, 0.5]    # リーダー2（固定）
]
```

**役割**:
- 相対測定の基準点
- SLAF制御における観測ベクトル（ξ、ψ、τ）の計算に使用
- フォロワーの目標位置計算には使用しない（オフセット機能削除）

---

## 修正ファイル

1. **`mocap_slaf_main.py`**:
   - `FOLLOWER_INITIAL_TARGETS`を追加
   - `follower_target_positions`を追加
   - Tキー押下時の処理を修正（目標位置を初期値に設定）
   - 推定器初期化処理を修正（MOCAP未接続時も目標位置で初期化）
   - フォロワー目標設定からオフセット機能を削除
   - Hモード更新時に各フォロワーの目標位置を独立更新

---

## チェックリスト

✅ 仮想リーダーからのオフセット機能を削除  
✅ 各フォロワーの初期目標位置を独立設定（Tキー押下時）  
✅ ドローン1（フォロワー3）: [1.0, 0.8]  
✅ ドローン2（フォロワー4）: [0.0, 0.8]  
✅ 推定器初期化（推定誤差ゼロ、論文準拠）  
✅ MOCAP未接続時も目標位置で初期化  
✅ Hモード更新時に各フォロワーの目標位置を独立更新  
✅ 仮想リーダーは引き続き固定（観測に使用）  
⏳ 実機テスト（ユーザー確認待ち）

---

**作成日**: 2025-11-27  
**ステータス**: 修正完了、テスト準備完了
