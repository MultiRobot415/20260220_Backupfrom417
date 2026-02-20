# Hモード軌道生成実装

**日時**: 2025-11-27  
**実装内容**: Hキー一度押下で目標軌道生成開始、Jキーで停止

---

## 設計要件

### 1. 基本動作

- **Tキー**: SLAF制御モード開始、初期位置へ移動（従来通り、src/CBF_for2TELLOsと同じ）
- **Hキー**: **一度押下**で目標軌道生成開始（長押し不要、src/CBF_for2TELLOsのHキー長押し機能と同等の効果）
- **Jキー**: Hモード停止
- **G/B/V/Nキー**: 使用しない（削除）

### 2. 軌道生成方法（src/CBF_for2TELLOs準拠）

```
目標加速度（予め指定）
    ↓ 積分
目標速度（算出）
    ↓ 積分
目標位置（算出）
```

**実装**:
```python
# 毎ステップ更新
dt = current_time - last_h_mode_update_time

# 目標速度を更新：v_star(t+dt) = v_star(t) + a_star * dt
target_velocity_2d += target_acceleration_2d * dt

# 目標位置を更新：p_star(t+dt) = p_star(t) + v_star * dt
dx = target_velocity_2d[0] * dt
dz = target_velocity_2d[1] * dt
virtual_leaders.update_reference_leader_target(dx=dx, dz=dz)
```

### 3. 終了条件

- 終了時間は設計しない
- **Jキー押下で解除**

---

## 実装詳細

### 1. グローバル変数の追加

```python
# Hモード軌道生成（src/CBF_for2TELLOs準拠）
h_mode_active = False  # Hモードアクティブフラグ
target_acceleration_2d = np.array([0.0, -0.08])  # 目標加速度 [X, Z] m/s^2
target_velocity_2d = np.array([0.0, 0.0])  # 目標速度 [X, Z] m/s
last_h_mode_update_time = None  # 前回のHモード更新時刻
```

**説明**:
- `h_mode_active`: Hモードがアクティブかどうか
- `target_acceleration_2d`: 目標加速度（固定値：[0.0, -0.08] m/s^2）
- `target_velocity_2d`: 目標速度（加速度から積分で算出）
- `last_h_mode_update_time`: 前回の更新時刻（dt計算用）

### 2. Hキーの処理（一度押下で開始）

```python
# H: Hモード開始（一度押すと目標軌道生成開始）
if "h" in pressed_keys:
    if control_mode == "slaf" and virtual_leaders:
        if not h_mode_active:
            print("=" * 60)
            print("Hキー検出 - Hモード開始")
            print(f"目標加速度: [{target_acceleration_2d[0]:.3f}, {target_acceleration_2d[1]:.3f}] m/s^2")
            h_mode_active = True
            target_velocity_2d = np.array([0.0, 0.0])  # 初期速度ゼロ
            last_h_mode_update_time = current_time
            print("=" * 60)
```

**動作**:
- Hキーを一度押すと `h_mode_active = True` になる
- **長押し不要**（一度押せば継続）
- 初期速度はゼロ
- `last_h_mode_update_time` を記録して次回のdt計算に使用

### 3. Jキーの処理（停止）

```python
# J: Hモード停止
if "j" in pressed_keys:
    if h_mode_active:
        print("=" * 60)
        print("Jキー検出 - Hモード停止")
        h_mode_active = False
        target_velocity_2d = np.array([0.0, 0.0])
        last_h_mode_update_time = None
        print("=" * 60)
```

**動作**:
- Jキーを押すと `h_mode_active = False` になる
- 目標速度をゼロにリセット
- `last_h_mode_update_time` をクリア

### 4. 制御ループ内の軌道更新（src/CBF_for2TELLOs準拠）

```python
# Hモード：目標軌道更新（src/CBF_for2TELLOs準拠）
global last_h_mode_update_time
if h_mode_active and last_h_mode_update_time is not None:
    dt = current_time - last_h_mode_update_time
    # 目標速度を更新：v_star(t+dt) = v_star(t) + a_star * dt
    target_velocity_2d += target_acceleration_2d * dt
    # 目標位置を更新：p_star(t+dt) = p_star(t) + v_star * dt
    dx = target_velocity_2d[0] * dt
    dz = target_velocity_2d[1] * dt
    virtual_leaders.update_reference_leader_target(dx=dx, dz=dz)
if h_mode_active:
    last_h_mode_update_time = current_time
```

**動作**:
1. **目標速度の更新**: `v_star += a_star * dt`
2. **目標位置の更新**: `p_star += v_star * dt`
3. **仮想リーダーの目標位置を更新**: `update_reference_leader_target(dx, dz)`
4. **時刻を記録**: 次回のdt計算用

### 5. フォロワーへの目標軌道設定

```python
# Hモード時は目標速度・加速度を設定
if h_mode_active:
    follower_targets[follower_id] = {
        'position': target_pos,
        'velocity': target_velocity_2d,      # Hモードから取得
        'acceleration': target_acceleration_2d  # Hモードから取得
    }
else:
    follower_targets[follower_id] = {
        'position': target_pos,
        'velocity': np.zeros(2),
        'acceleration': np.zeros(2)
    }
```

**動作**:
- Hモード時は `target_velocity_2d` と `target_acceleration_2d` をフォロワーに設定
- 非Hモード時は速度・加速度ともにゼロ

---

## src/CBF_for2TELLOsとの対応

### src/CBF_for2TELLOs の実装

```python
# Hキー長押しで target_moving = True
if target_moving:
    if last_target_update_time is not None:
        dt = current_time - last_target_update_time
        # 全てのドローンのターゲット位置を更新
        for i in range(len(target_positions)):
            target_positions[i][0] += target_move_velocity[0] * dt  # X座標
            target_positions[i][1] += target_move_velocity[1] * dt  # Y座標
            target_positions[i][2] += target_move_velocity[2] * dt  # Z座標
        # コントローラーの目標位置も更新
        for i, ctrl in enumerate(controllers):
            ctrl.set_target_position(*target_positions[i])
    last_target_update_time = current_time
```

**特徴**:
- 毎ステップ `p_star += v_star * dt` で更新
- `target_move_velocity` は固定値

### MOCAP_forSLAF の実装

```python
# Hキー一度押しで h_mode_active = True
if h_mode_active and last_h_mode_update_time is not None:
    dt = current_time - last_h_mode_update_time
    # 目標速度を更新：v_star(t+dt) = v_star(t) + a_star * dt
    target_velocity_2d += target_acceleration_2d * dt
    # 目標位置を更新：p_star(t+dt) = p_star(t) + v_star * dt
    dx = target_velocity_2d[0] * dt
    dz = target_velocity_2d[1] * dt
    virtual_leaders.update_reference_leader_target(dx=dx, dz=dz)
if h_mode_active:
    last_h_mode_update_time = current_time
```

**差別化**:
- **一度押し vs 長押し**: Hキーを一度押すだけで継続的に軌道生成（長押し不要）
- **加速度→速度→位置**: `v_star` は加速度から積分で算出（src/CBF_for2TELLOsは速度固定）
- **同じ設計**: 毎ステップ更新、時刻記録、dt計算は同じ

---

## 動作確認方法

### テストシナリオ

```bash
python mocap_slaf_main.py
# Q（離陸） → T（SLAF開始） → H（Hモード開始） → 3秒待機 → J（停止） → E（着陸） → SPACE
```

### 期待される結果

1. **Hキー押下直後**:
   ```
   ============================================================
   Hキー検出 - Hモード開始
   目標加速度: [0.000, -0.080] m/s^2
   ============================================================
   ```

2. **Hモード中（2秒ごとのステータス表示）**:
   ```
   [SLAF制御] 飛行: True | Hモード: ON | 時刻: 10.50
     目標速度: [0.000, -0.160] m/s
   
   [SLAF制御] 飛行: True | Hモード: ON | 時刻: 12.50
     目標速度: [0.000, -0.320] m/s
   ```
   - `target_velocity_2d[1]` が時間とともに増加（負の方向）
   - 加速度 -0.08 m/s^2 で積分されている

3. **Jキー押下**:
   ```
   ============================================================
   Jキー検出 - Hモード停止
   ============================================================
   ```

4. **CSVログ確認**:
   - `target_x`, `target_z` が二次関数的に変化（加速度一定→速度線形→位置二次）
   - `v_star` がゼロ以外（ログには記録されない場合あり）

---

## 実装状況

✅ Hキー一度押下でHモード開始  
✅ Jキーで停止  
✅ 目標加速度 → 目標速度 → 目標位置 の順に計算  
✅ 毎ステップ更新（src/CBF_for2TELLOs準拠）  
✅ 長押し不要  
✅ G/B/V/Nキー削除  
✅ 終了時間なし（Jキーで手動停止）  

---

## 数式

### 目標速度の更新

$$
\mathbf{v}^*(t + \Delta t) = \mathbf{v}^*(t) + \mathbf{a}^* \Delta t
$$

ここで:
- $\mathbf{v}^*(t)$: 時刻 $t$ の目標速度
- $\mathbf{a}^*$: 目標加速度（固定：[0.0, -0.08] m/s^2）
- $\Delta t$: 制御周期（0.1秒）

### 目標位置の更新

$$
\mathbf{p}^*(t + \Delta t) = \mathbf{p}^*(t) + \mathbf{v}^*(t) \Delta t
$$

ここで:
- $\mathbf{p}^*(t)$: 時刻 $t$ の目標位置
- $\mathbf{v}^*(t)$: 時刻 $t$ の目標速度（上式で算出）

### 初期条件

$$
\begin{align}
\mathbf{v}^*(0) &= \mathbf{0} \\
\mathbf{a}^* &= [0.0, -0.08]^\top \text{ m/s}^2
\end{align}
$$

---

## 操作方法

```
【操作方法】
  Q: 離陸
  E: 着陸
  T: SLAF制御モード開始（観測・推定・制御を常に実行）
  H: Hモード開始（一度押すと目標軌道生成開始、加速度→速度→位置）
  J: Hモード停止
  O: オクルージョンON（フォロワー4 = ドローン2、ξ→0, ψ・τ有効）
  P: オクルージョンOFF（フォロワー4センサ復旧）
  M: 手動モード
  ESC: 緊急停止
  SPACE: 正常終了
```

---

**作成日**: 2025-11-27  
**ステータス**: 実装完了、テスト準備完了
