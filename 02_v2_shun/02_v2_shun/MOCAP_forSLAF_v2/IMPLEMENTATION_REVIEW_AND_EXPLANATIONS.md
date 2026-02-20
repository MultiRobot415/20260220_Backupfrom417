# システム実装の検証と説明

**日時**: 2025-11-28  
**内容**: Leaderの実装確認、ψの入力変換、ξの分解、CSVログ更新

---

## 1. Leaderの実装と観測プロセスの確認

### **現在の設計：仮想リーダー（Virtual Leaders）**

#### **実装の概要**

現在のシステムでは、**2機の仮想リーダー**がソフトウェアエージェントとして実装されています。

**クラス構成**:
- `VirtualLeader`: 個々の仮想リーダー
- `VirtualLeaderManager`: 複数の仮想リーダーを管理

#### **仮想リーダーの特性**

```python
# virtual_leader.py
class VirtualLeader:
    def __init__(self, leader_id, initial_position, dt=0.1):
        self.position = np.array(initial_position)  # 3D位置 [x, y, z]
        self.velocity = np.zeros(3)                  # 速度
        self.acceleration = np.zeros(3)              # 加速度
        self.target_position = np.array(initial_position)
    
    def update(self):
        # 位置を目標位置に設定（理想的な追従）
        self.position = self.target_position.copy()
        
        # 速度・加速度を数値微分で推定
        self.velocity = (self.position - prev_position) / self.dt
        self.acceleration = (self.velocity - prev_velocity) / self.dt
```

**重要な点**:
1. **理想的な追従**: 位置は常に目標位置と一致 (`position = target_position`)
2. **観測プロセスなし**: MOCAPやカメラでの観測は行われていない
3. **既知の状態**: フォロワーは仮想リーダーの位置を完全に知っている

#### **論文との対応**

**論文の前提（Assumption 1）**:
> "Each leader i∈Vl can access its position and is located at its target position."

つまり、**リーダーは目標位置に正確に位置している**という理想的な前提です。

**現在の実装**:
- ✅ リーダーは目標位置に正確に位置（`position = target_position`）
- ✅ フォロワーはリーダーの位置を既知として扱う
- ✅ 論文の前提と一致

---

### **観測プロセスの有無**

#### **リーダーの観測: なし**

```python
# mocap_slaf_main.py - control_drones_thread()
# 仮想リーダー更新（固定位置だが、目標位置は更新可能）
virtual_leaders.update_all()
leader_states_3d = virtual_leaders.get_all_states()

# 2D状態に変換（x, z平面）
leader_states_2d = []
for state_3d in leader_states_3d:
    leader_states_2d.append({
        'position': np.array([state_3d['position'][0], state_3d['position'][2]]),
        'target_position': np.array([state_3d['target_position'][0], state_3d['target_position'][2]])
    })
```

**観測の流れ**:
1. 仮想リーダーは内部状態を更新（理想的な追従）
2. 状態を取得（観測ではなく、直接的な状態アクセス）
3. フォロワーの制御に使用

**結論**: 
- **仮想リーダーには観測プロセスがありません**
- リーダーの位置は**完全に既知**（理想的な状態）
- これは論文の前提と一致

---

#### **フォロワーの観測: あり**

```python
# mocap_slaf_main.py - control_drones_thread()
if MOCAP_CONNECTED:
    for follower_id in [3, 4]:
        # MOCAPから位置を取得
        mocap_pos = ms.get_rigid_body_position(rigid_id)
        if mocap_pos:
            x = mocap_pos.get('x', 0.0)
            z = mocap_pos.get('z', 0.0)
            mocap_positions[follower_id] = np.array([x, z])
```

**観測の流れ**:
1. MOCAPシステムから実機ドローンの位置を観測
2. 観測値を`slaf_manager.update_followers()`に渡す
3. 推定器が観測値と重み行列から状態を推定

**結論**:
- **フォロワーには観測プロセスがあります**
- MOCAP測定値が観測に相当
- 推定器が観測値から状態を推定

---

### **現在の設計まとめ**

```
システム構成（4エージェント）:

[仮想リーダー1]  [仮想リーダー2]
    |                 |
    | 既知の状態      | 既知の状態
    | (観測なし)      | (観測なし)
    v                 v
[フォロワー3] <---> [フォロワー4]
    ^                 ^
    |                 |
    | MOCAP観測       | MOCAP観測
    |                 |
  実機ドローン1     実機ドローン2
```

**エージェント間のエッジ**:
- **Directed graph G**:
  - フォロワー3 → リーダー1, 2
  - フォロワー4 → リーダー1, 2
- **拡張エッジ（Induced graph）**:
  - フォロワー間の直接エッジ: なし（現在の実装）

**設計の妥当性**:
- ✅ 論文の前提（Assumption 1）と一致
- ✅ リーダーは理想的な追従（観測不要）
- ✅ フォロワーは観測→推定→制御のループ
- ✅ 4エージェントシステムとして正しく実装

---

## 2. ψ（共線回避項）の入力変換

### **ψの計算式（論文Eq. (19)）**

```
ψ_i = -τ_i(sign(p̂_i - p*_i) - λ_i)
```

ここで:
- `τ_i = ||g_ij - g*_ij||^2 + ||g_ik - g*_ik||^2`：bearing誤差
- `sign(·)`：符号関数（成分ごと）
- `λ_i`：調整ベクトル（0 < ||λ_i|| < 1）

### **現在の実装**

```python
# slaf_pid_controller.py - calculate_psi_and_tau()
def calculate_psi_and_tau(self, neighbor_positions_hat, neighbor_positions_star):
    # bearing誤差τの計算
    tau = np.linalg.norm(bearing_error_ij)**2 + np.linalg.norm(bearing_error_ik)**2
    
    # ψの計算
    if tau < self.tau_threshold:
        # 非共線（局所化可能）の場合、ψ = 0
        psi = np.zeros(2)
    else:
        # 共線の場合、ψを計算
        # sign関数の代わりにtanhを使用（実装上の安定性のため）
        tracking_error = self.p_hat - self.p_star
        sign_approx = np.tanh(tracking_error / 0.01)  # tanhで近似
        psi = -self.tau_gain * tau * (sign_approx - self.lambda_vec)
    
    return psi, tau
```

**パラメータ**:
- `self.tau_threshold = 0.1`: 共線判定閾値
- `self.tau_gain = 1.0`: τのゲイン
- `self.lambda_vec = [0.1, 0.1]`: 調整ベクトル

### **ψの制御入力への反映**

#### **推定器への入力（論文Eq. (19)）**

```
̇p̂_i = -2w(p̂_i - p*_i) + ̇p*_i + ξ_ijk + Σ_s ξ_sig - 2τ_i(sign(p̂_i - p*_i) - λ_i)
                                                    ^^^^^^^^^^^^^^^^^^^^^^^^^^^
                                                           = -2ψ_i
```

**実装**:
```python
# slaf_pid_controller.py - update_estimator()
def update_estimator(self, xi):
    # 推定器の更新（Eq. 39, 40, 5）
    # ̇p̂ = -2k_p(p̂ - p*) - 2k_v(v̂ - v*) + v* + ξ_gain*ξ + ψ項
    
    # 追跡誤差
    e_p_bar = self.p_hat - self.p_star
    e_v_bar = self.v_hat - self.v_star
    
    # 推定器のダイナミクス
    p_hat_dot = (
        -2 * self.k_p * e_p_bar
        - 2 * self.k_v * e_v_bar
        + self.v_star
        + self.xi_gain * xi
        # ψ項は含まれていない（現在の実装ではオクルージョン時のみ計算）
    )
```

**注意**: 現在の実装では、**ψ項は推定器に直接追加されていません**。これは、共線回避がオクルージョン時のみ作動する設計のためです。

#### **制御器への入力（論文Eq. (19)）**

```
u_i = -w(p̂_i - p*_i) + ̇p*_i - τ_i(sign(p̂_i - p*_i) - λ_i)
                                ^^^^^^^^^^^^^^^^^^^^^^^^^^^
                                       = -ψ_i
```

**実装**:
```python
# slaf_pid_controller.py - calculate_control_input()
def calculate_control_input(self, psi=None):
    # 制御入力計算
    # u = -k_cp*(p̂ - p*) - k_cv*(v - v*) + a* + ψ
    
    if psi is None:
        psi = np.zeros(2)
    
    # 追跡誤差（推定位置ベース）
    e_p_bar = self.p_hat - self.p_star
    e_v_bar = self.v_actual - self.v_star
    
    # PD制御 + フィードフォワード + 共線回避項
    u = (
        -self.k_cp * e_p_bar      # 位置誤差フィードバック
        - self.k_cv * e_v_bar      # 速度誤差フィードバック
        + self.a_star              # フィードフォワード加速度
        + psi                      # 共線回避項
    )
```

**ψの反映**:
1. `update()`メソッドでψを計算
2. `calculate_control_input(psi)`に渡す
3. 制御入力に**直接加算**

---

### **ψの物理的意味と効果**

#### **物理的意味**

- **τ_i**: bearing誤差（推定位置と目標位置のbearingのずれ）
- **sign(p̂_i - p*_i) - λ_i**: 追跡誤差の方向を調整
- **ψ_i = -τ_i × (方向調整)**: bearing誤差を補償する力

#### **制御への効果**

**通常時（非共線、τ < 0.1）**:
```
ψ = 0
u = -k_cp*(p̂ - p*) - k_cv*(v - v*) + a*
```
→ 通常のPD制御のみ

**共線時（τ ≥ 0.1）**:
```
ψ ≠ 0（bearing誤差に比例）
u = -k_cp*(p̂ - p*) - k_cv*(v - v*) + a* + ψ
```
→ 共線回避のための追加入力

**具体例**:
```python
# 共線状態で bearing誤差 τ = 0.5 の場合
tracking_error = [0.1, 0.2]  # p̂ - p*
sign_approx = tanh([0.1, 0.2] / 0.01) ≈ [1.0, 1.0]
lambda_vec = [0.1, 0.1]

psi = -1.0 * 0.5 * ([1.0, 1.0] - [0.1, 0.1])
    = -0.5 * [0.9, 0.9]
    = [-0.45, -0.45]  # m/s^2
```

この**追加加速度ψ**が制御入力に加わり、共線状態からの脱出を助けます。

---

### **ψの入力変換まとめ**

| 段階 | 処理 | 数式 |
|------|------|------|
| **1. τ計算** | bearing誤差 | `τ = ||g_ij - g*_ij||^2 + ||g_ik - g*_ik||^2` |
| **2. ψ計算** | 共線回避項 | `ψ = -τ(sign(p̂ - p*) - λ)` |
| **3. 制御入力** | ψを加算 | `u = -k_cp*(p̂ - p*) - k_cv*(v - v*) + a* + ψ` |
| **4. RC変換** | 加速度→RC | `rc_lr = clip(-u[0] * 40, -40, 40)` |

**ψの効果**:
- 共線状態の検出（τ > threshold）
- bearing誤差に比例した補償力
- 制御入力に直接加算（m/s^2単位）
- RCコマンドに変換されてドローンへ送信

---

## 3. ξの分解（ξ_ijk と ξ_sig）

### **論文Eq. (20)の定義**

```
ξ_ijk = H^T_ii * Hij * (p̂_j - p̂_i) + H^T_ii * Hik * (p̂_k - p̂_i)
ξ_sig = H^T_si * Hss * (p̂_s - p̂_i) - H^T_si * Hsg * (p̂_g - p̂_i)
```

ここで:
- **ξ_ijk**: 直接の隣接エージェント（j, k）からの補正項（directed graph G）
- **ξ_sig**: 拡張エッジ（フォロワー間）からの補正項（induced graph）

### **現在のシステムでの実装**

#### **4エージェントシステムの構成**

```
エージェント構成:
- リーダー1 (ID=1)
- リーダー2 (ID=2)
- フォロワー3 (ID=3): 隣接 = [1, 2]
- フォロワー4 (ID=4): 隣接 = [1, 2]

Directed graph G:
  1, 2 → 3
  1, 2 → 4

拡張エッジ（フォロワー間）:
  なし（現在の実装）
```

#### **ξの分解**

```python
# slaf_pid_controller.py - calculate_xi()
# 論文Eq. (20): ξ = ξ_ijk + Σ_s ξ_sig
# 現在の4エージェントシステム（リーダー2機、フォロワー2機）では：
# - ξ_ijk: 直接の隣接エージェント（j, k）からの情報 (リーダー1, 2)
# - ξ_sig: 拡張エッジ（フォロワー間）からの情報 = 0（エッジなし）

xi_ijk = xi_total.copy()  # 現在の実装ではξ_ijkのみ
xi_sig = np.zeros(2)       # 拡張エッジなし

# debug_infoに保存
self.debug_info['xi'] = xi_total
self.debug_info['xi_ijk'] = xi_ijk
self.debug_info['xi_sig'] = xi_sig
```

**説明**:
- **ξ_ijk**: リーダー1, 2からの相対測定に基づく補正（H_i1, H_i2を使用）
- **ξ_sig**: フォロワー間の相対測定（現在は0、将来的に追加可能）

---

### **CSVログでの記録**

#### **新しいCSVフォーマット**

```csv
timestamp,drone_id,...,xi_x,xi_y,xi_z,xi_ijk_x,xi_ijk_y,xi_ijk_z,xi_sig_x,xi_sig_y,xi_sig_z,...
```

**各カラムの意味**:
- `xi_x, xi_y, xi_z`: ξの合計（= ξ_ijk + ξ_sig）
- `xi_ijk_x, xi_ijk_y, xi_ijk_z`: 直接隣接からの補正項
- `xi_sig_x, xi_sig_y, xi_sig_z`: 拡張エッジからの補正項（現在は0）

**現在の実装での値**:
```
xi = xi_ijk        (ξ_sig = 0 なので)
xi_ijk = [..., ...]  (リーダー1, 2からの情報)
xi_sig = [0, 0]      (拡張エッジなし)
```

---

## 4. CSVログの更新まとめ

### **追加項目**

1. **v*star（目標速度）**: `target_vx, target_vy, target_vz`
2. **ξの分解**:
   - `xi_ijk_x, xi_ijk_y, xi_ijk_z`: 直接隣接からの補正項
   - `xi_sig_x, xi_sig_y, xi_sig_z`: 拡張エッジからの補正項

### **削除項目**

- `k_p, k_v, k_cv`: ゲイン値（処理負荷軽減のため）

### **新しいCSVヘッダー（抜粋）**

```csv
...,target_x,target_y,target_z,target_vx,target_vy,target_vz,...,
xi_x,xi_y,xi_z,xi_ijk_x,xi_ijk_y,xi_ijk_z,xi_sig_x,xi_sig_y,xi_sig_z,...
```

---

## まとめ

### **Leaderの実装**

✅ **仮想リーダーとして正しく実装**
- 理想的な追従（位置 = 目標位置）
- 観測プロセスなし（既知の状態）
- 論文の前提（Assumption 1）と一致

### **ψの入力変換**

✅ **制御入力に直接反映**
- τ（bearing誤差）を計算
- ψ = -τ × (方向調整) を計算
- 制御入力 u に加算
- RCコマンドに変換してドローンへ送信

### **ξの分解**

✅ **ξ_ijk と ξ_sig に分解**
- ξ_ijk: リーダーからの補正項（現在の主要項）
- ξ_sig: フォロワー間の補正項（現在は0）
- CSVログに両方を記録

### **CSVログ更新**

✅ **処理負荷軽減と情報追加**
- v*star（目標速度）追加
- ξの分解を記録
- k_p, k_v, kcv削除

---

**作成日**: 2025-11-28  
**ステータス**: 実装完了、説明完了
