# v2オブザーバ型SLAF実機実装設計書

## 文書情報
- **作成日**: 2025年12月11日
- **対象**: v2オブザーバ型二次系SLAF制御の実機実装
- **理論ベース**: `02_v2_shun/ref_v2/v4_observer.tex`
- **シミュレーション**: `02_v2_shun/ref_v2/sim_v2/`
- **参照実装**: `01_v1_PID/MOCAP_forSLAF/`（v1, PID階層型）

---

## 1. 概要

### 1.1 目的
v2オブザーバ型SLAF制御を実機ドローン（Tello EDU）に実装する。理論文書 `v4_observer.tex` で厳密に証明された制御則を、実機環境に適用する。

### 1.2 v1からの主要な変更点

| 項目 | v1 (PID階層型) | v2 (オブザーバ型) |
|------|----------------|-------------------|
| **推定器** | PID型（3状態） | Luenbergerオブザーバ型（2状態） |
| **積分状態** | あり（zᵢ） | **なし** |
| **速度フィードバック** | `v̂ᵢ - vᵢ`（差分） | `vᵢ - v̂ᵢ`（オブザーバ） |
| **理論構造** | 統合型（結合あり） | **完全分離（カスケード）** |
| **目標加速度** | 理論上要求 | **任意（完全キャンセル）** |

### 1.3 設計の基本方針

1. **理論厳密性の維持**: `v4_observer.tex` の式を厳密に実装
2. **基本設定の継承**: v1の仮想リーダー、目標値設計、MOCAP連携はそのまま使用
3. **速度測定の扱い**: 数値微分で推定（MOCAPベース）
4. **座標系**: v1と同じ（2D平面 [x, z]）

---

## 2. 理論式の厳密な定義

### 2.1 システムダイナミクス

```
ṗᵢ = vᵢ   (運動学)
v̇ᵢ = uᵢ   (二次系、制御入力)
```

### 2.2 推定器（Luenbergerオブザーバ型）

#### v2の推定器（オブザーバ型）
```python
˙p̂ᵢ = vᵢ + ξᵢ                    # 式(143) in v4_observer.tex
˙v̂ᵢ = uᵢ + K_obs(vᵢ - v̂ᵢ)        # 式(144) in v4_observer.tex
```

**重要なポイント**:
- **実速度 vᵢ を直接使用**（測定可能と仮定）
- オブザーバゲイン `K_obs` による速度推定誤差の収束
- **積分状態 zᵢ は存在しない**（I制御なし）

#### v1の推定器（参考：変更前）
```python
# v1: PID階層型（参考）
żᵢ = p̂ᵢ - pᵢ*
˙p̂ᵢ = v̂ᵢ
˙v̂ᵢ = -k_p(p̂ᵢ - pᵢ*) - k_v(v̂ᵢ - vᵢ) - k_i·zᵢ + aᵢ* + ξᵢ
```

### 2.3 制御器（PD型 + フィードフォワード）

#### v2の制御器
```python
uᵢ = p̈ᵢ* - K_p(p̂ᵢ - pᵢ*) - K_v(vᵢ - ṗᵢ*) + ψᵢ   # 式(174) in v4_observer.tex
```

**重要なポイント**:
- **目標加速度 p̈ᵢ* をフィードフォワード**（完全キャンセル）
- 推定位置 p̂ᵢ を使用（位置フィードバック）
- **実速度 vᵢ を使用**（速度フィードバック）
- 速度誤差項は `(vᵢ - ṗᵢ*) = (vᵢ - vᵢ*)`

#### v1の制御器（参考：変更前）
```python
# v1: PD型（参考）
uᵢ = -k_cp(p̂ᵢ - pᵢ*) - k_cv(vᵢ - vᵢ*) + aᵢ* + ψᵢ
```

### 2.4 状態変数

#### v2の状態（4変数）
```python
# エージェントiごとに
pᵢ       # 実位置 [x, z]
vᵢ       # 実速度 [vx, vz]（数値微分）
p̂ᵢ       # 推定位置 [x_hat, z_hat]
v̂ᵢ       # 推定速度 [vx_hat, vz_hat]

# ★積分状態 zᵢ は存在しない★
```

#### v1の状態（5変数、参考）
```python
# v1: 参考
pᵢ, vᵢ, p̂ᵢ, v̂ᵢ, zᵢ  # 積分状態 zᵢ あり
```

---

## 3. 実機実装の詳細設計

### 3.1 初期化

#### 初期推定誤差の仮定
理論（`v4_observer.tex` 仮定1）より：
```python
# 初期条件（推奨）
p̂ᵢ(0) = pᵢ(0)   # 推定位置 = 実位置（MOCAP測定値）
v̂ᵢ(0) = vᵢ(0)   # 推定速度 = 実速度（初期は零）
```

**実装**:
```python
def __init__(self):
    # 実位置・速度（MOCAP + 数値微分）
    self.p_actual = np.zeros(2)   # [x, z]
    self.v_actual = np.zeros(2)   # [vx, vz]
    
    # 推定位置・速度（オブザーバ）
    self.p_hat = np.zeros(2)      # [x_hat, z_hat]
    self.v_hat = np.zeros(2)      # [vx_hat, vz_hat]
    
    # ★積分状態 z は存在しない★
    
    # 目標値
    self.p_star = np.zeros(2)     # [x*, z*]
    self.v_star = np.zeros(2)     # [vx*, vz*]
    self.a_star = np.zeros(2)     # [ax*, az*]
```

### 3.2 推定器の更新（オブザーバ型）

#### 理論式（v4_observer.tex 式143-144）
```
˙p̂ᵢ = vᵢ + ξᵢ
˙v̂ᵢ = uᵢ + K_obs(vᵢ - v̂ᵢ)
```

#### 実装（Euler法）
```python
def update_estimator(self, xi, u):
    """
    オブザーバ型推定器の更新
    
    v4_observer.tex 式(143), 式(144) に厳密準拠
    
    Args:
        xi: 幾何学的補正項 [2,]
        u: 現在の制御入力（加速度） [2,]
    """
    # ˙p̂ᵢ = vᵢ + ξᵢ  (式143)
    p_hat_dot = self.v_actual + self.xi_gain * xi
    
    # ˙v̂ᵢ = uᵢ + K_obs(vᵢ - v̂ᵢ)  (式144)
    # ★重要: オブザーバフィードバック項 K_obs(vᵢ - v̂ᵢ)★
    v_hat_dot = u + self.K_obs * (self.v_actual - self.v_hat)
    
    # Euler法で更新
    self.p_hat += p_hat_dot * self.dt
    self.v_hat += v_hat_dot * self.dt
    
    # デバッグ情報
    self.debug_info['p_hat_dot'] = p_hat_dot
    self.debug_info['v_hat_dot'] = v_hat_dot
    self.debug_info['observer_feedback'] = self.K_obs * (self.v_actual - self.v_hat)
```

#### v1との比較（参考）
```python
# v1: PID階層型（参考、変更前）
def update_estimator(self, xi):
    # ż_i = p̂_i - p_i^*
    z_dot = self.p_hat - self.p_star
    
    # p̂̇_i = v̂_i
    p_hat_dot = self.v_hat
    
    # v̂̇_i = -k_p(p̂_i - p_i^*) - k_v(v̂_i - v_i) - k_i*z_i + a_i^* + ξ_i
    v_hat_dot = (
        - self.k_p * (self.p_hat - self.p_star)      # P項
        - self.k_v * (self.v_hat - self.v_actual)    # D項（速度フィードバック）
        - self.k_i * self.z_integral                 # I項
        + self.a_star                                # フィードフォワード
        + self.xi_gain * xi                          # 幾何学的補正項
    )
    
    # Euler法で更新
    self.z_integral += z_dot * self.dt
    self.p_hat += p_hat_dot * self.dt
    self.v_hat += v_hat_dot * self.dt
```

### 3.3 制御器の計算

#### 理論式（v4_observer.tex 式174）
```
uᵢ = p̈ᵢ* - K_p(p̂ᵢ - pᵢ*) - K_v(vᵢ - ṗᵢ*) + ψᵢ
```

#### 実装
```python
def calculate_control_input(self, psi=None):
    """
    制御入力の計算（PD型 + フィードフォワード）
    
    v4_observer.tex 式(174) に厳密準拠
    
    Args:
        psi: 共線回避項（オクルージョン時のみ） [2,]
    
    Returns:
        u: 制御入力（加速度指令） [ax, az]
    """
    if psi is None:
        psi = np.zeros(2)
    
    # 位置誤差を計算（推定位置ベース）
    position_error = self.p_hat - self.p_star
    
    # 不感帯処理（v1と同じ）
    if abs(position_error[0]) < self.deadband_x:
        position_error[0] = 0.0
    if abs(position_error[1]) < self.deadband_z:
        position_error[1] = 0.0
    
    # 制御入力の計算（式174）
    # uᵢ = p̈ᵢ* - K_p(p̂ᵢ - pᵢ*) - K_v(vᵢ - ṗᵢ*) + ψᵢ
    u = (
        self.a_star                                  # p̈ᵢ* フィードフォワード
        - self.K_p * position_error                  # -K_p(p̂ᵢ - pᵢ*) 位置制御項
        - self.K_v * (self.v_actual - self.v_star)   # -K_v(vᵢ - ṗᵢ*) 速度制御項
        + psi                                        # ψᵢ 共線回避項
    )
    
    # デバッグ情報
    self.debug_info['control_input'] = u
    self.debug_info['position_error_raw'] = self.p_hat - self.p_star
    self.debug_info['position_error_deadband'] = position_error
    self.debug_info['velocity_error'] = self.v_actual - self.v_star
    self.debug_info['feedforward'] = self.a_star
    
    return u
```

#### v1との比較（参考）
```python
# v1: PD型（参考、変更前）
def calculate_control_input(self, psi=None):
    if psi is None:
        psi = np.zeros(2)
    
    position_error = self.p_hat - self.p_star
    
    # 不感帯処理
    if abs(position_error[0]) < self.deadband_x:
        position_error[0] = 0.0
    if abs(position_error[1]) < self.deadband_z:
        position_error[1] = 0.0
    
    # uᵢ = -k_cp(p̂ᵢ - pᵢ*) - k_cv(vᵢ - vᵢ*) + aᵢ* + ψᵢ
    u = (
        - self.k_cp * position_error                 # 位置制御項
        - self.k_cv * (self.v_actual - self.v_star)  # 速度制御項
        + self.a_star                                # フィードフォワード
        + psi                                        # 共線回避項
    )
    
    return u
```

### 3.4 制御フロー（update関数）

```python
def update(self, p_actual, leader_positions, follower_positions, 
           p_star, v_star, a_star, dt, is_occluded=False):
    """
    制御ループのメイン更新処理
    
    Args:
        p_actual: 実位置（MOCAP測定） [x, z]
        leader_positions: リーダー位置のリスト
        follower_positions: フォロワー位置のリスト
        p_star: 目標位置 [x*, z*]
        v_star: 目標速度 [vx*, vz*]
        a_star: 目標加速度 [ax*, az*]
        dt: 制御周期 [s]
        is_occluded: オクルージョンフラグ
    
    Returns:
        u: 制御入力（加速度指令） [ax, az]
    """
    # 1. タイムステップ更新
    self.dt = dt
    current_time = time.time()
    
    # 2. 実位置の更新
    self.p_actual = p_actual.copy()
    
    # 3. 速度推定（数値微分）
    if self.prev_time is not None:
        dt_vel = current_time - self.prev_time
        self.v_actual = (p_actual - self.prev_position) / dt_vel
    
    # 4. 目標値の更新
    self.p_star = p_star.copy()
    self.v_star = v_star.copy()
    self.a_star = a_star.copy()
    
    # 5. 幾何学的補正項 ξᵢ の計算（重み行列ベース）
    if not is_occluded:
        xi = self.calculate_xi(leader_positions, follower_positions)
    else:
        xi = np.zeros(2)  # オクルージョン時はξ=0
    
    # 6. 推定器の更新（オブザーバ型）
    # 注意: 前回の制御入力 u を使用
    self.update_estimator(xi, self.prev_control_input)
    
    # 7. 共線回避項 ψᵢ の計算（オクルージョン時のみ）
    if is_occluded:
        psi, tau = self.calculate_psi_and_tau(leader_positions, follower_positions)
    else:
        psi = np.zeros(2)
        tau = 0.0
    
    # 8. 制御入力の計算
    u = self.calculate_control_input(psi)
    
    # 9. 前回値の保存
    self.prev_position = p_actual.copy()
    self.prev_time = current_time
    self.prev_control_input = u.copy()
    
    # 10. デバッグ情報の更新
    self.debug_info['xi'] = xi
    self.debug_info['psi'] = psi
    self.debug_info['tau'] = tau
    self.debug_info['is_occluded'] = is_occluded
    
    return u
```

---

## 4. ゲイン設定

### 4.1 理論的要求条件

`v4_observer.tex` 定理4.2より：
```
K_obs > 0     (オブザーバゲイン)
K_p > 0       (位置制御ゲイン)
K_v > 0       (速度制御ゲイン)
γ ∈ (0, 1]    (λ調整ゲイン)
λ_max ∈ (0, 1) (λ飽和限界)
```

### 4.2 推奨初期値

#### オブザーバゲイン
```python
K_obs = 10.0  # 速度推定誤差の収束速度を決定
```

**調整指針**:
- 大きい → 速い収束、振動的
- 小さい → 遅い収束、滑らか
- 推奨範囲: 5.0 ~ 20.0

#### 位置制御ゲイン
```python
K_p = 10.0    # ω_n^2 に相当（固有振動数）
```

**調整指針**:
- 2次系として設計: ω_n = √K_p
- 推奨 ω_n: 2 ~ 5 rad/s
- 推奨 K_p: 4 ~ 25

#### 速度制御ゲイン
```python
K_v = 10.0    # 2ζω_n に相当（減衰比）
```

**調整指針**:
- 減衰比: ζ = K_v / (2√K_p)
- 推奨 ζ: 0.7 ~ 1.5（臨界減衰付近）
- K_p=10 の場合: K_v = 2ζ√10 ≈ 4.4ζ → 推奨 K_v: 3 ~ 10

#### 補正項のゲイン（実装上の調整）★重要★
```python
w_xi = 5.0     # ξ項のゲイン（シミュレーションと同じ）
w_psi = 1.0    # ψ項のゲイン（シミュレーションと同じ）
```

**調整指針**:
- `w_xi`: 幾何学的補正項の強さを調整（推定誤差の収束速度に影響）
  - 大きい → 速い収束、振動的
  - 小さい → 遅い収束、滑らか
  - 推奨範囲: 1.0 ~ 10.0
  - シミュレーション（sim_v2）と実機で同じ値を使用

- `w_psi`: 共線回避項の強さを調整（オクルージョン時のみ有効）
  - 大きい → 強い回避、振動的
  - 小さい → 弱い回避、スムーズ
  - 推奨範囲: 0.5 ~ 2.0
  - シミュレーション（sim_v2）と実機で同じ値を使用

#### その他のゲイン
```python
gamma = 1.0        # λ調整ゲイン
lambda_max = 0.99  # λ飽和限界
```

### 4.3 v1との比較

| ゲイン | v1 (PID階層型) | v2 (オブザーバ型) |
|--------|----------------|-------------------|
| 推定器位置 | k_p = 5.0 | （対応なし） |
| 推定器速度 | k_v = 1.0 | K_obs = 10.0 |
| 推定器積分 | k_i = 0.1 | **なし** |
| 制御器位置 | k_cp = 5.0 | K_p = 10.0 |
| 制御器速度 | k_cv = 2.0 | K_v = 10.0 |
| ξゲイン | xi_gain = 1.0 | w_xi = 5.0 |
| ψゲイン | （なし） | w_psi = 1.0 |

### 4.4 実装
```python
def set_gains(self, K_obs=None, K_p=None, K_v=None, 
              gamma=None, lambda_max=None, w_xi=None, w_psi=None):
    """
    制御ゲインを設定する
    
    Args:
        K_obs: オブザーバゲイン (> 0)
        K_p: 位置制御ゲイン (> 0)
        K_v: 速度制御ゲイン (> 0)
        gamma: λ調整ゲイン (0 < γ ≤ 1)
        lambda_max: λ飽和限界 (0 < λ_max < 1)
        w_xi: ξ項のゲイン（実装上の調整）
        w_psi: ψ項のゲイン（実装上の調整）
    """
    if K_obs is not None:
        assert K_obs > 0, "K_obs must be positive"
        self.K_obs = K_obs
    
    if K_p is not None:
        assert K_p > 0, "K_p must be positive"
        self.K_p = K_p
    
    if K_v is not None:
        assert K_v > 0, "K_v must be positive"
        self.K_v = K_v
    
    if gamma is not None:
        assert 0 < gamma <= 1, "gamma must be in (0, 1]"
        self.gamma = gamma
    
    if lambda_max is not None:
        assert 0 < lambda_max < 1, "lambda_max must be in (0, 1)"
        self.lambda_max = lambda_max
    
    if w_xi is not None:
        self.w_xi = w_xi
    
    if w_psi is not None:
        self.w_psi = w_psi
```

---

## 5. 速度推定の実装

### 5.1 数値微分

v2では速度 vᵢ は**測定可能**と仮定しているが、実機ではMOCAPから位置のみ取得。

**実装方法**（v1と同じ）:
```python
# 位置の時系列微分
if self.prev_time is not None:
    dt = current_time - self.prev_time
    self.v_actual = (p_actual - self.prev_position) / dt
```

### 5.2 速度推定オブザーバの役割

v2の推定器は以下を推定：
```
v̂ᵢ: 速度推定値
```

しかし、実機では `v_actual` を数値微分で取得するため、オブザーバの `v̂ᵢ` は**フィルタリングされた速度推定値**として機能。

**重要**: 
- 制御入力計算では `v_actual`（数値微分）を使用
- オブザーバは `v̂ᵢ` を推定するが、これは主に理論的整合性のため
- `v̂ᵢ` と `v_actual` の差がオブザーバフィードバック項を駆動

---

## 6. 重み行列と補正項の計算

### 6.1 幾何学的補正項 ξᵢ

**v1と同じ実装を使用**。

```python
def calculate_xi(self, leader_positions, follower_positions):
    """
    幾何学的補正項 ξᵢ の計算
    
    v1と同じ実装（weight_matrices.py の calculate_xi_correction）
    
    Args:
        leader_positions: リーダー位置のリスト
        follower_positions: フォロワー位置のリスト
    
    Returns:
        xi: 幾何学的補正項 [2,]
    """
    # weight_matrices.py の関数を呼び出し
    from weight_matrices import calculate_xi_correction
    
    xi = calculate_xi_correction(
        self.follower_id,
        self.p_hat,  # ★推定位置ベース★
        leader_positions,
        follower_positions,
        self.neighbor_map
    )
    
    return xi
```

### 6.2 共線回避項 ψᵢ

**v1と同じ実装を使用**。

```python
def calculate_psi_and_tau(self, leader_positions, follower_positions):
    """
    共線回避項 ψᵢ と τᵢ の計算
    
    v1と同じ実装（weight_matrices.py の calculate_psi_collinearity_avoidance）
    
    Args:
        leader_positions: リーダー位置のリスト
        follower_positions: フォロワー位置のリスト
    
    Returns:
        psi: 共線回避項 [2,]
        tau: 共線状態検出項（スカラー）
    """
    # weight_matrices.py の関数を呼び出し
    from weight_matrices import calculate_psi_collinearity_avoidance
    
    psi, tau = calculate_psi_collinearity_avoidance(
        self.follower_id,
        self.p_hat,              # ★推定位置ベース★
        self.v_actual,           # 実速度
        self.v_hat,              # 推定速度
        leader_positions,
        follower_positions,
        self.neighbor_map,
        self.gamma,
        self.lambda_max
    )
    
    return psi, tau
```

---

## 7. v1からの移行手順

### 7.1 ファイル構成（変更なし）

```
MOCAP_forSLAF_v2/
├── slaf_observer_controller.py  # ← v1の slaf_pid_controller.py を変更
├── weight_matrices.py            # ← v1と同じ（変更なし）
├── virtual_leader.py             # ← v1と同じ（変更なし）
├── mocap_slaf_main.py            # ← v1と同じ（インポート名のみ変更）
├── mocap_stream.py               # ← v1と同じ（変更なし）
├── csv_logger.py                 # ← v1と同じ（変更なし）
└── README.md                     # ← v2用に更新
```

### 7.2 変更が必要なファイル

#### 7.2.1 `slaf_pid_controller.py` → `slaf_observer_controller.py`

**変更内容**:
1. クラス名変更:
   - `SLAFPIDController` → `SLAFObserverController`
   - `SLAFSystemManager` → そのまま（内部でSLAFObserverControllerを使用）

2. 状態変数の変更:
   - `self.z_integral` を削除
   - `self.k_i` を削除
   - `self.K_obs` を追加

3. `update_estimator` メソッドの変更:
   - PID型 → オブザーバ型に変更

4. `calculate_control_input` メソッドの変更:
   - フィードフォワード項 `a_star` を先頭に移動
   - 符号を明示的に記述

5. `set_gains` メソッドの変更:
   - `k_p`, `k_v`, `k_i`, `k_cp`, `k_cv` → `K_obs`, `K_p`, `K_v` に変更

#### 7.2.2 `mocap_slaf_main.py`

**変更内容**（最小限）:
```python
# 変更前（v1）
from slaf_pid_controller import SLAFPIDController, SLAFSystemManager

# 変更後（v2）
from slaf_observer_controller import SLAFObserverController, SLAFSystemManager
```

```python
# 変更前（v1）
controller1 = SLAFPIDController(follower_id=3, ...)
controller2 = SLAFPIDController(follower_id=4, ...)

# 変更後（v2）
controller1 = SLAFObserverController(follower_id=3, ...)
controller2 = SLAFObserverController(follower_id=4, ...)
```

```python
# ゲイン設定の変更
# 変更前（v1）
controller1.set_gains(
    k_p=5.0, k_v=1.0, k_i=0.1,
    k_cp=5.0, k_cv=2.0, xi_gain=1.0
)

# 変更後（v2）
controller1.set_gains(
    K_obs=10.0, K_p=10.0, K_v=10.0,
    gamma=1.0, lambda_max=0.99, w_xi=5.0, w_psi=1.0
)
```

#### 7.2.3 `README.md`

v2用に更新（制御則、ゲイン設定、理論参照）。

### 7.3 変更が不要なファイル

以下はv1のまま使用可能：
- `weight_matrices.py`: 重み行列計算（v1と同じ）
- `virtual_leader.py`: 仮想リーダー管理（v1と同じ）
- `mocap_stream.py`: MOCAP連携（v1と同じ）
- `csv_logger.py`: ログ記録（v1と同じ）
- `keyboard_control.py`: キーボード入力（v1と同じ）
- `custom_tello.py`: Tello制御（v1と同じ）

---

## 8. デバッグとチューニング

### 8.1 デバッグ情報

```python
self.debug_info = {
    'p_actual': self.p_actual,
    'v_actual': self.v_actual,
    'p_hat': self.p_hat,
    'v_hat': self.v_hat,
    'p_star': self.p_star,
    'v_star': self.v_star,
    'a_star': self.a_star,
    'xi': xi,
    'psi': psi,
    'tau': tau,
    'control_input': u,
    'observer_feedback': self.K_obs * (self.v_actual - self.v_hat),
    'estimation_error_p': np.linalg.norm(self.p_hat - self.p_actual),
    'estimation_error_v': np.linalg.norm(self.v_hat - self.v_actual),
    'tracking_error': np.linalg.norm(self.p_actual - self.p_star),
}
```

### 8.2 チューニング手順

#### ステップ1: オブザーバゲイン K_obs
1. K_obs = 5.0 から開始
2. 速度推定誤差 `‖v̂ᵢ - vᵢ‖` を確認
3. 振動が大きければ減少、収束が遅ければ増加
4. 推奨範囲: 5.0 ~ 20.0

#### ステップ2: 位置制御ゲイン K_p
1. K_p = 10.0 から開始
2. 固有振動数 ω_n = √K_p ≈ 3.16 rad/s を確認
3. 応答が遅ければ増加、振動的なら減少
4. 推奨範囲: 4.0 ~ 25.0

#### ステップ3: 速度制御ゲイン K_v
1. 減衰比 ζ = K_v / (2√K_p) を計算
2. ζ ≈ 1.0（臨界減衰）を目標
3. オーバーシュートが大きければ増加、応答が遅ければ減少
4. 推奨: K_v = 2√K_p ~ 3√K_p

#### ステップ4: ξゲイン
1. xi_gain = 5.0 から開始
2. 推定誤差 `‖p̂ᵢ - pᵢ‖` を確認
3. 収束が遅ければ増加、振動的なら減少
4. 推奨範囲: 1.0 ~ 10.0

### 8.3 期待される挙動

#### 正常動作
- 推定誤差は指数的に減少（理論保証）
- 追跡誤差は漸近的に零に収束
- 振動は臨界減衰付近（ζ ≈ 0.7 ~ 1.5）

#### 異常動作と対処
- **振動的**: K_obs, K_v を増加、K_p を減少
- **応答が遅い**: K_p を増加、K_obs を増加
- **定常偏差**: v2はI制御なし → 目標値設計を見直し

---

## 9. 理論との対応関係

### 9.1 v4_observer.tex との対応

| 実装 | 理論式 | 備考 |
|------|--------|------|
| `update_estimator` | 式(143), (144) | Luenbergerオブザーバ |
| `calculate_control_input` | 式(174) | PD + フィードフォワード |
| `K_obs` | K_obs > 0 | 定理4.2の条件 |
| `K_p` | K_p > 0 | 定理4.2の条件 |
| `K_v` | K_v > 0 | 定理4.2の条件 |
| `gamma` | γ ∈ (0, 1] | 定理4.2の条件 |
| `lambda_max` | λ_max ∈ (0, 1) | 定理4.2の条件 |

### 9.2 sim_v2/system_dynamics.m との対応

| Python実装 | MATLAB実装 | 対応箇所 |
|-----------|-----------|---------|
| `update_estimator` | `system_dynamics.m` 76-86行 | 推定器 |
| `calculate_control_input` | `system_dynamics.m` 56-64行 | 制御器 |
| `K_obs` | `config.K_obs` | オブザーバゲイン |
| `K_p` | `config.K_p` | 位置制御ゲイン |
| `K_v` | `config.K_v` | 速度制御ゲイン |

---

## 10. 注意事項とFAQ

### 10.1 速度測定の扱い

**Q**: v2は速度測定可能と仮定しているが、実機では？
**A**: MOCAPから位置のみ取得し、数値微分で速度推定。理論との整合性は保たれる。

### 10.2 積分状態の削除

**Q**: I制御がないと定常偏差が残るのでは？
**A**: v2は理論的にI制御なしで収束を証明。実機で定常偏差が問題なら、目標値設計を見直すか、v1を使用。

### 10.3 目標加速度の扱い

**Q**: 目標加速度が零でなくてもよい？
**A**: はい。v2では目標加速度は完全にキャンセルされるため、任意の値でよい。これは理論的特徴。

### 10.4 srcフォルダとの関係

**Q**: `02_v2_shun/src/` のコードは使える？
**A**: 参考程度。異なるプロジェクト（フォーメーション制御・CBF）のため、直接流用しない。基本設計（MOCAP連携、キーボード操作）は類似。

### 10.5 v1との共存

**Q**: v1とv2を同時に実行できる？
**A**: ファイル構成が異なるため、ディレクトリを分ければ可能。ゲイン設定と制御則の違いに注意。

---

## 11. まとめ

### 11.1 実装のポイント
1. **推定器**: PID型 → オブザーバ型に変更
2. **積分状態**: 削除（I制御なし）
3. **オブザーバゲイン**: K_obs を追加
4. **制御器**: フィードフォワード項を明示
5. **基本設定**: v1と同じ（仮想リーダー、重み行列、MOCAP）

### 11.2 理論との整合性
- `v4_observer.tex` の式を厳密に実装
- sim_v2の構造を実機に移植
- 分離原理によるカスケード構造を維持

### 11.3 次のステップ
1. `slaf_pid_controller.py` を `slaf_observer_controller.py` に変更
2. 推定器と制御器のメソッドを修正
3. ゲイン設定を更新
4. 実機でチューニング
5. 性能評価（推定誤差、追跡誤差、収束速度）

---

---

## 12. 実装完了内容（2025年12月11日）

### 12.1 完成したファイル

**`slaf_observer_controller.py`**: v2オブザーバ型SLAF制御器（完成）
- クラス: `SLAFObserverController`, `SLAFSystemManager`
- 推定器: Luenbergerオブザーバ型（式143-144）
- 制御器: PD + フィードフォワード（式174）
- ゲイン: K_obs, K_p, K_v, w_xi, w_psi
- 初期推定誤差: 零（検証のため）
- 速度測定: 数値微分（v1と同じ）
- オクルージョン: 手動設定（v1と同じ）

### 12.2 確認事項

✅ **目標加速度**: v1と同様に継承（一定値で検証）  
✅ **実速度**: v1と同じく数値微分で取得（使い方が異なる）  
✅ **初期推定誤差**: 位置・速度ともに0（検証のため）  
✅ **共線回避**: 手動オクルージョン（v1と同じ）  
✅ **ξ, ψゲイン**: `w_xi`, `w_psi` で調整可能（シミュレーション準拠）  

### 12.3 主な変更点（v1からv2へ）

| 項目 | 変更内容 |
|------|---------|
| **ファイル名** | `slaf_pid_controller.py` → `slaf_observer_controller.py` |
| **クラス名** | `SLAFPIDController` → `SLAFObserverController` |
| **積分状態** | `self.z_integral` を削除 |
| **推定器** | PID型 → Luenbergerオブザーバ型 |
| **制御器** | PD型 → PD + フィードフォワード明示 |
| **ゲイン** | `k_p, k_v, k_i, k_cp, k_cv` → `K_obs, K_p, K_v` |
| **補正ゲイン** | `xi_gain` → `w_xi, w_psi` |

### 12.4 次のステップ

1. **README.md の更新**: v2用に更新（制御則、ゲイン設定、理論参照）
2. **mocap_slaf_main.py の変更**: インポート名とゲイン設定を更新
3. **実機テスト**: 初期ゲイン（K_obs=10, K_p=10, K_v=10, w_xi=5.0, w_psi=1.0）で検証
4. **チューニング**: 推定誤差・追跡誤差・収束速度を確認
5. **性能評価**: v1との比較（収束速度、定常偏差、振動）

---

**作成者**: Windsurf Cascade AI  
**更新日**: 2025年12月11日  
**理論ベース**: v4_observer.tex, sim_v2/system_dynamics.m  
**参照実装**: 01_v1_PID/MOCAP_forSLAF/slaf_pid_controller.py  
**実装完了**: slaf_observer_controller.py
