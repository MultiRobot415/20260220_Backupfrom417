# 実装更新サマリー（2025/11/25）- センサオクルージョン機能

## ユーザー要求
1. **CSVログにξの値を追加**
2. **センサオクルージョンを仮想的に発生**:
   - 対象：フォロワー4（実機ドローン2、TelloID 1）
   - オクルージョン時：重み行列=0、ξ=0、ψとτが有効
   - ref/pid_slaf_japanese_proof.texを参考
3. **ログにψ、τを追加**

## 実装内容

### 1. slaf_pid_controller.py の拡張

#### A. オクルージョンフラグの追加
```python
# オクルージョンフラグ
self.is_occluded = False  # センサオクルージョン状態

# 共線回避パラメータ
self.lambda_vec = np.array([0.1, 0.1])  # 調整ベクトル
self.tau_threshold = 0.1  # 共線判定閾値
```

#### B. calculate_psi_and_tau()メソッドの追加
```python
def calculate_psi_and_tau(self, neighbor_positions_hat, neighbor_positions_star):
    """
    共線回避項ψとbearing誤差τを計算
    
    Returns:
        psi: 共線回避項 (2x1)
        tau: bearing誤差ノルム（スカラー）
    """
    # τの計算（ref/pid_slaf_japanese_proof.tex Eq. 124）
    # τ_i = ||g_ij - g_ij^*||^2 + ||g_ik - g_ik^*||^2
    bearing_error_ij = g_ij_hat - g_ij_star
    bearing_error_ik = g_ik_hat - g_ik_star
    tau = np.linalg.norm(bearing_error_ij)**2 + np.linalg.norm(bearing_error_ik)**2
    
    # ψの計算（ref/pid_slaf_japanese_proof.tex Eq. 120）
    # ψ_i = -τ_i(sign(p̂_i - p_i^*) - λ_i)
    if tau < self.tau_threshold:
        psi = np.zeros(2)  # 非共線時
    else:
        tracking_error = self.p_hat - self.p_star
        sign_approx = np.tanh(tracking_error / 0.01)  # tanhで近似
        psi = -tau * (sign_approx - self.lambda_vec)
    
    return psi, tau
```

#### C. オクルージョン時のξ制御
```python
def calculate_xi(self, neighbor_positions_hat, neighbor_positions_star):
    # オクルージョン時はξ = 0
    if self.is_occluded:
        return np.zeros(2)
    
    # 通常時は幾何学的補正項を計算
    xi = calculate_xi_correction(...)
    return xi
```

#### D. 推定器と制御器にψを追加
```python
# 推定器更新（Eq. 149 with ψ）
def update_estimator(self, xi, psi=None):
    v_hat_dot = (
        - self.k_p * (self.p_hat - self.p_star)
        - self.k_v * (self.v_hat - self.v_actual)
        - self.k_i * self.z_integral
        + self.a_star
        + self.xi_gain * xi
        - psi  # 共線回避項（オクルージョン時に効果）
    )

# 制御器（Eq. 6 with ψ）
def calculate_control_input(self, psi=None):
    u = (
        - self.k_cp * (self.p_hat - self.p_star)
        - self.k_cv * (self.v_actual - self.v_star)
        + self.a_star
        + psi  # 共線回避項（オクルージョン時に効果）
    )
```

#### E. SLAFSystemManagerの拡張
```python
def set_follower_occlusion(self, follower_id, is_occluded):
    """
    特定のフォロワーのオクルージョン状態を設定
    
    Args:
        follower_id: フォロワーID（3 or 4）
        is_occluded: Trueの場合、センサオクルージョン状態
    """
    if follower_id in self.follower_controllers:
        self.follower_controllers[follower_id].set_occlusion(is_occluded)
```

### 2. mocap_slaf_main.py の拡張

#### A. オクルージョンキー制御
```python
# O: オクルージョンモードON（フォロワー4 = ドローン2）
if "o" in pressed_keys:
    if control_mode == "slaf" and slaf_manager:
        print("=" * 60)
        print("Oキー検出 - オクルージョンモードON")
        print("対象：フォロワー4（ドローン2、TelloID 1）")
        print("  - 重み行列 H = 0（隣接情報なし）")
        print("  - ξ = 0（幾何学的補正なし）")
        print("  - ψ, τが有効（共線回避動作）")
        print("=" * 60)
        slaf_manager.set_follower_occlusion(4, True)

# P: オクルージョンモードOFF
if "p" in pressed_keys:
    if control_mode == "slaf" and slaf_manager:
        print("=" * 60)
        print("Pキー検出 - オクルージョンモードOFF")
        print("フォロワー4のセンサ復旧")
        print("=" * 60)
        slaf_manager.set_follower_occlusion(4, False)
```

#### B. CSVログにτとis_occludedを追加
```python
log_data = {
    ...
    'xi': state['xi'],
    'psi': state['psi'],
    'tau': state['tau'],  # bearing誤差
    'is_collinear': state['is_collinear'],
    'is_occluded': state['is_occluded'],  # オクルージョン状態
    ...
}
```

### 3. csv_logger.py の拡張

#### CSVヘッダーの更新
```python
control_csv_writer.writerow([
    'timestamp', 'drone_id', 'role', 'mode',
    'x', 'y', 'z',  # 実際位置（MOCAP）
    'x_hat', 'y_hat', 'z_hat',  # 推定位置（オブザーバー）
    'vx', 'vy', 'vz',  # 実際速度
    'vx_hat', 'vy_hat', 'vz_hat',  # 推定速度（オブザーバー）
    'target_x', 'target_y', 'target_z',
    'error_x', 'error_y', 'error_z',
    'rc_lr', 'rc_fb', 'rc_ud', 'rc_yaw',
    'trust',
    'xi_x', 'xi_y', 'xi_z',  # 幾何学的補正項ξ
    'psi_x', 'psi_y', 'psi_z',  # 共線回避項ψ
    'tau',  # bearing誤差
    'is_collinear',  # 共線状態
    'is_occluded',  # オクルージョン状態
    'tracking_error', 'estimation_error'
])
```

## 理論的背景（ref/pid_slaf_japanese_proof.tex）

### Bearing誤差 τ（Eq. 124）
```
τ_i = ||g_ij - g_ij^*||² + ||g_ik - g_ik^*||²
```
- `g_ij = (p_j - p_i) / ||p_j - p_i||`：Bearing単位ベクトル
- `τ_i = 0 ⇔ 非共線（局所化可能）`

### 共線回避項 ψ（Eq. 120）
```
ψ_i = -τ_i(sign(p̂_i - p_i^*) - λ_i)
```
- Filippov解による不連続制御
- `τ_i > τ_threshold`の時に有効

### 推定器ダイナミクス（Eq. 149）
```
v̂̇_i = -k_p(p̂_i - p_i^*) - k_v(v̂_i - v_i) - k_i*z_i + a_i^* + xi_gain*ξ_i - ψ_i
```

### 制御器（Eq. 6 + Eq. 155）
```
u_i = -k_cp(p̂_i - p_i^*) - k_cv(v_i - v_i^*) + a_i^* + ψ_i
```

## 使用方法

### 基本操作
1. **Q**: 離陸
2. **T**: SLAF制御モード開始
3. **G/B/V/N**: 目標位置移動（任意）
4. **O**: オクルージョンON（フォロワー4）
   - ξ → 0
   - ψ, τが有効
5. **P**: オクルージョンOFF
   - 通常のSLAF制御に復帰
6. **E**: 着陸

### 期待される挙動
- **オクルージョン時**:
  - ξ = 0（幾何学的補正なし）
  - τが増加（Bearing誤差）
  - ψが非ゼロ（共線回避動作）
  - 軌道が変化する可能性

- **センサ復旧時**:
  - ξが復活
  - ψ → 0
  - 通常の追従挙動

## CSVログ分析

### ξの確認
```python
import pandas as pd
df = pd.read_csv('slaf_results/control_log_*.csv')
df4 = df[df['role'] == 'follower_4']

# ξのノルム
xi_norm = np.sqrt(df4['xi_x']**2 + df4['xi_z']**2)

# オクルージョン時にξ = 0であることを確認
print(df4[df4['is_occluded'] == 1]['xi_x'].describe())
```

### ψとτの確認
```python
# τの時系列
plt.plot(df4['timestamp'], df4['tau'])
plt.axhline(y=0.1, color='r', linestyle='--', label='threshold')
plt.xlabel('Time')
plt.ylabel('τ (Bearing Error)')

# ψのノルム
psi_norm = np.sqrt(df4['psi_x']**2 + df4['psi_z']**2)
plt.plot(df4['timestamp'], psi_norm)
plt.xlabel('Time')
plt.ylabel('||ψ||')
```

## 更新されたファイル

### コア実装
- ✅ `slaf_pid_controller.py`: オクルージョン機能、ψ・τ計算
- ✅ `mocap_slaf_main.py`: O/Pキー制御、ログ出力
- ✅ `csv_logger.py`: ξ, ψ, τ, is_collinear, is_occludedをCSVに追加

### ドキュメント
- ✅ `OCCLUSION_FEATURE.md`: オクルージョン機能の詳細説明（新規作成）
- ✅ `README.md`: 操作方法、CSVログの説明を更新
- ✅ `UPDATE_SUMMARY_20251125.md`: このファイル（新規作成）

## パラメータ

### 共線回避パラメータ
```python
self.lambda_vec = np.array([0.1, 0.1])  # 調整ベクトル
self.tau_threshold = 0.1  # 共線判定閾値（これより大きいとψが有効）
```

### 調整指針
- `lambda_vec`を大きく → ψの効果が強まる
- `tau_threshold`を小さく → より早く共線を検出

## 検証項目

1. **オクルージョン時の挙動**:
   - Oキー押下でξ = 0になることを確認
   - τが変化することを確認
   - ψが非ゼロになることを確認

2. **センサ復旧時の挙動**:
   - Pキー押下でξが復活することを確認
   - ψがゼロに戻ることを確認
   - 通常の追従に復帰することを確認

3. **CSVログの確認**:
   - ξ, ψ, τの値が正しく記録されていることを確認
   - is_occludedフラグが正しく記録されていることを確認

## 参考文献
- `ref/pid_slaf_japanese_proof.tex`: PID階層型SLAF制御の理論的証明
- Fang et al.: Bearing-based formation control
- Fischer et al. (2013): Filippov解による不連続制御
