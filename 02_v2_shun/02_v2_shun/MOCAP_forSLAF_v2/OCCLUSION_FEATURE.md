# センサオクルージョン機能

## 概要
本機能は、センサオクルージョン（隣接エージェントとの相対位置測定不可状態）を仮想的に発生させ、共線回避項ψとbearing誤差τの効果を検証するための機能です。

## 理論的背景
ref/pid_slaf_japanese_proof.tex に基づく実装

### 通常時（オクルージョンなし）
- **重み行列**: H ≠ 0（隣接情報あり）
- **幾何学的補正項**: ξ ≠ 0（Bearingベース協調制御）
- **共線回避項**: ψ = 0（局所化可能）
- **Bearing誤差**: τ = ||g_ij - g_ij^*||² + ||g_ik - g_ik^*||² ≈ 0

### オクルージョン時
- **重み行列**: H = 0（隣接情報なし）
- **幾何学的補正項**: ξ = 0（相対測定不可）
- **共線回避項**: ψ ≠ 0（共線回避動作）
- **Bearing誤差**: τ > τ_threshold（共線検出）

## 数学的定義

### Bearing誤差 τ（Eq. 124）
```
τ_i = ||g_ij - g_ij^*||² + ||g_ik - g_ik^*||²
```
ここで：
- `g_ij = (p_j - p_i) / ||p_j - p_i||`：Bearing単位ベクトル（推定位置ベース）
- `g_ij^* = (p_j^* - p_i^*) / ||p_j^* - p_i^*||`：目標Bearing単位ベクトル

### 共線回避項 ψ（Eq. 120）
```
ψ_i = -τ_i(sign(p̂_i - p_i^*) - λ_i)
```
ここで：
- `τ_i`：Bearing誤差
- `sign(·)`：符号関数（実装ではtanhで近似）
- `λ_i`：調整ベクトル（0 < ||λ_i|| < 1）

### 推定器ダイナミクス（Eq. 149 with ψ）
```
v̂̇_i = -k_p(p̂_i - p_i^*) - k_v(v̂_i - v_i) - k_i*z_i + a_i^* + xi_gain*ξ_i - ψ_i
```

### 制御器（Eq. 6 with ψ）
```
u_i = -k_cp(p̂_i - p_i^*) - k_cv(v_i - v_i^*) + a_i^* + ψ_i
```

## 実装仕様

### 対象フォロワー
- **フォロワー4**（実機ドローン2、TelloID 1）
- 隣接エージェント：リーダー1、フォロワー3

### オクルージョン状態の切り替え

#### オクルージョンON（Oキー）
```python
slaf_manager.set_follower_occlusion(4, True)
```
**効果**：
1. `is_occluded = True`フラグ設定
2. `ξ = 0`（calculate_xi()で即座に0を返す）
3. `ψ, τ`を計算して制御に使用
4. 推定器：`v̂̇ = ... - ψ`
5. 制御器：`u = ... + ψ`

#### オクルージョンOFF（Pキー）
```python
slaf_manager.set_follower_occlusion(4, False)
```
**効果**：
1. `is_occluded = False`フラグ解除
2. `ξ ≠ 0`（通常の幾何学的補正に復帰）
3. `ψ ≈ 0`（τが小さいため）
4. 通常のSLAF制御に復帰

## 使用方法

### 基本操作フロー
1. **Q**: 離陸
2. **T**: SLAF制御モード開始
3. **G/B/V/N**: 目標位置移動（任意）
4. **O**: オクルージョンON
   - フォロワー4のセンサが失われた状態をシミュレート
   - ξ = 0、ψとτが制御に効果
5. **P**: オクルージョンOFF
   - フォロワー4のセンサが復旧
   - 通常のSLAF制御に復帰
6. **E**: 着陸

### 期待される挙動
- **オクルージョン時**:
  - フォロワー4は幾何学的補正（ξ）を失う
  - 共線状態に近づくと、τが増加
  - ψが非ゼロとなり、共線回避動作が発動
  - 軌道が通常時と異なる可能性がある

- **センサ復旧時**:
  - ξが再び有効になり、協調制御が復活
  - ψは再びゼロに近づく
  - 通常の追従挙動に戻る

## CSVログ出力

### 追加された列
```csv
xi_x, xi_y, xi_z         # 幾何学的補正項ξ
psi_x, psi_y, psi_z      # 共線回避項ψ
tau                       # Bearing誤差
is_collinear              # 共線状態（0 or 1）
is_occluded               # オクルージョン状態（0 or 1）
```

### データ分析例
```python
import pandas as pd
import matplotlib.pyplot as plt

# CSVロード
df = pd.read_csv('slaf_results/control_log_*.csv')

# フォロワー4のデータ抽出
df4 = df[df['role'] == 'follower_4']

# オクルージョン期間の特定
occlusion_periods = df4[df4['is_occluded'] == 1]

# τの時系列プロット
plt.figure(figsize=(12, 4))
plt.subplot(1, 3, 1)
plt.plot(df4['timestamp'], df4['tau'], label='τ')
plt.axhline(y=0.1, color='r', linestyle='--', label='threshold')
plt.xlabel('Time (s)')
plt.ylabel('τ (Bearing Error)')
plt.legend()
plt.title('Bearing Error')

# ξのノルムプロット
plt.subplot(1, 3, 2)
xi_norm = np.sqrt(df4['xi_x']**2 + df4['xi_z']**2)
plt.plot(df4['timestamp'], xi_norm, label='||ξ||')
plt.xlabel('Time (s)')
plt.ylabel('||ξ||')
plt.legend()
plt.title('Geometric Correction')

# ψのノルムプロット
plt.subplot(1, 3, 3)
psi_norm = np.sqrt(df4['psi_x']**2 + df4['psi_z']**2)
plt.plot(df4['timestamp'], psi_norm, label='||ψ||')
plt.xlabel('Time (s)')
plt.ylabel('||ψ||')
plt.legend()
plt.title('Collinearity Avoidance')

plt.tight_layout()
plt.show()
```

## パラメータ

### 共線回避パラメータ（slaf_pid_controller.py）
```python
self.lambda_vec = np.array([0.1, 0.1])  # 調整ベクトル
self.tau_threshold = 0.1  # 共線判定閾値
```

### 調整指針
- `lambda_vec`を大きくする → ψの効果が強まる
- `tau_threshold`を小さくする → より早く共線を検出

## デバッグ情報

### コンソール出力（オクルージョンON時）
```
============================================================
Oキー検出 - オクルージョンモードON
対象：フォロワー4（ドローン2、TelloID 1）
  - 重み行列 H = 0（隣接情報なし）
  - ξ = 0（幾何学的補正なし）
  - ψ, τが有効（共線回避動作）
============================================================
```

### コンソール出力（オクルージョンOFF時）
```
============================================================
Pキー検出 - オクルージョンモードOFF
フォロワー4のセンサ復旧
============================================================
```

## 検証項目

### 実験1: オクルージョン時の挙動
1. SLAF制御モードで安定した追従を確認
2. Oキーでオクルージョン発生
3. τの変化を観測
4. ψが非ゼロになることを確認
5. 軌道の変化を記録

### 実験2: センサ復旧時の挙動
1. オクルージョン状態で一定時間飛行
2. Pキーでセンサ復旧
3. ξが復活することを確認
4. ψがゼロに戻ることを確認
5. 通常の追従挙動に復帰することを確認

### 実験3: 共線状態の検出
1. 意図的に共線に近い配置にする（目標位置操作）
2. τが閾値を超えるか確認
3. ψの値が変化するか確認

## トラブルシューティング

### τが常にゼロ
- フォロワー配置が適切か確認
- 目標位置が正しく設定されているか確認
- `calculate_psi_and_tau()`の計算を確認

### ψが効果を持たない
- オクルージョンフラグが正しく設定されているか確認
- τが閾値を超えているか確認
- 推定器と制御器にψが渡されているか確認

### 制御が不安定になる
- `lambda_vec`を小さくする（0.05程度）
- `tau_threshold`を調整
- ゲイン`k_p`, `k_cp`を確認

## 参考文献
- `ref/pid_slaf_japanese_proof.tex`: 理論的証明
- Fang et al.: Bearing-based formation control
- Fischer et al. (2013): Filippov解による不連続制御の解析
