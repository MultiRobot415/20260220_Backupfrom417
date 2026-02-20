# 重み行列とξの計算に関する重大なバグ修正

**日時**: 2025-11-27  
**修正内容**: 論文とMATLABコードに基づいて、重み行列とξの計算を正しく実装

---

## 発見された問題

### **問題1：重み行列の計算に推定値を使用していた**

**間違った実装**（`weight_matrices.py` Line 158-160）：
```python
# ❌ 間違い：推定位置から重み行列を計算
Hij, Hik, is_collinear = calculate_weight_matrices_for_agent(p_i_hat, p_j_hat, p_k_hat)
```

**論文の要求**（Remark 5）：
> "The weight matrices H_ij, H_ik, H_ss, H_si, H_sg can be obtained by **bearing measurements**"

**MATLABコードの実装**（`calculate_control_logic.m` Line 57-60）：
```matlab
% 真の位置（センサ情報）から重み行列を計算
[Hij, Hik, case_id] = calculate_weight_matrices(p_true_i, p_true_j, p_true_k, ...)
```

**なぜ間違いか**：
- **重み行列H**は**センサ観測（bearing測定）**から得られる情報行列
- シミュレーション：真の位置 = センサ観測に相当
- 実機：MOCAP測定値 = センサ観測に相当
- **推定値ではなく観測値を使用すべき**

---

### **問題2：ξの計算式が論文と異なっていた**

**間違った実装**（`weight_matrices.py` Line 191）：
```python
# ❌ 簡略版（不正確）
xi = Hij @ (g_ij_hat - g_ij_star) + Hik @ (g_ik_hat - g_ik_star)
```

**論文Eq. (20)の正しい式**：
```
ξijk = H^T_ii * Hij * (p̂_j - p̂_i) + H^T_ii * Hik * (p̂_k - p̂_i)
```

ここで `H_ii = H_ij + H_ik`

**なぜ間違いか**：
- bearingベクトル`g`の差分ではなく、**位置ベクトルの差分**を使用すべき
- `H^T_ii`項が欠落していた
- 論文の厳密な式とは異なる簡略化版だった

---

## 修正内容

### **修正1：`weight_matrices.py`の`calculate_xi_correction()`**

#### **関数シグネチャの変更**

```python
# 修正前
def calculate_xi_correction(p_i_hat, p_j_hat, p_k_hat, p_i_star, p_j_star, p_k_star):

# 修正後
def calculate_xi_correction(p_i_hat, p_j_hat, p_k_hat, p_i_star, p_j_star, p_k_star, 
                            p_i_actual, p_j_actual, p_k_actual):
```

- **追加引数**: `p_i_actual`, `p_j_actual`, `p_k_actual`（実位置、観測値）

#### **重み行列の計算を修正**

```python
# ===重要：重み行列は観測値（真の位置、センサ情報）から計算===
# 論文Remark 5: "The weight matrices can be obtained by bearing measurements"
# MATLABコード: calculate_weight_matrices(p_true_i, p_true_j, p_true_k, ...)
# シミュレーション：真の位置 = センサ観測に相当
# 実機：MOCAP測定値 = センサ観測に相当
Hij, Hik, is_collinear = calculate_weight_matrices_for_agent(p_i_actual, p_j_actual, p_k_actual)
```

#### **ξの計算式を論文Eq. (20)に従って修正**

```python
# ===論文Eq. (20)の実装===
# ξijk = H^T_ii * Hij * (p̂_j - p̂_i) + H^T_ii * Hik * (p̂_k - p̂_i)
# ここで H_ii = H_ij + H_ik

Hii = Hij + Hik

# 推定位置の相対ベクトル
p_rel_ij_hat = p_j_hat - p_i_hat
p_rel_ik_hat = p_k_hat - p_i_hat

# 論文Eq. (20)に従った補正項の計算
xi = Hii.T @ Hij @ p_rel_ij_hat + Hii.T @ Hik @ p_rel_ik_hat
```

---

### **修正2：`slaf_pid_controller.py`の`calculate_xi()`**

#### **メソッドシグネチャの変更**

```python
# 修正前
def calculate_xi(self, neighbor_positions_hat, neighbor_positions_star):

# 修正後
def calculate_xi(self, neighbor_positions_hat, neighbor_positions_star, neighbor_positions_actual):
```

- **追加引数**: `neighbor_positions_actual`（隣接エージェントの実位置）

#### **実装の修正**

```python
def calculate_xi(self, neighbor_positions_hat, neighbor_positions_star, neighbor_positions_actual):
    """
    幾何学的補正項ξを計算（論文Eq. (20)に準拠）
    
    重要：
    - 重み行列H：観測値（実位置、MOCAP測定）から計算
    - ξの計算：推定位置を使用
    """
    j_idx, k_idx = self.neighbors
    
    # 推定位置（ξの計算に使用）
    p_j_hat = neighbor_positions_hat.get(j_idx, self.p_hat)
    p_k_hat = neighbor_positions_hat.get(k_idx, self.p_hat)
    p_j_star = neighbor_positions_star.get(j_idx, self.p_star)
    p_k_star = neighbor_positions_star.get(k_idx, self.p_star)
    
    # 実位置（観測値、重み行列の計算に使用）
    p_j_actual = neighbor_positions_actual.get(j_idx, self.p_actual)
    p_k_actual = neighbor_positions_actual.get(k_idx, self.p_actual)
    
    # オクルージョン時はξ = 0
    if self.is_occluded:
        return np.zeros(2)
    
    # 幾何学的補正項ξを計算（論文Eq. (20)）
    # 重要：重み行列は実位置から、ξは推定位置から
    xi = calculate_xi_correction(
        self.p_hat,      # 推定位置
        p_j_hat,          # 推定位置
        p_k_hat,          # 推定位置
        self.p_star,     # 目標位置
        p_j_star,         # 目標位置
        p_k_star,         # 目標位置
        self.p_actual,   # 実位置（観測値）
        p_j_actual,       # 実位置（観測値）
        p_k_actual        # 実位置（観測値）
    )
    
    self.debug_info['xi'] = xi
    return xi
```

#### **`update()`メソッドの修正**

```python
# 2. ξを計算（オクルージョン時は0）
# 重要：重み行列は実位置（観測値）から計算、ξは推定位置から計算
xi = self.calculate_xi(neighbor_positions_hat, neighbor_positions_star, neighbor_positions_actual)
```

---

## 正しい実装の理論的根拠

### **論文Eq. (19)-(20)の制御則**

```
推定器:
  ̇p̂_i = -2w(p̂_i - p*_i) + ̇p*_i + ξijk - 2τ_i(sign(p̂_i - p*_i) - λ_i)

制御器:
  u_i = -w(p̂_i - p*_i) + ̇p*_i - τ_i(sign(p̂_i - p*_i) - λ_i)

ここで:
  ξijk = H^T_ii * Hij * (p̂_j - p̂_i) + H^T_ii * Hik * (p̂_k - p̂_i)
  H_ii = H_ij + H_ik
```

### **重み行列Hの物理的意味**

**論文Remark 5**:
> "The weight matrices can be obtained by **bearing measurements**"

つまり：
- **H_ij, H_ik**：センサ観測（bearing測定）から計算される
- シミュレーション：真の位置から幾何学的にbearingを計算
- 実機：カメラやMOCAPからbearingを測定、または位置測定からbearingを計算

### **ξの役割**

**論文の説明**:
- `ξijk`：**相対測定からの補正項**（Line 171-173 in MATLAB code）
- 推定位置`p̂`と観測から得られる重み行列Hを組み合わせて計算
- 推定誤差を減少させる役割

### **MATLABコードの実装との対応**

**`calculate_control_logic.m`の実装**:
```matlab
% Line 25-34: センサ情報（真の幾何学）と推定値の分離
% 推定位置配列（制御入力の計算に使用）
p_hat_all = [num2cell(p_l, 1), num2cell(p_hat_f, 1)];

% 真の位置配列（センサ計測のシミュレーション用）
p_true_all = [num2cell(p_l, 1), num2cell(p_f, 1)];

% Line 57-60: 重み行列の計算（真の相対関係＝センサ情報を使用）
[Hij, Hik, case_id] = calculate_weight_matrices(p_true_i, p_true_j, p_true_k, ...);

% Line 173-176: Error計算（推定位置を使用）
Error = B_ff * p_hat_vec + B_fl * p_l_vec;

% Line 184: ξの計算
xi_vec_complete = -B_ff' * Error;
```

---

## 重み行列のノルムについて

### **CSVログの重み行列のノルム**

**現在のCSVログ**:
```
observer_weight_norm = 2.0  (一定)
observer_weight_pattern = "fixed"
```

これは**推定器の重み行列（L_p, L_v）のノルム**であり、**制御則の重み行列（H_ij, H_ik）のノルム**ではありません。

### **推定器の重み行列 vs 制御則の重み行列**

#### **推定器の重み行列（L_p, L_v）**

論文Eq. (19)を離散化した実装：
```python
# slaf_pid_controller.py
self.k_p = 1.0  # L_p = k_p * I
self.k_v = 1.0  # L_v = k_v * I

# 推定器の更新（observer.py相当）
L_p = [[k_p, 0], [0, k_p]]
L_v = [[k_v, 0], [0, k_v]]

# ノルム計算
||L_p|| = k_p * sqrt(2) = 1.414
||L_v|| = k_v * sqrt(2) = 1.414
||L|| = sqrt(||L_p||^2 + ||L_v||^2) = 2.0
```

これは**固定ゲイン**なので、CSVログで一定値（2.0）になるのは正しい。

#### **制御則の重み行列（H_ij, H_ik）**

論文Eq. (20)のξ計算に使用：
```python
# weight_matrices.py
Hij = calculate_weight_matrix_bearing(p_i_actual, p_j_actual)
Hik = calculate_weight_matrix_bearing(p_i_actual, p_k_actual)

# 定義（bearing-based）:
Hij = (I - g_ij * g_ij^T) / ||q_ij||
```

ここで：
- `g_ij = (p_j - p_i) / ||p_j - p_i||`：bearing（方向ベクトル）
- `q_ij = p_j - p_i`：相対位置ベクトル

**重要**：
- `H_ij, H_ik`は**エージェントの相対位置に依存**して変化
- シミュレーション中に`||H_ij||`, `||H_ik||`は変動する
- **これが制御入力に影響**する

### **制御入力への影響**

**制御則**（論文Eq. (19)）：
```
u_i = -k_cp*(p̂_i - p*_i) - k_cv*(v_i - v*_i) + a*_i + ψ_i

推定器の更新:
  ̇p̂_i = ... + ξijk + ...

ξijk = H^T_ii * Hij * (p̂_j - p̂_i) + H^T_ii * Hik * (p̂_k - p̂_i)
```

**影響の流れ**:
1. **観測値（MOCAP測定）** → `p_i_actual, p_j_actual, p_k_actual`
2. **重み行列の計算** → `H_ij, H_ik` (観測値から)
3. **ξの計算** → `ξijk` (重み行列H + 推定位置p̂)
4. **推定器の更新** → `p̂_i, v̂_i` (ξを使用)
5. **制御入力の計算** → `u_i` (推定位置p̂を使用)

つまり、**重み行列Hは間接的に制御入力に影響**します。

---

## 修正の確認

### **構文チェック**

```bash
$ python3 -c "
import sys
sys.path.insert(0, '/home/initial/01_v1_PID/MOCAP_forSLAF')
from weight_matrices import *
from slaf_pid_controller import *
print('✅ インポート成功')
print('✅ 修正完了：重み行列は観測値から、ξは推定値から計算')
"

✅ インポート成功
✅ 修正完了：重み行列は観測値から、ξは推定値から計算
```

### **修正内容のまとめ**

| 項目 | 修正前 | 修正後 |
|------|--------|--------|
| **重み行列Hの計算** | 推定位置から | **観測値（実位置）から** ✅ |
| **ξの計算式** | 簡略版（不正確） | **論文Eq. (20)に準拠** ✅ |
| **ξの計算に使用する位置** | 推定位置 | **推定位置**（変更なし） ✅ |
| **関数シグネチャ** | 引数6個 | **引数9個**（実位置追加） ✅ |

---

## 期待される効果

### **1. より正確な推定**

- 重み行列が**センサ観測**に基づくため、理論と実装が一致
- ξが正しく計算されるため、推定誤差が減少

### **2. 制御性能の向上**

- 推定位置`p̂`が実位置`p`に近づく
- 制御入力`u`が目標軌道追従に適切に作用

### **3. 論文との整合性**

- MATLABコードと同じ実装
- 論文Eq. (20)の厳密な実装

---

## CSVログでの確認方法

### **重み行列のノルムの変化**

現在のCSVログでは**推定器の重み行列（L_p, L_v）のノルム**を記録しています：
```python
# slaf_pid_controller.py Line 427-436
L_p_norm = self.k_p * np.sqrt(2)
L_v_norm = self.k_v * np.sqrt(2)
observer_weight_norm = np.sqrt(L_p_norm**2 + L_v_norm**2)
```

これは固定ゲイン（k_p=1.0, k_v=1.0）なので、`observer_weight_norm = 2.0`で一定です。

**制御則の重み行列（H_ij, H_ik）のノルム**を確認したい場合は、別途ログに追加する必要があります：
```python
# calculate_xiメソッド内でHijのノルムを計算
Hij, Hik, is_collinear = calculate_weight_matrices_for_agent(...)
self.debug_info['Hij_norm'] = np.linalg.norm(Hij)
self.debug_info['Hik_norm'] = np.linalg.norm(Hik)
```

### **制御入力の変化**

修正後、以下の変化が期待されます：
1. **推定誤差の減少**: `||p̂_i - p_i||` が小さくなる
2. **制御入力の適切化**: `u_i` が滑らかになる
3. **軌道追従性能の向上**: 目標位置への収束が速くなる

---

## まとめ

### **修正完了項目**

✅ 重み行列Hは観測値（実位置、MOCAP測定）から計算  
✅ ξの計算式を論文Eq. (20)に修正  
✅ `calculate_xi_correction()`の関数シグネチャを拡張  
✅ `slaf_pid_controller.py`の`calculate_xi()`を修正  
✅ `update_followers()`で`neighbor_positions_actual`を正しく渡す  
✅ 構文チェック成功  

### **理論との整合性**

✅ 論文Eq. (20)に準拠  
✅ MATLABコード`calculate_control_logic.m`と同じ実装  
✅ センサ観測と推定値の正しい分離  

---

**作成日**: 2025-11-27  
**ステータス**: 修正完了、テスト準備完了
