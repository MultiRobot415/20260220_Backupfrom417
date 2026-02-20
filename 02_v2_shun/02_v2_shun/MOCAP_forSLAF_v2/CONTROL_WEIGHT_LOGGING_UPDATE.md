# 制御則の重み行列ログ出力と目標加速度の変更

**日時**: 2025-11-27  
**変更内容**: 推定器の重み行列から制御則の重み行列H_iiへのログ出力変更と目標加速度の更新

---

## 変更内容

### **1. 制御則の重み行列H_iiのノルムをログに出力**

#### **変更前：推定器の重み行列のノルム**

```python
# slaf_pid_controller.py - get_state()
# 推定器の重み行列（L_p, L_v）のノルム
L_p_norm = self.k_p * np.sqrt(2)
L_v_norm = self.k_v * np.sqrt(2)
observer_weight_norm = np.sqrt(L_p_norm**2 + L_v_norm**2)  # 固定値 2.0
observer_weight_pattern = "fixed"
```

**問題点**：
- 推定器ゲイン（k_p=1.0, k_v=1.0）は固定なので、ノルムは常に2.0
- 制御則に直接使用される重み行列H_ij, H_ikの情報が得られない

---

#### **変更後：制御則の重み行列H_iiのノルム**

```python
# slaf_pid_controller.py - calculate_xi()
# 制御則の重み行列を計算（観測値から）
from weight_matrices import calculate_weight_matrices_for_agent
Hij, Hik, is_collinear = calculate_weight_matrices_for_agent(
    self.p_actual, p_j_actual, p_k_actual)

if not is_collinear:
    Hii = Hij + Hik
    # 重み行列H_iiのノルムを保存
    self.debug_info['Hii_norm'] = np.linalg.norm(Hii)
else:
    self.debug_info['Hii_norm'] = 0.0
```

```python
# slaf_pid_controller.py - get_state()
# 制御則の重み行列H_iiのノルムを取得
control_weight_norm = self.debug_info.get('Hii_norm', 0.0)

return {
    # ...
    'control_weight_norm': control_weight_norm,  # 制御則の重み行列H_iiのノルム
    # 削除: 'observer_weight_norm', 'observer_weight_pattern'
}
```

**利点**：
- エージェント間の相対位置に応じて変化する重み行列のノルムが記録される
- 制御則への影響を直接確認できる
- H_iiは論文Eq. (20)のξ計算で使用される実際の重み行列

---

### **2. 目標加速度の変更**

#### **変更前**
```python
# mocap_slaf_main.py
target_acceleration_2d = np.array([0.0, -0.08])  # [X, Z] m/s^2
```

#### **変更後**
```python
# mocap_slaf_main.py
target_acceleration_2d = np.array([0.01, -0.08])  # [X, Z] m/s^2
```

**変更理由**：
- X方向に微小な加速度成分（0.01 m/s²）を追加
- より複雑な軌道での制御性能を評価

---

## 修正ファイル一覧

### **1. `slaf_pid_controller.py`**

**変更箇所**：
- `calculate_xi()`メソッド：制御則の重み行列H_iiのノルムを計算・保存
- `get_state()`メソッド：推定器の重み行列を削除、制御則の重み行列を返す

```python
# Line 229-239: calculate_xi()内でH_iiのノルムを計算
from weight_matrices import calculate_weight_matrices_for_agent
Hij, Hik, is_collinear = calculate_weight_matrices_for_agent(
    self.p_actual, p_j_actual, p_k_actual)

if not is_collinear:
    Hii = Hij + Hik
    self.debug_info['Hii_norm'] = np.linalg.norm(Hii)
else:
    self.debug_info['Hii_norm'] = 0.0

# Line 454-473: get_state()で制御則の重み行列のノルムを返す
control_weight_norm = self.debug_info.get('Hii_norm', 0.0)
return {
    # ...
    'control_weight_norm': control_weight_norm,  # 制御則の重み行列H_iiのノルム
    # 削除: 'observer_weight_norm', 'observer_weight_pattern'
}
```

---

### **2. `csv_logger.py`**

**変更箇所**：
- CSVヘッダー：`observer_weight_norm`, `observer_weight_pattern` → `control_weight_norm`
- データ行：`log_data.get('observer_weight_norm')` → `log_data.get('control_weight_norm')`

```python
# Line 74-75: CSVヘッダー
'control_weight_norm',  # 制御則の重み行列H_iiのノルム
# 削除: 'observer_weight_norm', 'observer_weight_pattern'

# Line 527: データ行
log_data.get('control_weight_norm', 0.0),  # 制御則の重み行列H_iiのノルム
```

---

### **3. `mocap_slaf_main.py`**

**変更箇所**：
- 目標加速度の定義
- log_dataの重み行列項目

```python
# Line 48: 目標加速度
target_acceleration_2d = np.array([0.01, -0.08])  # [X, Z] m/s^2

# Line 665: log_data
'control_weight_norm': state['control_weight_norm'],  # 制御則の重み行列H_iiのノルム
# 削除: 'observer_weight_norm', 'observer_weight_pattern'
```

---

## 制御則の重み行列H_iiについて

### **定義（論文Eq. (20)）**

```
ξijk = H^T_ii * Hij * (p̂_j - p̂_i) + H^T_ii * Hik * (p̂_k - p̂_i)

H_ii = H_ij + H_ik
```

### **計算方法（bearing-based）**

```python
# weight_matrices.py
Hij = (I - g_ij * g_ij^T) / ||q_ij||
Hik = (I - g_ik * g_ik^T) / ||q_ik||

Hii = Hij + Hik
```

ここで：
- `g_ij = (p_j - p_i) / ||p_j - p_i||`：bearing（方向ベクトル）
- `q_ij = p_j - p_i`：相対位置ベクトル

### **物理的意味**

- **H_ij, H_ik**：センサ観測（bearing測定）から得られる情報行列
- **H_ii**：エージェントiの幾何学的情報を統合した行列
- **||H_ii||**：制御則への幾何学的情報の寄与度を表す

### **特性**

1. **位置依存性**：エージェント間の相対位置によって変化
2. **共線性**：共線状態では||H_ii|| = 0
3. **制御への影響**：ξを通じて推定器に影響→推定位置→制御入力

---

## CSVログでの確認方法

### **新しいログフォーマット**

```csv
timestamp,drone_id,follower_id,mode,...,control_weight_norm,k_p,k_v,k_cv
1764232216.45,0,3,slaf,...,1.234,1.0,1.0,1.0
```

### **確認項目**

1. **control_weight_norm**：
   - 時間経過とともに変化することを確認
   - 共線状態では0に近い値になることを確認
   - エージェント間距離が遠いほど小さくなることを確認（1/||q_ij||に比例）

2. **目標加速度の影響**：
   - X方向の目標位置が時間とともに増加することを確認
   - 制御入力のX成分が非ゼロになることを確認

---

## 期待される効果

### **1. より詳細な制御解析**

- 制御則に実際に使用される重み行列の挙動が記録される
- エージェント配置と制御性能の関係を分析可能

### **2. 共線状態の検出**

- ||H_ii|| ≈ 0 のとき、エージェントが共線状態にあることが分かる
- ψ（共線回避項）の作用タイミングとの対応を確認可能

### **3. 軌道追従性能の評価**

- X方向加速度による軌道変化を評価
- より動的な軌道での制御性能を確認

---

## まとめ

### **修正完了項目**

✅ 制御則の重み行列H_iiのノルムを計算・ログ出力  
✅ 推定器の重み行列のログ出力を削除  
✅ 目標加速度を[0.01, -0.08]に変更  
✅ CSVヘッダーとデータ行を修正  
✅ 全ファイルの整合性を確認  
✅ 構文チェック成功  

### **変更の意義**

- **理論との整合性**：論文Eq. (20)で使用される実際の重み行列H_iiを記録
- **実用性**：エージェント配置に依存する動的な情報を取得
- **デバッグ性**：共線状態や制御性能の問題を特定しやすい

---

**作成日**: 2025-11-27  
**ステータス**: 修正完了、テスト準備完了
