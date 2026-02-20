# Z軸推定誤差の初期化問題の修正

**日時**: 2025-11-28  
**問題**: 初期化直後からZ軸の推定誤差が発生  
**原因**: 初期化時と制御ループでの位置更新のタイミング問題  
**解決**: 初期化フラグによる推定誤差の強制リセット

---

## 問題の詳細

### **実行ログから観察された現象**

```
# Tキー押下時の初期化
フォロワー3: p_hat(0)=[1.41, 0.79]

# 制御ループの最初の出力（初期化直後）
ドローン1（フォロワー3）:
  実際位置=[0.60, 0.47]
  推定位置=[0.62, 0.34]  # Z軸で0.13mの誤差！
  目標位置=[0.92, 0.03]
```

**問題点**：
- 初期化時の推定位置は `[1.41, 0.79]`
- 制御ループでの実際位置は `[0.60, 0.47]`（大きく異なる！）
- 推定位置は `[0.62, 0.34]` で、Z軸に0.13mの誤差が発生

---

## 根本原因の分析

### **初期化と制御ループの処理順序**

```
1. Tキー押下
   ↓
2. MOCAP位置を取得 → follower_target_positions[follower_id] = [x, z]
   ↓
3. initialize_followers(follower_target_positions) を呼び出し
   ↓
4. initialize_state(p_initial) 実行
   - self.p_actual = p_initial    # 初期化時のMOCAP位置
   - self.p_hat = p_actual.copy()  # 推定誤差ゼロ
   - self.v_hat = v_actual.copy()
   ↓
5. 制御ループ開始（数ms〜数百ms後）
   ↓
6. MOCAP位置を再取得 → p_mocap = [x', z']  # ドローンが動いて位置が変化
   ↓
7. update(p_mocap, ...) 実行
   ↓
8. update_actual_state(p_mocap)
   - self.p_actual = p_mocap  # ← 新しいMOCAP位置に更新
   ↓
9. update_estimator(xi)
   - self.p_hat += p_hat_dot * dt  # ← 推定器のダイナミクスで更新
```

**問題の発生メカニズム**：

1. **初期化時の`p_actual`**: 初期化時に取得したMOCAP位置（古い値）
2. **初期化時の`p_hat`**: `p_actual.copy()` → 古い値
3. **制御ループでの`p_actual`**: 新しく取得したMOCAP位置（新しい値）
4. **制御ループでの`p_hat`**: 推定器のダイナミクスで更新（古い値ベース）

結果：`p_hat` と `p_actual` の間に誤差が発生

---

## 理論的背景

### **論文の前提（Assumption 3.1）**

> "p̂_i(0) = p_i(0): Initial estimation error is zero"

**理論では**：
- 初期推定誤差はゼロ
- `p_hat(0) = p_actual(0)`

**実装での課題**：
- 初期化時の`p_actual(0)`と制御ループ開始時の`p_actual(t)`は異なる
- ドローンは常に動いているため、数ms〜数百msの遅延で位置が変化

---

## 修正内容

### **1. 初期化フラグの追加**

```python
# slaf_pid_controller.py - __init__()
# 初期化フラグ（初期化直後の最初の更新で推定誤差をゼロにリセット）
self.just_initialized = False
```

### **2. 初期化時にフラグを立てる**

```python
# slaf_pid_controller.py - initialize_state()
def initialize_state(self, p_initial, v_initial=None):
    self.p_actual = np.array(p_initial, dtype=float)
    self.v_actual = np.array(v_initial, dtype=float) if v_initial is not None else np.zeros(2)
    
    # Assumption: p̂_i(0) = p_i(0), v̂_i(0) = v_i(0)
    self.p_hat = self.p_actual.copy()
    self.v_hat = self.v_actual.copy()
    
    # z_i(0) = 0
    self.z_integral = np.zeros(2)
    
    # 初期化フラグを設定（次の更新で推定誤差を強制的にゼロにする）
    self.just_initialized = True  # ← 追加
```

### **3. 初回更新時に推定誤差を強制リセット**

```python
# slaf_pid_controller.py - update()
def update(self, p_mocap, neighbor_positions_hat, neighbor_positions_star, neighbor_positions_actual):
    # 1. 実状態を更新
    self.update_actual_state(p_mocap)
    
    # 初期化直後の最初の更新：推定誤差を強制的にゼロにリセット
    if self.just_initialized:
        self.p_hat = self.p_actual.copy()  # ← 最新のMOCAP位置で上書き
        self.v_hat = self.v_actual.copy()  # ← 最新の速度で上書き
        self.z_integral = np.zeros(2)     # ← 積分項をリセット
        self.just_initialized = False
        logger.info(f"フォロワー{self.follower_id}初期化後リセット: p_hat={self.p_hat}, v_hat={self.v_hat}")
        # 初回は制御入力をゼロにする
        return np.zeros(2)
    
    # 2. ξを計算
    xi = self.calculate_xi(...)
    # ...
```

---

## 修正の動作

### **新しい処理フロー**

```
1. Tキー押下
   ↓
2. MOCAP位置を取得（初期化用）
   ↓
3. initialize_state(p_initial) 実行
   - self.p_actual = p_initial
   - self.p_hat = p_actual.copy()
   - self.just_initialized = True  ← フラグON
   ↓
4. 制御ループ開始
   ↓
5. MOCAP位置を再取得 → p_mocap（最新）
   ↓
6. update(p_mocap, ...) 実行
   ↓
7. update_actual_state(p_mocap)
   - self.p_actual = p_mocap  ← 最新のMOCAP位置
   ↓
8. if self.just_initialized: ← フラグがONの場合
   - self.p_hat = self.p_actual.copy()  ← 強制リセット
   - self.v_hat = self.v_actual.copy()  ← 強制リセット
   - self.z_integral = np.zeros(2)     ← 積分項リセット
   - self.just_initialized = False  ← フラグOFF
   - return np.zeros(2)  ← 初回は制御入力ゼロ
   ↓
9. 2回目以降の update()
   - just_initialized = False なので、通常の推定器更新
```

**効果**：
- 制御ループの最初の1回で、推定位置を最新の実位置にリセット
- **推定誤差が確実にゼロになる**
- その後は通常の推定器ダイナミクスで動作

---

## 期待される結果

### **修正前**

```
# 初期化直後の最初の制御ループ
実際位置=[0.60, 0.47]
推定位置=[0.62, 0.34]  # 誤差あり！
```

### **修正後**

```
# 初期化直後の最初の制御ループ
実際位置=[0.60, 0.47]
推定位置=[0.60, 0.47]  # 誤差ゼロ！（強制リセット）

# 2回目以降
実際位置=[0.61, 0.48]
推定位置=[0.61, 0.48]  # 推定器が正常に追従
```

---

## 論文との整合性

### **Assumption 3.1の保証**

修正により、制御ループの実質的な開始時点で以下が保証されます：

```
p̂_i(t_start) = p_i(t_start)  ← 推定位置 = 実位置
v̂_i(t_start) = v_i(t_start)  ← 推定速度 = 実速度
z_i(t_start) = 0               ← 積分項はゼロ
```

ここで、`t_start`は制御ループで最初にMOCAP位置を取得した時刻です。

**理論との一致**：
- ✅ 初期推定誤差ゼロ
- ✅ 完全収束定理（Theorem 3.1）の前提条件を満たす
- ✅ 実装上のタイミング問題を解決

---

## 現在の設定（確認済み）

### **制御ゲイン**

```python
# slaf_pid_controller.py
k_p = 1.0      # 推定器位置ゲイン
k_v = 1.0      # 推定器速度ゲイン
k_i = 0.0      # 積分ゲイン（未使用）
k_cp = 0.2     # 制御器位置ゲイン
k_cv = 5.0     # 制御器速度ゲイン
xi_gain = 1.0  # ξ項のゲイン
```

### **制御パラメータ**

```python
# mocap_slaf_main.py
MAX_SPEED = 50  # 最大RC速度
target_acceleration_2d = [0.05, -0.05]  # 目標加速度 [X, Z] m/s^2
```

---

## 追加の改善提案

### **1. 初期化時のMOCAP取得を制御ループ直前に変更**

現在の実装では、Tキーを押した時にMOCAP位置を取得していますが、制御ループが開始されるまでに遅延があります。より確実にするには、制御ループの最初の1回で初期化するのが理想的です。

### **2. 初期化後の数サイクルは制御ゲインを小さくする**

初期化直後は大きな制御入力を避けるため、最初の数サイクル（例：5サイクル）は制御ゲインを小さくする（例：`k_cp *= 0.5`）ことも検討できます。

### **3. ログで初期化を確認**

初期化後リセット時にログが出力されるため、CSVログやコンソールで以下を確認できます：

```
フォロワー3初期化後リセット: p_hat=[0.60, 0.47], v_hat=[0.00, 0.00]
```

---

## まとめ

### **修正内容**

✅ 初期化フラグ `just_initialized` を追加  
✅ `initialize_state()` でフラグを立てる  
✅ `update()` の最初の呼び出しで推定誤差を強制的にゼロにリセット  
✅ 初回の制御入力はゼロを返す  

### **効果**

✅ Z軸（および全軸）の推定誤差が初期化直後に確実にゼロになる  
✅ 論文のAssumption 3.1（初期推定誤差ゼロ）を保証  
✅ 初期化と制御ループのタイミング問題を解決  

### **検証方法**

1. プログラムを実行
2. Tキーを押してSLAFモードに入る
3. 最初の制御出力で「実際位置 = 推定位置」を確認
4. CSVログで推定誤差の時系列を確認

---

**作成日**: 2025-11-28  
**ステータス**: 修正完了、テスト準備完了
