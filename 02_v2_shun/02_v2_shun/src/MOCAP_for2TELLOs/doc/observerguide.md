# MOCAP for 2 TELLOs オブザーバーシステム設計ガイド

## 🎆 2025-08-07 更新: リーダー交代時の目標位置継承機能完成

リーダー交代時の目標位置継承バグを完全修正し、信頼度ベースの動的リーダー交代機能が実機で成功しました。

### ✅ 修正完了機能
- **リーダー交代検出**: `previous_leader_idx`管理ロジックの修正
- **目標位置継承**: 新リーダーが旧リーダーの目標位置を動的継承
- **即座信頼度応答**: `fault_counter_threshold = 1`で即座リーダー交代
- **フォーメーション再構成**: リーダー交代後のフォロワー位置自動調整

## 1. オブザーバーシステム概要

### 1.1 設計思想

本システムのオブザーバーは、**論文通りのルーエンバーガーオブザーバ**と**残差解析による故障検知**を組み合わせた設計です。

#### 基本原理
```
正常モデルによる推定 ⟷ 実際の測定値 = 残差
残差の増大 → 故障検知 → 信頼度低下 → リーダー交代
```

### 1.2 アーキテクチャ

```
DroneObserver (複数ドローン管理)
├── StateObserver (ドローン1)
│   ├── カルマンフィルタ
│   ├── 残差計算
│   └── 故障検知
├── StateObserver (ドローン2)
│   ├── カルマンフィルタ
│   ├── 残差計算
│   └── 故障検知
└── LeaderSelector (リーダー交代判定)
```

## 2. StateObserver詳細設計

### 2.1 状態空間モデル

#### 状態ベクトル
```python
x = [px, py, pz, vx, vy, vz]^T  # 位置と速度（6次元）
```

#### システムモデル
```python
# 状態遷移行列 (6x6)
A = [[1, 0, 0, dt, 0,  0 ],
     [0, 1, 0, 0,  dt, 0 ],
     [0, 0, 1, 0,  0,  dt],
     [0, 0, 0, 1,  0,  0 ],
     [0, 0, 0, 0,  1,  0 ],
     [0, 0, 0, 0,  0,  1 ]]

# 観測行列 (3x6) - 位置のみ観測
C = [[1, 0, 0, 0, 0, 0],
     [0, 1, 0, 0, 0, 0],
     [0, 0, 1, 0, 0, 0]]
```

#### オブザーバーゲイン
```python
# 固定オブザーバーゲイン（論文通り）
L = np.array([
    [0.5, 0, 0],
    [0, 0.5, 0], 
    [0, 0, 0.5],
    [0.7, 0, 0],
    [0, 0.7, 0],
    [0, 0, 0.7]
])
```

### 2.2 ルーエンバーガーオブザーバ実装

#### 予測ステップ（論文通り）
```python
def predict(self, control_input=None):
    # 状態予測（システムモデル）
    self.x_hat = self.A @ self.x_hat
    
    # 制御入力の考慮（重要：正常制御入力のみ使用）
    if control_input is not None:
        acceleration = self._rc_to_acceleration(control_input)
        self.x_hat[3:6] += acceleration * self.dt
```

#### 更新ステップ（論文通り）
```python
def update(self, measurement):
    # 残差計算
    z = measurement.reshape(3, 1)
    y_hat = self.C @ self.x_hat.reshape(6, 1)
    self.residual = (z - y_hat).flatten()
    
    # オブザーバー状態更新（固定ゲインL使用）
    self.x_hat = self.x_hat + (self.L @ self.residual).flatten()
```

### 2.3 制御入力の考慮

#### RCコマンドから加速度への変換
```python
def _rc_to_acceleration(self, rc_values):
    """RCコマンド [roll, pitch, throttle, yaw] を加速度に変換"""
    roll, pitch, throttle, yaw = rc_values
    
    # 簡易的な変換（実際の機体特性に基づく）
    ax = pitch * 0.01   # ピッチ → X軸加速度
    ay = throttle * 0.01  # スロットル → Y軸加速度  
    az = roll * 0.01    # ロール → Z軸加速度
    
    return np.array([ax, ay, az])
```

## 3. 故障検知アルゴリズム

### 3.1 残差ベース故障検知

#### 基本原理
```python
# 残差ノルム計算
residual_norm = np.linalg.norm(self.residual)

# しきい値判定
if residual_norm > self.fault_threshold:
    self.fault_counter += 1
else:
    self.fault_counter = max(0, self.fault_counter - 1.0)
```

#### パラメータ設定
```python
self.fault_threshold = 0.5  # 残差しきい値（調整済み）
self.fault_counter_threshold = 3  # 連続検知回数
```

### 3.2 信頼度計算

#### 段階的信頼度低下
```python
def get_trust_metric(self):
    if self.fault_counter >= self.fault_counter_threshold:
        # 継続的な故障による指数的減衰
        excess_time = self.fault_counter - self.fault_counter_threshold
        decay_factor = np.exp(-excess_time / 10.0)
        trust = max(0.1, decay_factor)
    else:
        # 軽微な異常による線形減少
        normalized_counter = self.fault_counter / self.fault_counter_threshold
        trust = 1.0 - 0.3 * normalized_counter
    
    return float(trust)
```

## 4. 故障注入時の残差問題の考察

### 4.1 問題の現象

**観測された問題**
- 故障注入（Fキー）時にオブザーバーの残差が期待通り増大しない
- 信頼度が十分に低下せず、リーダー交代が発動しない

### 4.2 理論的な動作原理

#### 正常時の動作
```
オブザーバー: 正常モデルで状態推定
実際のドローン: 正常な制御入力で飛行
→ 残差は小さい（ノイズレベル）
```

#### 故障時の期待動作
```
オブザーバー: 正常モデルで状態推定（故障を知らない）
実際のドローン: 異常な制御入力で飛行（ピッチハードオーバー等）
→ 推定位置と実際位置に乖離 → 残差増大
```

### 4.3 問題の原因分析

#### 仮説1: 故障の影響が不十分
```python
# 現在の故障注入実装
if fault_mode:
    control_values = fault_handler.modify_control_values(i, control_values)
```

**可能な問題点**
- 故障注入による制御値変更が実際のドローン動作に十分な影響を与えていない
- ピッチハードオーバーの強度が不足している
- 故障継続時間が短すぎる

#### 仮説2: **制御フローの順序問題（主要原因）**
```python
# 現在の問題のある順序
1. オブザーバー更新 ← 前回のRC値（正常値）を使用
2. 制御値計算
3. 故障注入による制御値修正 ← 異常制御値生成
4. ドローンに送信 ← 異常制御値でドローン飛行
```

**問題の本質**
- オブザーバーが故障注入**前**の正常制御入力で推定
- 実際のドローンは故障注入**後**の異常制御入力で飛行
- しかし、オブザーバーは異常制御入力も受け取って推定してしまう
- 結果として残差が出ない

#### 仮説3: 時間スケールの不一致
```python
self.fault_counter_threshold = 3  # 0.3秒で故障判定
```

**考慮すべき点**
- 故障の影響がドローンの位置に現れるまでの遅延
- オブザーバーの収束時間
- 制御ループの周期（100ms）との関係

### 4.4 設計上の重要な考慮事項

#### オブザーバーの役割
```
✅ 正しい設計: オブザーバーは常に正常モデルで推定
❌ 間違った設計: オブザーバーが故障を考慮して推定
```

#### 残差生成メカニズム
```
実際の故障 → ドローン位置のずれ → MOCAPで観測 → オブザーバーとの差 → 残差
```

## 5. 改善提案

### 5.1 故障注入の強化

#### より強力な故障パターン
```python
def inject_stronger_fault(self, drone_index):
    """より強力な故障注入"""
    if drone_index == 0:  # 1号機に故障注入
        # ピッチを最大値に固定
        return [0, 20, 0, 0]  # [roll, pitch, throttle, yaw]
```

#### 故障継続時間の延長
```python
self.fault_injection_duration = 5.0  # 5秒間継続
```

### 5.2 制御フロー順序の修正（最重要）

#### 正しい制御フロー
```python
# 修正後の正しい順序
1. 制御値計算
2. オブザーバー更新 ← 正常制御値で推定
3. 故障注入による制御値修正 ← 異常制御値生成
4. ドローンに送信 ← 異常制御値でドローン飛行
```

#### 実装修正案
```python
# オブザーバーには故障注入前の正常制御値を渡す
original_control_values = control_values.copy()
obs_results = droneObserver.update(drone_pos, [original_control_values])

# その後で故障注入
if fault_mode:
    control_values = fault_handler.modify_control_values(i, control_values)
```

### 5.3 残差解析の強化

#### 残差の方向性分析
```python
def analyze_residual_pattern(self):
    """残差のパターン分析"""
    if len(self.residual_history) >= 5:
        recent_residuals = self.residual_history[-5:]
        # X, Y, Z方向の残差傾向を分析
        residual_trend = np.mean(recent_residuals, axis=0)
        return residual_trend
```

#### 複数指標による故障検知
```python
def multi_metric_fault_detection(self):
    """複数指標による故障検知"""
    # 残差ノルム
    residual_norm = np.linalg.norm(self.residual)
    
    # 残差の変化率
    if len(self.residual_history) >= 2:
        residual_rate = (self.residual_history[-1] - 
                        self.residual_history[-2]) / self.dt
    
    # 複合判定
    fault_score = residual_norm + 0.5 * abs(residual_rate)
    return fault_score > self.fault_threshold
```

## 6. デバッグ支援機能

### 6.1 詳細ログ出力

#### 残差履歴の可視化
```python
def log_residual_analysis(self):
    """残差解析結果のログ出力"""
    if self.residual is not None:
        residual_norm = np.linalg.norm(self.residual)
        print(f"🔍 残差解析: ノルム={residual_norm:.4f}, "
              f"しきい値={self.fault_threshold}, "
              f"カウンター={self.fault_counter}/{self.fault_counter_threshold}")
```

#### 状態推定精度の監視
```python
def monitor_estimation_accuracy(self, true_position):
    """推定精度の監視"""
    estimation_error = np.linalg.norm(self.x_hat[:3] - true_position)
    print(f"📊 推定精度: 誤差={estimation_error:.4f}m")
```

### 6.2 実験的検証手順

#### ステップ1: 基本動作確認
1. 正常時の残差レベル確認
2. オブザーバー収束性の確認
3. 制御入力の影響度確認

#### ステップ2: 故障注入テスト
1. 段階的な故障強度での残差変化確認
2. 故障継続時間と残差の関係確認
3. 信頼度計算の妥当性確認

#### ステップ3: パラメータ最適化
1. しきい値の調整
2. 故障検知感度の最適化
3. リーダー交代タイミングの調整

## 6.3 リーダー交代時の目標位置継承技術詳細 (2025-08-07 新規)

### A. 問題の根本原因と解決策

#### 原因 1: リーダー交代検出ロジックの不具合
```python
# 修正前（問題あり）
if new_leader_idx != previous_leader_idx:
    # 目標位置継承処理
    previous_leader_idx = new_leader_idx  # 毎回上書きで検出不可

# 修正後（正しい実装）
# グローバル変数で管理
previous_leader_idx = 0  # 初期化は1回のみ

# 制御ループ内
if new_leader_idx != previous_leader_idx:
    # 目標位置継承処理
    old_leader_target = controllers[previous_leader_idx].get_target_position()
    controllers[new_leader_idx].set_target_position(*old_leader_target)
    # 重要: 検出後に更新
    previous_leader_idx = new_leader_idx
```

#### 原因 2: フォーメーション制御のスコープ問題
```python
# 修正前（問題あり）
# リーダー交代処理でcurrent_leader_idxを更新
# しかしフォーメーション制御が古い値を参照

# 修正後（正しい実装）
# フォーメーション制御直前で最新リーダーインデックスを取得
current_leader_idx = droneObserver.get_leader_index()
print(f"🔄 フォーメーション制御: 最新リーダーインデックス = {current_leader_idx}")
```

#### 原因 3: 信頼度しきい値の継続条件
```python
# 修正前（問題あり）
self.fault_counter_threshold = 3  # 0.3秒の継続が必要

# 修正後（即座反応）
self.fault_counter_threshold = 1  # 即座に検知
```

### B. 信頼度ベース故障検知の最適化

#### 残差アルゴリズムのパラメータ調整
```python
# observer.py - 最適化されたパラメータ
self.fault_threshold = 0.4  # 残差闾値（適切な感度）
self.fault_counter_threshold = 1  # 即座検知（応答性重視）

# 信頼度計算アルゴリズム
if self.fault_counter >= self.fault_counter_threshold:
    excess_time = self.fault_counter - self.fault_counter_threshold
    # 指数関数的減衰: 10回（1秒）で信頼度0.1まで低下
    decay_factor = np.exp(-excess_time / 10.0)
    trust = max(0.1, decay_factor)
```

#### リーダー選出アルゴリズム
```python
# leader_switching.py - 信頼度ベース選出
class LeaderSelector:
    def __init__(self):
        self.emergency_threshold = 0.25  # 緊急交代闾値
    
    def update(self, trust_metrics):
        # 最高信頼度ドローンをリーダーに選出
        new_leader_idx = np.argmax(trust_metrics)
        return new_leader_idx
```

### C. フォーメーション再構成アルゴリズム

#### リーダー交代後のフォロワー位置調整
```python
# position_control.py - 動的オフセット調整
def _update_formation_offset(self):
    if self.is_leader:
        self.formation_offset = [0.0, 0.0, 0.0]  # リーダーは基準位置
    else:
        # フォロワーのオフセット（ドローンIDに基づく）
        if self.drone_id == 0:  # 1号機がフォロワーの場合
            self.formation_offset = [0.0, 0.0, 1.0]  # Z軸方向に+1mオフセット
        elif self.drone_id == 1:  # 2号機がフォロワーの場合
            self.formation_offset = [0.0, 0.0, 1.0]  # Z軸方向に+1mオフセット
```

### D. デバッグログの充実

#### リーダー交代イベントの可視化
```python
# mocap_for_2tellos.py - 詳細デバッグログ
if new_leader_idx != previous_leader_idx:
    print(f"🔄 リーダー交代検出: {previous_leader_idx+1}号機 → {new_leader_idx+1}号機")
    
    old_leader_target = controllers[previous_leader_idx].get_target_position()
    print(f"📍 旧リーダー目標位置: {old_leader_target}")
    
    controllers[new_leader_idx].set_target_position(*old_leader_target)
    print(f"✅ 新リーダー目標位置継承完了: {old_leader_target}")
    
    previous_leader_idx = new_leader_idx
```

### E. 実機検証結果

#### 成功した動作フロー
1. **初期状態**: 1号機（リーダー）[0,1,0]、2号機（フォロワー）[0,1,1]
2. **故障注入**: Fキーで1号機の信頼度を0.6未満に低下
3. **リーダー交代**: 2号機が新リーダーに選出
4. **目標位置継承**: 2号機が[0,1,0]を継承
5. **フォーメーション再構成**: 1号機が[0,1,1]（2号機からZ軸+1m）に移動

#### CSVログでの確認ポイント
- `control_log.csv`: role列でリーダー/フォロワーの切替え確認
- `observer_log.csv`: trust列で信頼度変化確認
- target_x, target_y, target_z列で目標位置継承確認

## 7. 今後の発展方向

### 7.1 高度な故障検知手法

#### 統計的変化点検出
- CUSUM (Cumulative Sum) アルゴリズム
- 逐次確率比検定 (SPRT)

#### 機械学習ベース故障検知
- 異常検知アルゴリズム（One-Class SVM等）
- 時系列異常検知（LSTM Autoencoder等）

### 7.2 マルチモーダル故障検知

#### センサーフュージョン
- MOCAP + IMU + 制御入力の統合
- 複数情報源による故障検知精度向上

#### 故障タイプ分類
- アクチュエータ故障
- センサー故障
- 通信故障

## 8. まとめ

本オブザーバーシステムは、カルマンフィルタベースの状態推定と残差解析による故障検知を組み合わせた設計です。現在確認されている「故障注入時の残差問題」については、故障の影響度、オブザーバーパラメータ、時間スケール等の複合的な要因が考えられます。

**重要な設計原則**
- オブザーバーは常に正常モデルで推定
- 実際の故障による位置ずれが残差として現れる
- 段階的な故障検知による信頼度計算

今後は提案した改善策を段階的に実装し、実機検証を通じてシステムの信頼性向上を図る予定です。
