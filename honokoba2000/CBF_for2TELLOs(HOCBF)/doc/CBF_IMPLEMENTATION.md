# CBF（Control Barrier Function）衝突回避実装ドキュメント

## 概要

本ドキュメントは、2機のTelloドローンにおける障害物回避機能（CBF）の実装詳細を記録します。
故障対応やリーダー交代などの他機能は含まず、**CBF衝突回避機能のみ**に焦点を当てています。

---

## 1. CBFの基本概念

### 1.1 目的
- 静的障害物を回避しながら目標位置に到達する
- 最小限の制御入力修正で安全性を確保
- フォーメーション制御との統合

### 1.2 座標系仕様
実機の座標系（proj座標系）：
- **X軸**: 前後方向（+X=前進、-X=後退）
- **Y軸**: 上下方向（+Y=上昇、-Y=下降）
- **Z軸**: 左右方向（+Z=右、-Z=左）

CBF計算用のtest座標系：
- **x_test**: proj座標系のZ軸に対応（左右方向）
- **y_test**: proj座標系のX軸に対応（前後方向）
- **z_test**: proj座標系のY軸に対応（上下方向）※CBFでは使用しない

### 1.3 障害物設定
- **障害物位置（test座標系）**: (x_o, y_o) = (0, -0.6)
- **安全距離**: Δ = 0.9 [m]

---

## 2. CBFパラメータ

### 2.1 デフォルトパラメータ（cbf_filter.py）
```python
K1 = 0.009          # X軸方向のゲイン
K2 = 0.009          # Y軸方向のゲイン
alpha1 = 1.0        # CBF制約パラメータ1（現在値）
alpha2 = 1.0        # CBF制約パラメータ2（現在値）
alpha3 = 1.0        # CBF制約パラメータ3
Delta = 0.9         # 障害物からの安全距離 [m]
u_min = -30.0       # 制御入力下限
u_max = 30.0        # 制御入力上限
```

### 2.2 ダイナミクスモデル
```
状態: [x, y, ẋ, ẏ]

ẋ = [0  0  1  0] [x]   [0   0 ] [u_x]
ẏ = [0  0  0  1] [y] + [0   0 ] [u_y]
ẍ   [0  0  0  0] [ẋ]   [K1  0 ]
ÿ   [0  0  0  0] [ẏ]   [0   K2]
```

### 2.3 CBF制約式
```
h(x) = (x-x_o)² + (y-y_o)² - Δ²

制約条件:
2ẋ² + 2(α₂+α₃)(x-x_o)ẋ + 2ẏ² + 2(α₂+α₃)(y-y_o)ẏ 
+ 2K₁(x-x_o)u_x + 2K₂(y-y_o)u_y 
+ α₁α₂[(x-x_o)² + (y-y_o)² - Δ²] ≥ 0
```

---

## 3. 実装ファイル構成

### 3.1 CBF関連ファイル（故障対応機能は除く）
```
src2/
├── cbf_filter.py              # CBFフィルタ実装（コアモジュール）
├── position_control.py        # 位置制御 + CBF統合
├── mocap_for_2tellos.py       # メインプログラム（CBF有効化制御）
└── test.md                    # CBFパラメータ定義・制約式
```

### 3.2 除外ファイル（故障対応関連）
以下のファイルはCBF機能とは独立した故障対応機能です：
- `observer.py` - 故障検知・信頼度計算
- `leader_switching.py` - リーダー交代ロジック
- `fault_handler.py` - 故障注入機能

---

## 4. CBFフィルタの実装（cbf_filter.py）

### 4.1 CBFParams クラス
```python
class CBFParams:
    def __init__(self, K1=0.009, K2=0.009, alpha1=1.0, alpha2=1.0, 
                 alpha3=1.0, Delta=0.9, u_min=-30.0, u_max=30.0):
        self.K1 = K1
        self.K2 = K2
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.alpha3 = alpha3
        self.Delta = Delta
        self.u_min = u_min
        self.u_max = u_max
```

### 4.2 enforce_cbf 関数（メイン処理）
```python
def enforce_cbf(u_nom: Tuple[float, float],
                state_test: Tuple[float, float, float, float],
                obstacle_test: Tuple[float, float],
                params: CBFParams = None) -> Tuple[np.ndarray, Dict]:
    """
    CBF制約を適用して制御入力を修正
    
    入力:
      - u_nom: (u_x, u_y) test座標系の名目制御入力
      - state_test: (x, y, ẋ, ẏ) test座標系の状態
      - obstacle_test: (x_o, y_o) test座標系の障害物位置
      - params: CBFParams パラメータ
      
    出力:
      - u_safe: 安全な制御入力
      - info: {
          'feasible_nom': bool,      # 名目入力が制約を満たすか
          'projected': bool,          # 投影が成功したか
          'fell_back': bool,          # フォールバックしたか
          'margin_before': float,     # 修正前のマージン
          'margin_after': float       # 修正後のマージン
        }
    """
```

### 4.3 投影アルゴリズム
1. **Box制約への投影**: u_min ≤ u ≤ u_max
2. **半空間への投影**: a^T u + b ≥ 0
3. **ライン探索**: Box制約と半空間制約の両立
4. **方向探索**: a方向への移動
5. **フォールバック**: 制約違反を許容

### 4.4 デバッグメッセージ
```
🟢 CBF: 名目入力が制約満足 margin=XXX
🔴 CBF: 名目入力が制約違反 margin=XXX, 修正が必要
🟡 CBF: 半空間投影成功 margin=XXX
🟠 CBF: 半空間投影失敗 margin=XXX, ライン探索を試行
🔴 CBF: フォールバック - 制約違反 margin=XXX (INFEASIBLE)
```

---

## 5. 位置制御への統合（position_control.py）

### 5.1 PositionController クラスのCBF関連属性
```python
class PositionController:
    def __init__(self):
        # CBF関連
        self.cbf_enabled = False  # Tモード時にTrueに設定
        self.cbf_params = CBFParams()
        self.cbf_obstacle_test = (0, -0.6)  # 障害物位置（test座標系）
        
        # 速度推定用
        self._last_pos = None
        self._last_time = None
```

### 5.2 calculate_control メソッドでのCBF適用
```python
def calculate_control(self, current_position, current_yaw=None, quaternion=None):
    # 1. 基本的な位置制御を計算
    control_x = int(error_x * self.gain_x * 100)
    control_y = int(error_y * self.gain_y * 100)
    control_z = int(error_z * self.gain_z * 100)
    control_yaw = int(error_yaw * self.gain_yaw)
    
    # 2. 制限値を適用
    control_x = self._clamp(control_x, -self.max_speed_x, self.max_speed_x)
    control_y = self._clamp(control_y, -self.max_speed_y, self.max_speed_y)
    control_z = self._clamp(control_z, -self.max_speed_z, self.max_speed_z)
    
    # 3. CBFフィルタ適用（Tモード時のみ）
    if self.cbf_enabled:
        # 速度推定
        vx, vz = 0.0, 0.0
        if self._last_pos is not None and self._last_time is not None:
            dt = max(1e-6, now - self._last_time)
            vx = (current_position[0] - self._last_pos[0]) / dt
            vz = (current_position[2] - self._last_pos[2]) / dt
        
        # proj座標系 → test座標系への変換
        x_test = current_position[2]  # Z_proj → x_test
        y_test = current_position[0]  # X_proj → y_test
        xdot_test = vz
        ydot_test = vx
        
        # 名目入力（test座標系）
        u_nom_test = (float(control_x), float(control_z))
        
        # CBFフィルタ適用
        u_safe, info = enforce_cbf(
            u_nom=u_nom_test,
            state_test=(x_test, y_test, xdot_test, ydot_test),
            obstacle_test=self.cbf_obstacle_test,
            params=self.cbf_params,
        )
        
        # 修正後の制御値を適用
        control_x = int(u_safe[0])
        control_z = int(u_safe[1])
    
    return [control_z, control_x, control_y, control_yaw]
```

### 5.3 座標変換の詳細
```python
# proj座標系 → test座標系
x_test = position_proj[2]  # Z軸（左右）
y_test = position_proj[0]  # X軸（前後）

xdot_test = velocity_proj_z
ydot_test = velocity_proj_x

# test座標系の制御入力 → proj座標系
u_x_test → control_x (前後制御)
u_y_test → control_z (左右制御)
```

---

## 6. メインプログラムでの制御（mocap_for_2tellos.py）

### 6.1 CBF有効化（Tモード切替時）
```python
def process_keyboard_input():
    if "t" in pressed_keys:
        if control_mode == "manual":
            control_mode = "auto"
            
            # CBFフィルタをTモード時に有効化
            print(f"🚀 CBF有効化開始: Tモード切替時")
            for i, ctrl in enumerate(controllers):
                ctrl.cbf_enabled = True
                print(f"✅ ドローン{i+1}: CBF有効化完了")
```

### 6.2 CBF無効化（Mモード切替時）
```python
    if "m" in pressed_keys:
        if control_mode == "auto":
            control_mode = "manual"
            
            # CBFフィルタをMモード時に無効化
            print(f"🚫 CBF無効化開始: Mモード切替時")
            for i, ctrl in enumerate(controllers):
                ctrl.cbf_enabled = False
                print(f"✅ ドローン{i+1}: CBF無効化完了")
```

### 6.3 制御ループ内の処理順序
```python
# 制御スレッド内（control_drones_thread関数）
for i, position_tuple in enumerate(positions):
    # 1. 基本的な位置制御を計算（CBFはここで適用される）
    control_values = controllers[i].calculate_control(
        position_data, quaternion=quaternion
    )
    
    # 2. フォーメーション制御入力を加算
    if formation_control_enabled:
        formation_input = controllers[i].calculate_formation_control(
            position_data, CONTROL_INTERVAL
        )
        control_values[0] += formation_gain * formation_input[1]
        control_values[1] += formation_gain * formation_input[0]
        control_values[2] += formation_gain * formation_input[2]
    
    # 3. 最終的な制御値を制限
    for idx in range(3):
        control_values[idx] = max(-20, min(20, control_values[idx]))
```

---

## 7. 現在の問題点と解決策

### 7.1 問題: CBFとフォーメーション制御の統合順序
**現象**:
- 障害物に近いドローン（2号機）がCBFで回避しようとする
- しかしフォーメーション制御が「リーダーに追従しろ」と引っ張る
- 結果：障害物の境界上でCBFとフォーメーション制御が拮抗し、動けなくなる

**原因**:
```python
# 現在の処理順序
control_values = calculate_control()  # ← CBFがここで適用される
control_values += formation_input     # ← フォーメーション制御が無修正で加算
```

CBFは基本制御（目標位置への追従）にのみ適用され、その後フォーメーション制御入力が無修正で加算されるため、フォーメーション制御入力が障害物回避を打ち消す。

### 7.2 解決策案

#### 解決策1: CBFを最終制御値に適用（推奨）
```python
# 1. 基本制御を計算
control_values = calculate_control()

# 2. フォーメーション制御入力を加算
control_values += formation_input

# 3. 合成された制御値にCBFを適用（★ここで適用）
if cbf_enabled:
    # 合成制御値をCBFで修正
    control_values_safe = apply_cbf_to_combined(control_values, position, velocity)
    control_values = control_values_safe
```

#### 解決策2: CBFを個別に適用
```python
# 1. 基本制御を計算
basic_control = calculate_control()

# 2. 基本制御にCBFを適用
basic_control_safe = apply_cbf(basic_control)

# 3. フォーメーション制御を計算
formation_control = calculate_formation_control()

# 4. フォーメーション制御にもCBFを適用
formation_control_safe = apply_cbf(formation_control)

# 5. 両方を合成
control_values = basic_control_safe + formation_control_safe
```

#### 解決策3: フォーメーション制御のゲインを動的調整
```python
# CBF発火時にフォーメーションゲインを下げる
if cbf_fired:
    formation_gain = 0.1  # 通常0.3から低減
else:
    formation_gain = 0.3
```

---

## 8. CSVログ記録項目（CBF関連）

### 8.1 control_log.csv に記録されるCBF情報
```
cbf_fire_flag     - CBFが発火したか（True/False）
rc_nom_x          - 名目制御入力X（test座標系）
rc_nom_z          - 名目制御入力Z（test座標系）
rc_safe_x         - CBF修正後の制御入力X
rc_safe_z         - CBF修正後の制御入力Z
rc_diff_norm      - 修正量のノルム
qp_status         - QP最適化ステータス
active_constraint_id - アクティブな制約ID
h_x               - h(x)の値（障害物からの距離関数）
HOh_x             - CBF制約式の左辺の値
```

### 8.2 ログの読み方
- `cbf_fire_flag=True`: CBFが発火し、制御入力が修正された
- `cbf_fire_flag=False`: CBFが発火せず、名目入力をそのまま使用
- `rc_diff_norm`: 修正量が大きいほど、CBFの影響が大きい
- `h_x < 0`: 障害物の安全領域内（危険）
- `h_x > 0`: 障害物の安全領域外（安全）

---

## 9. 使用方法

### 9.1 プログラム実行
```bash
cd /home/initial/honokoba2000/CBF_for2TELLOs/src2
python mocap_for_2tellos.py
```

### 9.2 操作手順
1. **Q**: 離陸
2. **T**: Tモード（CBF自動有効化）
3. ドローンが目標位置に向かって移動（障害物を自動回避）
4. **E**: 着陸
5. **SPACE**: 正常終了

### 9.3 パラメータ調整
`test.md`または`cbf_filter.py`でパラメータを変更：
```python
K1 = 0.009      # ダイナミクスゲイン
K2 = 0.009
alpha1 = 1.0    # CBF制約パラメータ
alpha2 = 1.0
alpha3 = 1.0
Delta = 0.9     # 安全距離
```

---

## 10. まとめ

### 10.1 実装済み機能
- ✅ CBFフィルタの実装（cbf_filter.py）
- ✅ 位置制御への統合（position_control.py）
- ✅ Tモードでの自動有効化
- ✅ CSVログ記録
- ✅ 座標変換（proj ↔ test）

### 10.2 未解決の課題
- ⚠️ フォーメーション制御との統合順序の問題
- ⚠️ 障害物回避とフォーメーション維持の優先度調整

### 10.3 次のステップ
1. フォーメーション制御後にCBFを適用する実装に変更
2. 両ドローンが協調して障害物を回避する挙動を確認
3. パラメータチューニング（α1, α2, K1, K2）

---

## 参考情報

### ファイルパス
- CBFフィルタ: `/home/initial/honokoba2000/CBF_for2TELLOs/src2/cbf_filter.py`
- 位置制御: `/home/initial/honokoba2000/CBF_for2TELLOs/src2/position_control.py`
- メイン: `/home/initial/honokoba2000/CBF_for2TELLOs/src2/mocap_for_2tellos.py`
- パラメータ定義: `/home/initial/honokoba2000/CBF_for2TELLOs/test.md`

### 関連ドキュメント
- `doc/cbf_spec.md`: CBF仕様書
- `test.md`: CBFパラメータと制約式定義
- `README.md`: プロジェクト全体の概要（※故障対応機能も含む）

---

**作成日**: 2025-10-10  
**バージョン**: 1.0  
**対象コード**: src2/（CBF機能のみ）
