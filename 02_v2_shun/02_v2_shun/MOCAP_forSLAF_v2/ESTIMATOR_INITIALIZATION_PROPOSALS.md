# 初期推定誤差リセット機構の実装案

## 理論的要件

### Assumption（ref/pid_slaf_japanese_proof.tex）
```
Assumption (初期条件):
  p̂_i(0) = p_i(0)  （初期推定位置 = 初期実位置）
  v̂_i(0) = v_i(0)  （初期推定速度 = 初期実速度）
  z_i(0) = 0        （積分項の初期値はゼロ）
```

**理論的根拠**:
- この仮定により、完全収束定理（Theorem 3.1）が保証される
- 初期推定誤差がゼロであれば、推定器は正しく動作し、追跡誤差も収束する
- 初期推定誤差が大きいと、制御開始時に大きな制御入力が発生し、機体が不安定になる

### 現状の問題点

**現在の実装**（プロトタイプに合わせて）:
```python
# slaf_pid_controller.py
def __init__(self, ...):
    self.p_hat = np.zeros(2)  # ゼロ初期化
    self.v_hat = np.zeros(2)
    self.z_integral = np.zeros(2)
```

**問題**:
- Tキー押下時、推定位置 `p_hat = [0, 0]` だが、実位置は `p_actual = [x, z]`（MOCAPから取得）
- 初期推定誤差 `ê(0) = p̂(0) - p(0) = [0, 0] - [x, z] = [-x, -z]` ≠ 0
- 理論的仮定（Assumption）を満たしていない

---

## 実装案の比較

### 案1: Tキー押下時にMOCAP位置で自動初期化（推奨）

#### 概要
Tキー（SLAF制御開始）押下時に、MOCAPから取得した現在位置で推定器を自動的に初期化する。

#### 実装方法
```python
# mocap_slaf_main.py - process_keyboard_input関数内
if "t" in pressed_keys:
    # ... 既存の処理 ...
    
    # フォロワーの推定器をMOCAP位置で初期化
    mocap_positions = {}
    for follower_id in [3, 4]:
        tello_id = follower_to_drone_map[follower_id]
        rigid_id = RIGID_BODY_IDS[tello_id]
        
        try:
            mocap_pos = ms.get_rigid_body_position(rigid_id)
            if mocap_pos:
                x = mocap_pos.get('x', 0.0)
                z = mocap_pos.get('z', 0.0)
                mocap_positions[follower_id] = np.array([x, z])
        except:
            print(f"警告: フォロワー{follower_id}のMOCAP位置取得失敗")
    
    # 推定器を初期化（Assumption: p̂_i(0) = p_i(0)）
    slaf_manager.initialize_followers(mocap_positions)
    print("推定器を初期化しました（初期推定誤差零）")
```

#### メリット
✅ **理論的に正しい**: Assumptionを満たす
✅ **自動化**: ユーザー操作が不要
✅ **安全**: 制御開始時の急激な動作を防ぐ
✅ **実装が簡単**: Tキー処理に数行追加するだけ

#### デメリット
⚠️ **MOCAP依存**: MOCAPデータが取得できない場合、初期化失敗
⚠️ **タイミング**: Tキー押下時のMOCAP位置が正確でない可能性（遅延、欠損）

#### 対策
- MOCAP位置取得失敗時は警告を表示し、ゼロ初期化のままにする
- または、初期化が成功するまでSLAF制御を開始しない

---

### 案2: 専用の初期化キー（例: Iキー）を追加

#### 概要
SLAF制御開始（Tキー）とは別に、推定器初期化専用のキー（例: Iキー）を設ける。

#### 実装方法
```python
# mocap_slaf_main.py - process_keyboard_input関数内
if "i" in pressed_keys:
    if not hasattr(process_keyboard_input, 'i_last_time') or \
       current_time - process_keyboard_input.i_last_time > 0.5:
        process_keyboard_input.i_last_time = current_time
        
        if control_mode == "slaf" and slaf_manager:
            print("=" * 60)
            print("Iキー検出 - 推定器を初期化")
            
            # MOCAP位置で推定器を初期化
            mocap_positions = {}
            for follower_id in [3, 4]:
                tello_id = follower_to_drone_map[follower_id]
                rigid_id = RIGID_BODY_IDS[tello_id]
                
                try:
                    mocap_pos = ms.get_rigid_body_position(rigid_id)
                    if mocap_pos:
                        x = mocap_pos.get('x', 0.0)
                        z = mocap_pos.get('z', 0.0)
                        mocap_positions[follower_id] = np.array([x, z])
                except:
                    pass
            
            slaf_manager.initialize_followers(mocap_positions)
            print("推定器初期化完了（初期推定誤差零）")
            print("=" * 60)
```

#### メリット
✅ **柔軟性**: 任意のタイミングで初期化可能
✅ **デバッグ**: 実験中に推定器をリセットできる
✅ **明示的**: 初期化が意図的に行われることが明確

#### デメリット
⚠️ **操作が増える**: ユーザーが2つのキー（T + I）を押す必要がある
⚠️ **忘れやすい**: Tキーのみ押して、Iキーを忘れる可能性

#### 推奨される使い方
1. Tキー押下 → SLAF制御開始（推定器はゼロ初期化のまま）
2. Iキー押下 → 推定器をMOCAP位置で初期化
3. 制御が安定する

---

### 案3: 自動初期化 + 手動リセット機能（ハイブリッド）

#### 概要
案1（Tキー押下時の自動初期化）と案2（専用キーでのリセット）を組み合わせる。

#### 実装方法
```python
# Tキー押下時: 自動初期化
if "t" in pressed_keys:
    # ... 既存の処理 ...
    slaf_manager.initialize_followers(mocap_positions)  # 自動初期化

# Iキー押下時: 手動リセット
if "i" in pressed_keys:
    # ... 案2と同じ ...
    slaf_manager.initialize_followers(mocap_positions)  # 手動リセット
```

#### メリット
✅ **自動 + 手動**: 通常は自動、必要に応じて手動リセット可能
✅ **柔軟性**: 最も柔軟な運用が可能
✅ **安全**: 制御開始時は自動的に初期化される

#### デメリット
⚠️ **複雑性**: 実装が若干複雑になる
⚠️ **混乱**: 2つの初期化方法があることで混乱する可能性

---

### 案4: 継続的な推定器補正（理論に反する - 非推奨）

#### 概要
制御中に、推定位置と実位置の誤差が大きい場合、推定器を自動的に補正する。

#### 実装例（理論に反するため非推奨）
```python
# slaf_pid_controller.py - update_estimator内
def update_estimator(self, xi):
    # 推定誤差が大きい場合、推定器を補正
    estimation_error = self.p_hat - self.p_actual
    if np.linalg.norm(estimation_error) > THRESHOLD:
        self.p_hat = self.p_actual  # 強制的にリセット
```

#### デメリット
❌ **理論に反する**: Assumptionは「初期」条件であり、継続的な補正は理論対象外
❌ **収束性不明**: 理論的保証がなくなる
❌ **振動の可能性**: 推定器と実測定の間で振動する可能性

#### 結論
**この案は推奨しません**。理論的根拠がなく、予期しない挙動を引き起こす可能性があります。

---

## 推奨実装: 案1（Tキー押下時の自動初期化）

### 理由
1. **理論的に正しい**: Assumptionを満たす
2. **ユーザーフレンドリー**: 追加操作が不要
3. **安全**: 制御開始時の急激な動作を防ぐ
4. **実装が簡単**: 最小限の変更で実現可能

### 実装手順

#### Step 1: `slaf_pid_controller.py`に初期化メソッドが既にある
```python
def initialize_state(self, p_initial, v_initial=None):
    """初期状態を設定（Assumption: 初期推定誤差零）"""
    self.p_actual = np.array(p_initial, dtype=float)
    self.v_actual = np.array(v_initial) if v_initial else np.zeros(2)
    
    # Assumption: p̂_i(0) = p_i(0), v̂_i(0) = v_i(0)
    self.p_hat = self.p_actual.copy()
    self.v_hat = self.v_actual.copy()
    self.z_integral = np.zeros(2)
```

#### Step 2: `SLAFSystemManager`に初期化メソッドを追加
```python
# slaf_pid_controller.py
class SLAFSystemManager:
    def initialize_followers(self, follower_positions):
        """
        フォロワーの推定器を初期化（Assumption: 初期推定誤差零）
        
        Args:
            follower_positions: {follower_id: np.array([x, z])}
        """
        for follower_id, controller in self.follower_controllers.items():
            p_initial = follower_positions.get(follower_id, [0.0, 0.0])
            controller.initialize_state(p_initial)
        logger.info("全フォロワーの推定器を初期化しました（初期推定誤差零）")
```

#### Step 3: `mocap_slaf_main.py`のTキー処理に追加
```python
if "t" in pressed_keys:
    # ... 既存の処理 ...
    
    # フォロワーの推定器をMOCAP位置で初期化
    print("推定器を初期化中...")
    mocap_positions = {}
    for follower_id in [3, 4]:
        tello_id = follower_to_drone_map[follower_id]
        rigid_id = RIGID_BODY_IDS[tello_id]
        
        try:
            mocap_pos = ms.get_rigid_body_position(rigid_id)
            if mocap_pos:
                x = mocap_pos.get('x', 0.0)
                z = mocap_pos.get('z', 0.0)
                mocap_positions[follower_id] = np.array([x, z])
                print(f"  フォロワー{follower_id}: p_initial = [{x:.3f}, {z:.3f}]")
        except Exception as e:
            print(f"  警告: フォロワー{follower_id}のMOCAP位置取得失敗: {e}")
    
    # 推定器を初期化（Assumption: p̂_i(0) = p_i(0)）
    slaf_manager.initialize_followers(mocap_positions)
    print("✅ 推定器初期化完了（初期推定誤差零）")
```

---

## 処理フローの詳細

### タイミング図
```
時刻 t0: ドローン離陸、手動モードでホバリング
         ↓
時刻 t1: ユーザーがTキーを押下
         ↓
         (1) MOCAP位置を取得: p_3(t1), p_4(t1)
         (2) 推定器を初期化: p̂_3(t1) = p_3(t1), p̂_4(t1) = p_4(t1)
         (3) 仮想リーダー目標位置を設定
         (4) SLAF制御モード開始
         ↓
時刻 t1+: SLAF制御ループ実行
         - MOCAPで実位置を観測: p_3(t), p_4(t)
         - 推定器を更新: p̂_3(t), p̂_4(t)
         - 制御入力を計算: u_3(t), u_4(t)
         - ドローンに送信
```

### 初期化失敗時の対応

#### オプション1: 警告表示 + ゼロ初期化のまま継続
```python
if not mocap_positions:
    print("⚠️  警告: MOCAP位置取得失敗 - 推定器はゼロ初期化のまま継続")
    print("    制御が不安定になる可能性があります")
else:
    slaf_manager.initialize_followers(mocap_positions)
```

#### オプション2: SLAF制御を開始しない
```python
if not mocap_positions:
    print("❌ エラー: MOCAP位置取得失敗 - SLAF制御を開始できません")
    print("    MOCAP接続を確認してください")
    return  # SLAF制御を開始しない
else:
    slaf_manager.initialize_followers(mocap_positions)
    control_mode = "slaf"
```

**推奨**: オプション1（警告表示 + 継続）
- 実験中にMOCAPが一時的に切断される可能性があるため
- ユーザーに判断を委ねる

---

## テスト方法

### テスト1: 初期化なしの場合（現状）
```
期待される動作:
- Tキー押下時、p̂_3 = [0, 0], p̂_4 = [0, 0]
- 実位置が p_3 = [0.5, 0.3] の場合、初期推定誤差 = [-0.5, -0.3]
- 制御開始時に大きな制御入力が発生
- 機体が不安定になる可能性

CSVログ確認:
timestamp,x,x_hat,target_x,...
1234.0,0.50,0.00,0.50,...  ← x_hat = 0.0（ゼロ初期化）
```

### テスト2: 初期化ありの場合（推奨実装後）
```
期待される動作:
- Tキー押下時、p̂_3 = p_3, p̂_4 = p_4（MOCAP位置で初期化）
- 初期推定誤差 = 0
- 制御開始時の制御入力は小さい
- 機体が安定してホバリング

CSVログ確認:
timestamp,x,x_hat,target_x,...
1234.0,0.50,0.50,0.50,...  ← x_hat = 0.50（MOCAP位置で初期化）
```

---

## FAQ

### Q1. 初期化はいつ行うべきか？
**A**: Tキー押下時（SLAF制御開始時）に自動的に行うのが推奨です。これにより、理論的仮定（Assumption）を満たし、安全な制御が可能になります。

### Q2. 制御中に推定誤差が大きくなった場合、再初期化すべきか？
**A**: いいえ。理論的には、初期推定誤差がゼロであれば、推定誤差は時間とともに収束します（Theorem 3.1）。制御中の再初期化は理論対象外であり、推奨しません。

### Q3. MOCAPデータが取得できない場合、どうすべきか？
**A**: 警告を表示し、ゼロ初期化のまま継続する（オプション1）か、SLAF制御を開始しない（オプション2）のいずれかを選択します。推奨はオプション1です。

### Q4. プロトタイプ（ゼロ初期化）でも動作していたが？
**A**: プロトタイプは小規模シミュレーションであり、初期位置が原点付近であったため、初期推定誤差が小さかったと考えられます。実機では、初期位置が任意の位置になるため、MOCAP位置での初期化が必要です。

---

## まとめ

### 推奨実装: 案1（Tキー押下時の自動初期化）

**実装内容**:
1. `slaf_pid_controller.py`に`initialize_followers`メソッドを追加（既存の`initialize_state`を活用）
2. `mocap_slaf_main.py`のTキー処理に、MOCAP位置取得 + 推定器初期化を追加

**理論的根拠**:
- Assumption (初期条件): p̂_i(0) = p_i(0), v̂_i(0) = v_i(0), z_i(0) = 0
- 完全収束定理（Theorem 3.1）が保証される

**メリット**:
- 理論的に正しい
- ユーザーフレンドリー
- 安全で安定した制御

**次のステップ**:
1. ✅ 不感帯処理をテストする（既に実装済み）
2. 初期化機構を実装する（この文書の推奨案1）
3. 実機テストで動作確認

---

---

## 実装状況

### ✅ 実装完了: 案1（Tキー押下時の自動初期化）

**実装日**: 2025-11-26

**変更ファイル**:
1. `slaf_pid_controller.py`
   - `initialize_followers`メソッドに理論的根拠を追加
   - 詳細なログ出力を実装
   - 初期化成功/失敗の判定を追加

2. `mocap_slaf_main.py`
   - Tキー処理に推定器初期化コードを追加
   - MOCAP位置取得とエラーハンドリング
   - 詳細な状態表示（成功/失敗）

**処理フロー**:
```
Tキー押下
  ↓
1. 仮想リーダー目標位置設定
  ↓
2. MOCAP位置取得（各フォロワー）
  ↓
3. 推定器初期化（p̂_i(0) = p_i(0)）
  ↓
4. SLAF制御モード開始
```

**競合分析**:
- ✅ 推定器初期化は`p_hat`, `v_hat`, `z_integral`のみを設定
- ✅ 目標位置`p_star`は制御ループ内で独立して設定される
- ✅ 競合は発生しない

**エラーハンドリング**:
- MOCAP位置取得失敗時: 警告表示 + ゼロ初期化のまま継続（オプション1）
- 一部フォロワー失敗時: 取得できたフォロワーのみ初期化

---

## テスト方法（実装後）

### テスト手順

1. **MOCAPシステム起動**
   ```bash
   # MOCAPソフトウェアを起動
   # Rigid Body ID 1, 2 が追跡されていることを確認
   ```

2. **プログラム実行**
   ```bash
   cd /home/initial/01_v1_PID/MOCAP_forSLAF
   python3 mocap_slaf_main.py
   ```

3. **ドローン離陸**
   - Qキー押下（離陸）
   - 手動モードでホバリング確認

4. **SLAF制御開始（Tキー）**
   - Tキー押下
   - 以下のメッセージを確認：
     ```
     推定器初期化中（Assumption: 初期推定誤差零）...
       フォロワー3（ドローン0）: p_initial=[0.xxx, 0.xxx]
       フォロワー4（ドローン1）: p_initial=[0.xxx, 0.xxx]
     ✅ 全フォロワーの推定器初期化完了（初期推定誤差零）
     ```

5. **CSVログ確認**
   ```bash
   # 最新のcontrol_log_*.csvを確認
   # Tキー押下直後の行で、x_hat ≈ x であることを確認
   ```

### 期待される結果

#### 成功時（初期化あり）
```csv
timestamp,x,x_hat,target_x,z,z_hat,target_z,...
1234.0,0.50,0.50,0.50,0.30,0.30,-0.20,...  ← x_hat = x（初期推定誤差零）
1234.1,0.50,0.50,0.50,0.30,0.30,-0.20,...
```

**確認ポイント**:
- Tキー押下直後の`x_hat`が`x`と一致（初期推定誤差零）
- 制御入力が小さい（機体が安定）
- 機体が急激に動かない

#### 失敗時（初期化なし - 以前の挙動）
```csv
timestamp,x,x_hat,target_x,z,z_hat,target_z,...
1234.0,0.50,0.00,0.50,0.30,0.00,-0.20,...  ← x_hat = 0（ゼロ初期化）
1234.1,0.50,0.05,0.50,0.30,0.03,-0.20,...
```

**問題点**:
- Tキー押下直後の`x_hat = 0`（初期推定誤差 = -0.50）
- 大きな制御入力が発生
- 機体が急激に動く可能性

---

## トラブルシューティング（実装後）

### Q1. "⚠️ MOCAP未接続" と表示される
**原因**: MOCAPシステムが起動していない、またはネットワーク未接続
**対策**: 
1. MOCAPソフトウェアを起動
2. ネットワーク接続を確認
3. プログラムを再起動

### Q2. "⚠️ MOCAP位置取得失敗（データなし）" と表示される
**原因**: Rigid Bodyが追跡されていない
**対策**:
1. MOCAPソフトウェアでRigid Body ID 1, 2が追跡されているか確認
2. マーカーが正しく配置されているか確認
3. ドローンがMOCAPの視野内にあるか確認

### Q3. 初期化は成功したが、機体が不安定
**原因**: 初期追跡誤差が大きい（目標位置と実位置が離れている）
**分析**: 
- 初期推定誤差はゼロだが、初期追跡誤差が大きい可能性
- これは理論的には収束するが、過渡応答が大きい

**対策**:
1. ゲイン調整（`k_cp`, `k_cv`を減らす）
2. 目標位置を実位置に近づける（`formation_offsets`を調整）
3. 不感帯を設定（`deadband_x`, `deadband_z`）

### Q4. CSVログで初期推定誤差を確認したい
**確認方法**:
```python
# control_log_*.csv
# Tキー押下直後の行で、以下を計算:
initial_estimation_error_x = x_hat - x
initial_estimation_error_z = z_hat - z

# 期待値: 0.0（初期推定誤差零）
# 許容範囲: ±0.01m（MOCAPの精度による誤差）
```

---

## 更新履歴
- **2025-11-26 (初版)**: 初期版作成、4つの実装案を比較検討
- **2025-11-26 (実装完了)**: 案1の実装完了、テスト方法とトラブルシューティングを追加
