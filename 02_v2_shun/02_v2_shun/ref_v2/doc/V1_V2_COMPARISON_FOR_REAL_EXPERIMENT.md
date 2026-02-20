# v1とv2の実機実装比較

## 概要

本文書では、v1（PID階層制御型SLAF）とv2（オブザーバ型SLAF）の実機実装における相違点を整理し、v2の実機実装に必要な変更点を明確化する。

## 制御則の根本的な違い

### v1: PID階層制御型
```
推定器: 位置・速度・積分項を含む階層型PID
制御器: 推定値を直接フィードバック（状態フィードバック的）
共線回避: 適応λによる調整
```

### v2: オブザーバ型分離構造
```
推定器: Luenbergerオブザーバ（位置・速度）
制御器: 推定位置と測定速度を用いたPD制御
共線回避: 適応λによる調整（v1と同様）
```

## 実装上の主要な違い

### 1. 推定器の構造

#### v1の推定器（PID階層型）
```matlab
% v1: sim_PID_v1/system_dynamics.m相当
dot_p_hat_i = v_hat_i;
dot_v_hat_i = -k_p*(p_hat_i - p_star_i) - k_v*(v_hat_i - v_i) ...
              - k_i*z_i + dot_v_star_i + xi_i;
dot_z_i = p_hat_i - p_star_i;
```

**特徴**:
- 3状態（位置、速度、積分項）
- 制御項（-k_p, -k_v, -k_i）を推定器に含む
- 目標速度の微分（dot_v_star_i）が必要

#### v2の推定器（Luenbergerオブザーバ型）
```matlab
% v2: sim_v2/system_dynamics.m相当
dot_p_hat_i = v_i + xi_i;              % 測定速度を使用
dot_v_hat_i = u_i + K_obs*(v_i - v_hat_i);  % オブザーバゲイン
```

**特徴**:
- 2状態（位置、速度）のみ
- 制御入力u_iをそのまま使用
- 速度測定v_iを直接利用
- 推定器と制御器が完全分離

### 2. 制御器の構造

#### v1の制御器
```matlab
% v1では推定器に制御項が含まれるため、実質的には：
u_i = 0;  % 制御入力は存在しない（推定器内で完結）
```

**特徴**:
- 推定器が制御も兼ねる
- 外部に制御入力が存在しない

#### v2の制御器
```matlab
% v2: define_trajectory_simple.m, calculate_control_logic.m
u_i = ddot_p_star_i ...                    % フィードフォワード
      - K_p*(p_hat_i - p_star_i) ...       % 位置フィードバック
      - K_v*(v_i - dot_p_star_i) ...       % 速度フィードバック（測定値）
      + psi_i;                             % 共線回避項
```

**特徴**:
- 明示的な制御入力
- 目標加速度をフィードフォワード
- 速度フィードバックに推定値ではなく測定値v_iを使用

### 3. 補正項ξの計算

#### 両バージョン共通
```matlab
% calculate_weight_matrices.m で計算される重み行列を使用
xi_i = -D_ii * e_hat_p_i;  % 線形化近似
```

**v1との互換性**:
- 重み行列の計算ロジックは同一
- `calculate_weight_matrices.m`を共有可能

### 4. 共線回避項ψの計算

#### v1の実装
```matlab
% v1: calculate_control_logic.m
psi_i = -tau_i * (sign(e_bar_p_i) - lambda_i);
lambda_i = -gamma * Delta_v_i;  % 飽和処理あり
Delta_v_i = e_bar_v_i - e_hat_v_i;
```

#### v2の実装（ほぼ同一）
```matlab
% v2: calculate_control_logic.m
psi_i = -tau_i * (sign(p_hat_i - p_star_i) - lambda_i);
lambda_i = -gamma * Delta_v_i;  % 飽和処理あり
Delta_v_i = e_bar_v_i - e_hat_v_i;
```

**差異**:
- v1: `sign(e_bar_p_i)` （追跡誤差）
- v2: `sign(p_hat_i - p_star_i)` （推定位置と目標位置の差）
- 実質的には同じ（e_bar_p_i = p_i - p_star_iで、p_i ≈ p_hat_iの場合）

### 5. 状態方程式

#### v1: 3状態システム
```matlab
% X = [p_f; v_f; p_hat_f; v_hat_f; z_f]
% 状態数: 5 × 3 × num_followers
function dX = system_dynamics(t, X, config)
    % p_f, v_f: 実際の位置・速度
    % p_hat_f, v_hat_f: 推定位置・速度
    % z_f: 積分項
end
```

#### v2: 2状態システム
```matlab
% X = [p_f; v_f; p_hat_f; v_hat_f]
% 状態数: 4 × 3 × num_followers
function dX = system_dynamics(t, X, config)
    % p_f, v_f: 実際の位置・速度
    % p_hat_f, v_hat_f: 推定位置・速度（積分項なし）
end
```

## 実機実装への影響

### v1実機実装の特徴（既存）

#### 測定要求
1. **速度測定**: IMUまたはオプティカルフローによる速度測定
2. **方位測定**: カメラによる相対方位測定
3. **目標軌道**: 事前計画または通信による目標位置・速度・加速度

#### 計算負荷
- 状態数が多い（積分項を含む）
- 推定器内で制御計算も実行

#### 実装ファイル
```
v1/sim_PID_v1/
├── main_simple.m              # メインループ
├── system_dynamics.m          # 状態方程式（推定器+実機ダイナミクス）
├── calculate_control_logic.m  # ξ, ψ, τの計算
├── calculate_weight_matrices.m # 重み行列計算
└── define_trajectory_simple.m  # 軌道定義
```

### v2実機実装の要求

#### 測定要求（v1と同じ）
1. **速度測定**: 必須（v_i）
2. **方位測定**: 必須（g_ij, g_ik）
3. **目標軌道**: 必須（p_star_i, dot_p_star_i, ddot_p_star_i）

#### 計算負荷（v1より軽減）
- 状態数が少ない（積分項なし）
- 推定器と制御器が分離（並列実装可能）

#### 必要な新規実装
```
v2/sim_v2_real_experiment/（新規作成予定）
├── main_real_experiment.m     # 実機用メインループ
├── system_dynamics.m          # 状態方程式（新規）
├── calculate_control_logic.m  # ξ, ψ, τの計算（v1から一部変更）
├── calculate_weight_matrices.m # 重み行列計算（v1と共通）
└── define_trajectory_simple.m  # 軌道定義（現存）
```

## 実機実装の移植戦略

### Phase 1: シミュレーションコードの整備（完了）
- [x] v2シミュレーションの完成
- [x] 理論文書の整備（v4_observer.tex）
- [x] ゲイン調整指針の確立

### Phase 2: v1実機コードの理解
- [ ] v1実機コードの動作確認
- [ ] v1で使用しているセンサI/Oの確認
- [ ] v1の通信プロトコルの確認

### Phase 3: v2実機コードの作成
1. **system_dynamics.mの書き換え**
   - 積分項z_iの削除
   - 推定器をLuenbergerオブザーバ型に変更
   - 制御入力u_iの明示化

2. **calculate_control_logic.mの修正**
   - 制御入力u_iの計算を追加
   - ψ項の引数を調整（e_bar_p_i → p_hat_i - p_star_i）

3. **main_real_experiment.mの作成**
   - v1のmain_simple.mをベースに作成
   - 状態ベクトルのサイズ調整（5状態→4状態）
   - センサI/Oインターフェース（v1と共通）

### Phase 4: 実機検証
- [ ] 静止状態でのオブザーバ動作確認
- [ ] 直線軌道での追従性能評価
- [ ] 共線回避動作の確認
- [ ] v1との性能比較

## 実機実装の優先度付け

### 高優先度（実装必須）
1. **system_dynamics.m**: オブザーバ型への書き換え
2. **calculate_control_logic.m**: 制御入力計算の追加
3. **main_real_experiment.m**: 実機用メインループ

### 中優先度（既存コード活用）
4. **calculate_weight_matrices.m**: v1と共通（変更不要）
5. **define_trajectory_simple.m**: 現存（軽微な調整のみ）

### 低優先度（後回し可能）
6. **plot_results.m**: 可視化（v1のコード流用）
7. **animate_3d_trajectory.m**: アニメーション（v1のコード流用）

## v1とv2の理論的優位性比較

### v1の優位性
- **実績**: 既に実機検証済み
- **安定性**: PID制御の枯れた技術
- **実装**: 推定器と制御器が一体（実装がシンプル）

### v2の優位性
- **理論的洗練性**: カスケード分離構造、厳密な証明
- **計算効率**: 状態数が少ない（5状態→4状態）
- **初期誤差対応**: 初期推定誤差から指数収束
- **オクルージョン対応**: 理論的に明確な挙動
- **時変加速度対応**: 任意の滑らかな軌道に追従可能
- **分離原理**: 推定器と制御器を独立に設計・調整可能

### 実機実装での期待

#### 予想される改善点
1. **収束速度**: オブザーバゲインK_obsによる高速化
2. **初期誤差**: 初期推定誤差からの確実な収束
3. **ロバスト性**: オクルージョンからの復帰性能

#### 予想される課題
1. **速度測定ノイズ**: v2は速度測定v_iに直接依存
2. **ゲイン調整**: 新たなゲイン（K_obs, K_p, K_v）の調整
3. **未検証**: 実機での動作確認が未実施

## 実機実装チェックリスト

### v1実機コードの確認（Phase 2）
- [ ] センサI/Oインターフェースの仕様確認
- [ ] 通信プロトコルの確認（Leader-Follower間）
- [ ] 制御周期の確認
- [ ] 実行時間の計測（計算負荷）

### v2実機コードの作成（Phase 3）
- [ ] system_dynamics.mの書き換え
- [ ] calculate_control_logic.mの修正
- [ ] main_real_experiment.mの作成
- [ ] ゲイン設定ファイルの作成

### 実機検証（Phase 4）
- [ ] 静止状態テスト（推定誤差の収束）
- [ ] 直線軌道テスト（追従性能）
- [ ] 共線回避テスト（脱出動作）
- [ ] オクルージョンテスト（復帰性能）
- [ ] v1との比較テスト

## 結論

v2（オブザーバ型）は理論的洗練性と計算効率の面でv1より優れている。実機実装への移植は、v1の既存コードを大きく活用しつつ、推定器と制御器の構造を変更することで実現可能である。特に、状態数の削減（5状態→4状態）により、計算負荷の軽減が期待できる。

実機検証により、v2の理論的優位性が実用面でも確認できれば、今後のマルチエージェントシステム制御の標準的手法となりうる。
