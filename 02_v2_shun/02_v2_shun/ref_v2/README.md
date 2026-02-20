# v2: オブザーバ型二次系SLAF制御

## 概要

v2は、速度測定可能な二次系マルチエージェントシステムに対する**オブザーバ型SLAF (Simultaneous Localization And Formation) 制御則**の実装です。本制御則は、推定器と制御器が完全に分離したカスケード構造を持ち、理論的洗練性と実装効率の両面で優れています。

## v1との主な違い

| 項目 | v1（PID階層制御型） | v2（オブザーバ型） |
|------|---------------------|-------------------|
| 推定器 | PID階層型（3状態） | Luenbergerオブザーバ（2状態） |
| 制御器 | 推定器に統合 | 明示的な制御入力 |
| 状態数 | 5状態/エージェント | 4状態/エージェント |
| 理論 | 経験的設計 | 厳密な収束証明 |
| 初期誤差 | 理論的扱い不明 | 指数収束を証明 |
| オクルージョン | 対応可能 | 理論的に明確な挙動 |

## ディレクトリ構成

```
v2/
├── README.md                    # 本ファイル
├── v4_observer.tex              # 理論文書（厳密な証明）
├── sim_v2/                      # シミュレーションコード
│   ├── main_simple.m            # メインスクリプト
│   ├── system_dynamics.m        # 状態方程式
│   ├── calculate_control_logic.m # 制御ロジック（ξ, ψ, u計算）
│   ├── calculate_weight_matrices.m # 重み行列計算（v1と共通）
│   ├── define_trajectory_simple.m  # 軌道定義・ゲイン設定
│   └── plot_results.m           # 結果可視化
└── sim_v2_real_experiment/      # 実機実装用（作成予定）
```

## 制御則の構造

### 推定器（Luenbergerオブザーバ型）
```matlab
% 位置推定
dot_p_hat_i = v_i + xi_i;

% 速度推定
dot_v_hat_i = u_i + K_obs*(v_i - v_hat_i);
```

**特徴**:
- 速度測定値`v_i`を直接利用
- 補正項`xi_i`による幾何学的補正
- オブザーバゲイン`K_obs`による収束速度調整

### 制御器（PD制御+フィードフォワード）
```matlab
% 制御入力
u_i = ddot_p_star_i ...                % フィードフォワード
      - K_p*(p_hat_i - p_star_i) ...   % 位置フィードバック
      - K_v*(v_i - dot_p_star_i) ...   # 速度フィードバック
      + psi_i;                         % 共線回避項
```

**特徴**:
- 目標加速度の完全なキャンセル（任意の時変加速度に対応）
- 推定位置と測定速度を用いたフィードバック
- 共線回避項`psi_i`による局所化不可能状態の回避

### カスケード構造
```
推定器系（完全独立）
    ↓ 一方向結合
制御器系（推定誤差に依存）
```

**分離原理**: 推定器と制御器の収束性を独立に証明可能

## 理論的特徴

### 仮定
1. **初期推定誤差の有界性**: `||p_hat_i(0) - p_i(0)|| ≤ ε_p`, `||v_hat_i(0) - v_i(0)|| ≤ ε_v`
2. **目標軌道の滑らかさ**: `p_star(t)`が2階微分可能かつ有界
3. **局所化可能性**: 目標配置において重み行列が正則

### 主要な結果
- **補題**: 推定誤差は指数的に零に収束（初期誤差が非零でも）
- **定理**: 追跡誤差は漸近的に零に収束（LaSalle-Yoshizawa定理）
- **重要な性質**:
  - 目標加速度は任意（零/非零、定数/時変すべて対応可能）
  - オクルージョン時は推定誤差が一定値に保たれ、解除後に指数収束
  - 相対測定（重み行列）が位置推定の長期的精度に不可欠

## ゲイン設定

### 理論で要求される条件
```matlab
K_obs > 0     % オブザーバゲイン
K_p > 0       % 位置制御ゲイン
K_v > 0       % 速度制御ゲイン
γ ∈ (0,1]     % λ調整ゲイン
λ_max ∈ (0,1) % λ飽和限界
```

### 推奨値（`define_trajectory_simple.m`）
```matlab
cfg.K_obs = 10;      % オブザーバゲイン
cfg.K_p = 10;        % 位置制御ゲイン（ω_n^2, ω_n=2-5 rad/s推奨）
cfg.K_v = 10;        # 速度制御ゲイン（2ζω_n, ζ=0.7-1.5推奨）
cfg.gamma = 1.0;     % λ調整ゲイン
cfg.lambda_max = 0.99; % λ飽和限界
cfg.w_xi = 5.0;      % ξ項のゲイン（実装上の調整）
cfg.w_psi = 1.0;     % ψ項のゲイン（実装上の調整）
```

### 調整指針
- **K_obs**: 速度推定誤差の収束速度（大きいほど速いが振動的）
- **K_p, K_v**: 標準的な2次系PD制御として調整
  - 固有振動数: `ω_n = sqrt(K_p)`
  - 減衰比: `ζ = K_v / (2*sqrt(K_p))`
- **γ, λ_max**: 共線回避項の振動抑制（詳細は`PSI_OSCILLATION_ANALYSIS.md`参照）

## 実行方法

### シミュレーション実行
```matlab
cd('c:\MATLAB\my_algorithm\v2\sim_v2')
main_simple
```

### 軌道タイプの変更
`define_trajectory_simple.m`の以下を編集：
```matlab
cfg.trajectory.type = 'constant';      % 一定加速度
% cfg.trajectory.type = 'time_varying'; % 正弦波加速度
% cfg.trajectory.type = 'circular';     % 円軌道
```

### 初期推定誤差の設定
```matlab
% 実際の初期位置
cfg.agent_actual_positions = {
    [-4; 3; -1];         % p3 (フォロワー3)
    [-1.5; -2.5; -4.4];  % p4 (フォロワー4)
    [-2; 0; -4]          % p5 (フォロワー5)
};

% 初期推定位置（手動指定）
cfg.agent_estimated_positions = {
    [-1; 3; -0];         % p3推定位置（実際と異なる）
    [-1; -2; -4];        % p4推定位置
    [-2; 0; -3]          % p5推定位置
};

% 初期速度推定誤差
cfg.initial_velocity_error = 0.2;  % [m/s]（0で誤差なし）
```

## 検証内容

### シミュレーションで確認済み
- [x] 初期推定誤差からの指数収束
- [x] 時変加速度軌道への追従
- [x] 共線状態からの脱出
- [x] 推定器と制御器の分離動作

### 今後の検証予定
- [ ] オクルージョンシナリオの実装
- [ ] 複数の軌道タイプでの性能比較
- [ ] ゲイン感度解析
- [ ] 実機実装と検証

## 主要な関連文書

### 理論文書
- **v4_observer.tex**: 厳密な収束性証明（本ディレクトリ）
- **my4thsemi.tex**: v1の理論文書（参考用、`../v1/`）

### 実装関連文書（`../02_windsurf/`）
- **V1_V2_COMPARISON_FOR_REAL_EXPERIMENT.md**: v1とv2の実装比較、実機移植戦略
- **PSI_OSCILLATION_ANALYSIS.md**: 共線回避項の振動解析と対策（★重要）
- **OCCLUSION_BEHAVIOR_ANALYSIS.md**: オクルージョン時の挙動解析
- **INITIAL_VELOCITY_ERROR_ANALYSIS.md**: 初期速度誤差の影響解析
- **RIGOROUS_THEORETICAL_ANALYSIS.md**: 理論的背景の詳細解析
- **DOCUMENT_PRIORITY_FOR_REAL_EXPERIMENT.md**: 実機実装時のドキュメント優先度

詳細は`../02_windsurf/V1_V2_COMPARISON_FOR_REAL_EXPERIMENT.md`を参照。

## トラブルシューティング

### Q: 推定誤差が収束しない
A: 以下を確認：
1. `K_obs`が正の値か
2. 重み行列`D_ii`が正定値か（局所化可能性）
3. `w_xi`が適切な値か（推奨: 1.0-10.0）

### Q: 共線回避項が振動的
A: `PSI_OSCILLATION_ANALYSIS.md`を参照し、以下を調整：
1. `lambda_max`を増加（0.7→0.9）
2. `gamma`を増加（0.5→1.0）
3. 飽和関数`sat_delta`の実装を検討

### Q: 制御入力が大きすぎる
A: 以下を調整：
1. `K_p`, `K_v`を減少
2. `w_psi`を減少（共線回避項のゲイン）
3. 目標軌道の加速度を緩やかに

## 参考文献

1. Xu Fang, Lihua Xie and Dimos V. Dimarogonas, "Simultaneous distributed localization and formation tracking control via matrix-weighted position constraints," *Automatica*, Vol.175, 112188, 2025.
2. Nicholas Fischer, Rushikesh Kamalapurkar and Warren E. Dixon, "LaSalle-Yoshizawa corollaries for nonsmooth systems," *IEEE Transactions on Automatic Control*, Vol. 73, pp.2333-2338, 2013.

## 連絡先・貢献

**開発者**: 滑川研究室  
**最終更新**: 2025年12月11日

---

**重要**: v2は理論的に洗練されていますが、実機検証はまだ完了していません。実機実装時は、まず`V1_V2_COMPARISON_FOR_REAL_EXPERIMENT.md`と`PSI_OSCILLATION_ANALYSIS.md`を熟読してください。
