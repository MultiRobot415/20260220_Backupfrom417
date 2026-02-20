# ドローンオブザーバーシミュレーション

このディレクトリには、ドローン状態推定と故障検出アルゴリズムの検証用シミュレーション環境が含まれています。

## 目的

このシミュレーション環境は以下の目的で作成されました：

1. オブザーバーアルゴリズムの設計と検証
2. 故障検出ロジックのテスト
3. 実機テスト前のパラメータチューニング
4. マルチUAVシステムにおける信頼度ベースのリーダー選定アルゴリズムの検証

## ディレクトリ構造

```
sim/
├── __init__.py                # Pythonパッケージ初期化
├── simulation_runner.py       # メインシミュレーション実行スクリプト
├── models/                    # モデル定義
│   ├── __init__.py
│   ├── observer.py            # オブザーバークラス
│   └── drone.py               # ドローン動力学モデル
├── utils/                     # ユーティリティ
│   ├── __init__.py
│   ├── visualization.py       # 可視化・CSVログ記録
│   └── metrics.py             # 性能評価指標
└── README.md                  # このファイル
```

## 主要クラス

### `models/observer.py`

- `StateObserver`: 単一ドローンのためのカルマンフィルタベースの状態推定
- `DroneObserver`: 複数ドローンの状態推定と故障検出を管理

### `models/drone.py`

- `DroneModel`: ドローンの動力学と故障をシミュレーション
- `MultiDroneSimulator`: 複数ドローンのシミュレーション管理

### `utils/visualization.py`

- `SimulationVisualizer`: シミュレーション結果の可視化
- `CSVLogger`: 実機と互換性のあるCSVログ記録

### `utils/metrics.py`

オブザーバーと故障検出アルゴリズムの性能評価指標を計算するユーティリティ

## 使い方

### 基本的な実行方法

```bash
python simulation_runner.py
```

これにより、デフォルト設定（シミュレーション時間30秒、時間ステップ0.1秒、故障なし）でシミュレーションが実行されます。

### オプション

```bash
python simulation_runner.py --help
```

主なオプション：

- `--time <秒>`: シミュレーション時間の設定
- `--dt <秒>`: 時間ステップの設定
- `--fault <1または2>`: 故障を発生させるドローンの指定（1=1号機、2=2号機）
- `--fault-time <秒>`: 故障発生時間の設定
- `--fault-type <タイプ>`: 故障の種類を指定
- `--no-viz`: 可視化を無効化

### 故障シミュレーションの例

```bash
# 1号機が15秒後に位置ドリフト故障を起こすシミュレーション
python simulation_runner.py --fault 1 --fault-time 15 --fault-type position_drift

# 2号機が10秒後に姿勢バイアス故障を起こすシミュレーション
python simulation_runner.py --fault 2 --fault-time 10 --fault-type attitude_bias

# センサーノイズ増加の故障シミュレーション
python simulation_runner.py --fault 1 --fault-type sensor_noise
```

## 出力

シミュレーションは以下の出力を生成します：

1. CSVログファイル: `sim_results/sim_log_<タイムスタンプ>_observer.csv`
   - 実機実験と同じ形式で記録
   - データ分析や実機データとの比較に利用可能

2. 可視化プロット: `sim_results/sim_plot_<タイムスタンプ>.png`
   - ドローンの軌跡
   - オブザーバー残差
   - 信頼度の推移
   - リーダー選定結果

3. コンソール出力:
   - シミュレーション設定の概要
   - 故障発生情報
   - 性能評価指標のサマリー

## 評価指標

- **RMSE (Root Mean Square Error)**: 位置推定の精度
- **MAE (Mean Absolute Error)**: 位置推定の平均絶対誤差
- **故障検出指標**: 精度、適合率、再現率、F1スコア
- **検出遅延**: 故障発生から検出までの遅延
- **信頼度-誤差相関**: 信頼度メトリクスの有効性評価

## 実機データとの連携

このシミュレーションでは実機と同じ形式のCSVログを生成するため、以下が可能です：

1. 実機データを使ったオフライン解析
2. シミュレーションと実機のデータ比較
3. シミュレーションで検証したアルゴリズムの実機への移植

## 注意点

- このシミュレーション環境はPython 3.6以上が必要です
- 依存ライブラリ: numpy, matplotlib
- 可視化にはGUIが必要です（リモート実行時はX11転送などを検討）
