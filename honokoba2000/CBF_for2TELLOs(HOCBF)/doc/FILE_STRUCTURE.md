# CBF_for2TELLOs プロジェクト構成

**最終更新**: 2025-10-10  
**バージョン**: CBF専用版（故障対応機能除去後）

---

## 📁 ディレクトリ構成

```
CBF_for2TELLOs/
├── doc/                          # ドキュメント
│   ├── CBF_IMPLEMENTATION.md     # CBF実装詳細ドキュメント
│   ├── FILE_STRUCTURE.md         # 本ファイル（プロジェクト構成）
│   ├── IMPROVEMENT_REPORT.md     # 改善報告書
│   ├── cbf_spec.md              # CBF仕様書
│   └── implementation_guide.md   # 実装ガイド
│
├── src2/                         # CBF専用実装（メインコード）
│   ├── __pycache__/             # Pythonキャッシュ
│   ├── src2_results/            # 実行結果（CSVログ）
│   ├── NatNetClient.py          # OptiTrack MOCAP通信ライブラリ
│   ├── cbf_filter.py            # CBFフィルタ実装
│   ├── csv_logger.py            # CSVログ記録
│   ├── custom_tello.py          # Telloドローン制御拡張
│   ├── keyboard_control.py      # キーボード入力処理
│   ├── mocap_for_2tellos.py     # メインプログラム
│   ├── mocap_stream.py          # MOCAP通信処理
│   └── position_control.py      # 位置制御 + CBF統合
│
├── src2_phase2_backup/          # 故障対応機能（バックアップ）
│   ├── observer.py              # 故障検知・信頼度計算
│   ├── leader_switching.py      # リーダー交代ロジック
│   ├── fault_handler.py         # 故障注入機能
│   ├── csv_postprocess.py       # データ後処理
│   ├── mocap_stream_test.py     # MOCAPテスト
│   └── tello_double_control_test.py  # ドローンテスト
│
├── sim/                          # シミュレーション（別プロジェクト）
└── ref/                          # 参考資料
```

---

## 🎯 src2/ (CBF専用実装) - ファイル詳細

### メインプログラム
- **`mocap_for_2tellos.py`** (92KB)
  - 2台のTelloドローンを制御するメインプログラム
  - MOCAP位置データ取得、キーボード入力処理、制御スレッド管理
  - Tモード（CBF+フォーメーション制御）、Mモード（手動）の切り替え
  - CSVログ記録機能

### 制御・CBF関連
- **`position_control.py`** (38KB)
  - `PositionController`クラス：位置制御アルゴリズム
  - CBFフィルタ統合（`cbf_enabled`フラグで有効化）
  - フォーメーション制御機能（リーダー・フォロワー）
  - PD制御による目標位置追従

- **`cbf_filter.py`** (6.7KB)
  - `enforce_cbf()`関数：CBF制約を強制する半空間投影
  - `CBFParams`クラス：CBFパラメータ（K1, K2, α1, α2, α3, Δ）
  - test座標系での障害物回避処理

### ドローン・通信関連
- **`custom_tello.py`** (24KB)
  - `CustomTello`クラス：Telloドローンの拡張制御
  - `TelloManager`クラス：複数ドローンの管理
  - 状態取得、RC制御コマンド送信

- **`NatNetClient.py`** (29KB)
  - OptiTrack MOCAPシステムとの通信
  - リジッドボディの位置・姿勢データ取得

- **`mocap_stream.py`** (23KB)
  - MOCAP通信の上位レイヤー
  - ドローン位置データの管理・取得

### ユーティリティ
- **`keyboard_control.py`** (14KB)
  - キーボード入力の検出・処理
  - Pygameを使用したリアルタイム入力

- **`csv_logger.py`** (19KB)
  - 制御データ、Tello状態データのCSV記録
  - デバッグログ機能

---

## 🔄 src2_phase2_backup/ (故障対応機能)

### 2025-10-10にバックアップ移動した理由
CBF機能のみに絞り込むため、Phase 2のリーダー交代・故障対応機能を分離しました。

### バックアップファイル
- **`observer.py`** (20KB)
  - `DroneObserver`クラス：ドローン状態推定・故障検知
  - 残差ベースの信頼度計算

- **`leader_switching.py`** (5.2KB)
  - `LeaderSelector`クラス：信頼度に基づくリーダー選定
  - 閾値ベースの交代判定

- **`fault_handler.py`** (3.5KB)
  - 故障注入機能（テスト用）
  - 制御値の意図的な修正

- **`csv_postprocess.py`** (7.4KB)
  - ログデータの後処理スクリプト

- **テストファイル**
  - `mocap_stream_test.py`, `tello_double_control_test.py`

---

## 🚀 実行方法

### 1. 事前準備
```bash
cd /home/initial/honokoba2000/CBF_for2TELLOs/src2
```

### 2. プログラム起動
```bash
python mocap_for_2tellos.py
```

### 3. キーボード操作
| キー | 機能 |
|------|------|
| `Q` | 離陸 |
| `E` | 着陸 |
| `T` | Tモード（CBF有効化 + autoモード） |
| `M` | Mモード（CBF無効化 + manualモード） |
| `SPACE` | プログラム終了 |
| `矢印キー` | 手動操縦（Mモード時） |

---

## ⚙️ CBF設定

### CBFパラメータ（cbf_filter.py）
```python
K1 = 0.02       # バリア関数ゲイン1
K2 = 0.02       # バリア関数ゲイン2
alpha1 = 0.1    # 拡張クラスK関数パラメータ1
alpha2 = 0.1    # 拡張クラスK関数パラメータ2
alpha3 = 0.1    # 拡張クラスK関数パラメータ3
Delta = 0.9     # 安全マージン（障害物からの最小距離）
```

### 障害物位置（position_control.py）
```python
cbf_obstacle_test = (0, -0.6)  # test座標系での障害物位置
```

### フォーメーション設定（position_control.py）
```python
# 2号機がフォロワーの場合のオフセット
self.formation_offset = [-0.8, 0.0, 0.0]  # X軸方向に-0.8m
# → リーダー[0.9,1,1] + [-0.8,0,0] = フォロワー[0.1,1,1]
```

---

## 📊 座標系

### proj座標系（MOCAP座標系）
- **X軸**: 前後方向
- **Y軸**: 高度
- **Z軸**: 左右方向

### test座標系（CBF計算用）
- **x_test = Z_proj**: 左右方向
- **y_test = X_proj**: 前後方向

---

## 🎯 デフォルト目標位置

```python
# mocap_for_2tellos.py
default_target_positions = [
    [0.9, 1, 1],  # ドローン1（リーダー）: proj(X,Y,Z)
    [0.1, 1, 1]   # ドローン2（フォロワー）: proj(X,Y,Z)
]
```

**Tモード時の自動計算**:
- リーダー（1号機）: 固定位置 [0.9, 1, 1]
- フォロワー（2号機）: [0.9, 1, 1] + [-0.8, 0, 0] = [0.1, 1, 1]

---

## 📝 ログファイル

実行すると `src2_results/` に以下が生成されます：

- `control_log_YYYYMMDD_HHMMSS.csv` - 制御データログ
- `observer_log_YYYYMMDD_HHMMSS.csv` - オブザーバーデータログ（使用されていません）
- `tello_status_log_YYYYMMDD_HHMMSS.csv` - Tello状態データログ

---

## 🔧 今回の主要変更（2025-10-10）

### 1. ファイル整理
- 故障対応ファイル6個を `src2_phase2_backup/` に移動
- CBF機能のみに絞り込み

### 2. コード削除（mocap_for_2tellos.py）
- `observer`, `leader_switching`, `fault_handler` のインポート削除
- オブザーバー・リーダーセレクタのグローバル変数削除
- F/Rキーの故障注入・解除処理削除
- リーダー交代判定処理削除
- 故障制御値修正処理削除

### 3. フォーメーション設定変更（position_control.py）
- 2号機のオフセットを `[0.0, 0.0, 1.0]` → `[-0.8, 0.0, 0.0]` に変更
- これにより2号機の目標位置が [0.1, 1, 1] に正しく設定される

### 4. 動作確認
- ✅ CBFが正常動作（障害物回避を確認）
- ✅ フォーメーション制御が正常動作
- ✅ 目標位置設定が正しく反映

---

## 📚 関連ドキュメント

- **CBF実装詳細**: `doc/CBF_IMPLEMENTATION.md`
- **CBF仕様**: `doc/cbf_spec.md`
- **改善報告**: `doc/IMPROVEMENT_REPORT.md`
- **実装ガイド**: `doc/implementation_guide.md`

---

## 🔜 次のステップ

### CBF専用版を使う場合
```bash
cd /home/initial/honokoba2000/CBF_for2TELLOs/src2
python mocap_for_2tellos.py
```

### 故障対応機能を復元する場合
```bash
# バックアップから復元
cp src2_phase2_backup/*.py src2/
# ただし、mocap_for_2tellos.pyの統合が必要
```

---

**作成者**: Windsurf Cascade AI  
**プロジェクト**: 2台のTelloドローンによるCBF制約付き衝突回避制御  
**実験環境**: OptiTrack MOCAPシステム、Tello EDU × 2台
