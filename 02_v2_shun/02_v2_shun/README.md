# 01_v1_PID プロジェクト概要

## このリポジトリの目的

このリポジトリは、次の 3 層構造で **同時分散位置推定・隊形追従制御（SLAF）** を扱うための一式をまとめたものです。

- **理論レベル**  
  Automatica 掲載論文
  「Simultaneous distributed localization and formation tracking control via matrix-weighted position constraints」
  およびそれに基づく **PID 階層型 SLAF の日本語証明**。

- **シミュレーションレベル（MATLAB）**  
  SLAF の分散推定・隊形制御アルゴリズムを、シンプルな軌道（直線・台形速度プロファイル）で検証する MATLAB スクリプト群。

- **実機レベル（Python + Tello + OptiTrack MOCAP）**  
  MOCAP からの 3 次元位置情報を用いて、2 機の Tello EDU ドローンを
  - MOCAP ホバリング（T モード）
  - フォーメーション制御
  - オブザーバによる故障検知とリーダー交代
 まで含めて制御・検証するためのコード群。

> 注: 現時点では、`ref/`（理論・MATLAB）と `src/`（Python 実機）の **コードレベルでの自動連結は行っていません**。  
> 制御構造・パラメータの対応づけは概念レベルで整理されており、将来的な連結・比較がしやすいようにフォルダ構成を統一しています。

---

## ディレクトリ構成（トップレベル）

```text
./
├─ ref/          # 理論・証明・MATLAB シミュレーション
└─ src/          # Python 実装（Tello 実機、MOCAP 連携）
```

### `ref/` : 理論・証明・MATLAB シミュレーション

```text
ref/
├─ Simultaneous_distributed_localization_and_formation_tracking_control.md
├─ pid_slaf_japanese_proof.tex
└─ sim_PID_v1/
```

- **Simultaneous_distributed_localization_and_formation_tracking_control.md**  
  Automatica 175 (2025) 掲載論文のテキスト版です。  
  - ウェイト行列に基づく位置拘束
  - bearing / ratio-of-distance / relative position などを混在させた分散位置推定
  - 3D 空間での leader–follower 形成追従制御
 などが記載されています。

- **pid_slaf_japanese_proof.tex**  
  上記論文のアイデアをベースにした **PID 階層型 SLAF** の完全な収束性証明（日本語）。  
  - 上位層: 速度フィードバック付き PID 型推定器
  - 下位層: 推定値と実速度を用いる位置制御器
  - 不連続項 \(\psi_i\) を含む共線回避項（Filippov 解）
  - 階層的な Lyapunov 関数 \(V_{est}, V_{ctrl}\) による安定性解析

- **sim_PID_v1/**  
  SLAF を MATLAB 上で検証するためのシミュレーション環境です。

  主なファイル・文書:
  - `README_SIMPLE.md`  
    直線軌道 + 台形速度プロファイルを用いた **シンプルな追従シミュレーション** の説明。
  - `main_simple.m`  
    直線軌道シナリオのメインスクリプト。
    - 目標軌道生成（`define_trajectory_simple.m`）
    - SLAF ダイナミクス（`system_dynamics.m`）
    - 制御ロジック・ウェイト行列計算（`calculate_control_logic.m`, `calculate_weight_matrices.m`）
    - 結果可視化（`plot_results.m`）
  - 各種設計・検証ドキュメント  
    - `PID_HIERARCHICAL_DESIGN.md`
    - `BUG_ANALYSIS.md`, `BUG_FIX_PROPOSAL.md`, `BUG_FIX_VERIFICATION.md`,
      `FINAL_CORRECTIONS_SUMMARY.md`, `THEORY_IMPLEMENTATION_VERIFICATION.md` など

  これらにより、
  - ウェイト行列に基づく位置拘束の数値的挙動
  - PID 階層型 SLAF の収束性とバグ修正の履歴
  を MATLAB 上で確認できます。

### `src/` : Python 実装（Tello + MOCAP）

```text
src/
├─ TELLO_double_proto/
└─ MOCAP_for2TELLOs/
```

#### `src/TELLO_double_proto/` : 2 機 Tello の基礎制御プロトタイプ

```text
TELLO_double_proto/
├─ src/
│   ├─ tello_double_control.py   # キーボード操作で 2 機の Tello を同時制御
│   ├─ tello_manager.py          # Tello 検出・コマンド送信管理
│   ├─ custom_tello.py           # Tello を Python3 から扱うためのラッパ
│   └─ KeyPressModule.py         # キーボード入力検出
├─ ref/
│   └─ formation_setup_tello2.py # ステーションモード用 Wi-Fi 設定スクリプト
└─ docs/
    ├─ work_log.md
    └─ technical_specification.md
```

- ステーションモードで Wi-Fi ルータに接続された **2 機の Tello** を、単一 PC から **キーボード操作のみで同時制御** するためのプロトタイプです。
- MOCAP には依存せず、Tello SDK コマンド（傾斜角度指令）を周期的に送る **ベースライン実装** として利用できます。
- 詳細は `TELLO_double_proto/README.md` を参照してください。

#### `src/MOCAP_for2TELLOs/` : MOCAP + 2 機 Tello フォーメーション・故障対応

```text
MOCAP_for2TELLOs/
├─ src/          # 基本版（MOCAP ホバリング + 2 機制御の初期実装）
├─ src2/         # フォーメーション + 故障検知 + リーダー交代を実装した発展版
├─ src2_results/ # src2 での実機ログ保存先
├─ sim/          # フォーメーション・リーダー交代ロジックのシミュレーション
├─ doc/          # 実装ガイド・改善レポート・オブザーバ解説
├─ ref/          # ゼミ資料などの参考資料
├─ README.md     # 本サブプロジェクトの詳細 README
├─ Dev_Purpose.md
└─ QandA.md
```

主な役割:

- **`src/`**  
  - `mocap_for_2tellos.py` : MOCAP 連携 + 2 機 Tello 制御の初期メインスクリプト
  - `mocap_stream.py` : NatNetClient を使った MOCAP 受信（位置・姿勢・接続状態管理）
  - `custom_tello.py`, `keyboard_control.py`, `position_control.py` など

- **`src2/`**  
  `src/` をベースに、以下を追加・整理した **メイン開発ライン** です。

  - フォーメーション制御（横一列 1m 間隔）  
    - `position_control.py` に SLAF/合意アルゴリズム相当のロジックを実装
  - オブザーバ + 信頼度計算 + 故障検知  
    - `observer.py` で残差ベースの信頼度推定
  - リーダー交代  
    - `leader_switching.py` で信頼度に基づく動的リーダー切替え
  - 包括的 CSV ログ  
    - `csv_logger.py` で MOCAP 位置、制御入力、フォーメーション状態、信頼度等を記録
  - キーボード操作  
    - Q/E/T/M/Z/G/B/V/N/1/2/0/ESC/SPACE などで
      - 離陸・着陸
      - T モード（自動ホバリング + フォーメーション）
      - リーダー目標位置の前後・左右操作
      - 緊急停止 / 正常終了
      を制御

- **`doc/`**  
  - `implementation_guide.md` : モジュール構成・スレッド構造・実装詳細ガイド
  - `observerguide.md` : 故障検知・信頼度計算・リーダー交代ロジックの詳細
  - `IMPROVEMENT_REPORT.md`, `Modifypolicy.md` など: 開発履歴・設計方針

- **`Dev_Purpose.md` / `QandA.md`**  
  - このサブプロジェクトの開発目的・仕様・フェーズ分割（Phase 1〜3）
  - メインスクリプトをこれ以上肥大化させず、**モジュール分割で機能追加する方針** 等を明記

---

## `ref` と `src` の対応関係と現状

- `ref/Simultaneous_...md` と `pid_slaf_japanese_proof.tex` で定義される **SLAF 理論** と、
  `ref/sim_PID_v1/` の MATLAB コードは、
  - bearing / ratio-of-distance を使った分散位置推定
  - PID 階層構造による推定 + 追従制御
  - 共線状態からの脱出・衝突回避
  を数式レベル・シミュレーションレベルで扱っています。

- `src/MOCAP_for2TELLOs/` の Python 実装は、
  - 実ドローン + MOCAP という制約の中で、
  - 上記 SLAF の考え方（リーダーフォロワー構造、相対位置の維持、オブザーバと信頼度に基づく故障対応）を
    **実装上可能な範囲で再現する試み** です。

- フォルダ間の関係は次のイメージです。

```text
理論 (ref/*.md, *.tex)
   └→ MATLAB 検証 (ref/sim_PID_v1/*.m)
        └→ 実機 Tello + MOCAP 実装 (src/MOCAP_for2TELLOs)
```

> 繰り返しになりますが、現時点では `ref` と `src` は **自動連結されていません**。  
> パラメータ・軌道・評価指標の対応づけは、各自が MATLAB と Python の両側を参照しながら手動で行う前提です。

---

## 典型的な利用フロー

1. **理論の把握**  
   - `ref/Simultaneous_distributed_localization_and_formation_tracking_control.md`
   - `ref/pid_slaf_japanese_proof.tex`

2. **MATLAB での動作確認**  
   - `ref/sim_PID_v1/README_SIMPLE.md` を読み、`main_simple.m` を実行。
   - ウェイト行列、補正項 \(\xi_i\)、共線回避項 \(\psi_i\) の挙動を確認。

3. **Tello + MOCAP 実機検証**  
   - まずは `src/TELLO_double_proto/README.md` を読み、
     キーボードによる 2 機同時制御の挙動を確認（MOCAP なし）。
   - その後、`src/MOCAP_for2TELLOs/README.md`、`Dev_Purpose.md`、`doc/implementation_guide.md` を参照し、
     MOCAP 接続 + フォーメーション制御 + 故障対応を実機で検証。

---

## 実行環境と依存ライブラリ（概要）

### MATLAB / シミュレーション側

- MATLAB（バージョンは `sim_PID_v1` 内のドキュメントを参照）
- 追加ツールボックスは、制御・可視化系が中心

### Python / 実機側

- Python 3 系
- 代表的な依存ライブラリ
  - `numpy`
  - `opencv-python`
  - `pygame`
  - NatNetClient（OptiTrack 用 Python クライアント）
  - ネットワーク系ライブラリ（`netifaces`, `netaddr` など、一部サブプロジェクトで使用）

具体的なインストールコマンドやバージョンは、各サブプロジェクトの README を参照してください。

---

## 新しくこのフォルダを見る人への読み順ガイド

1. **大づかみの全体像**  
   - 本 `README.md`（このファイル）
   - `src/MOCAP_for2TELLOs/README.md`

2. **理論とシミュレーション**  
   - `ref/Simultaneous_distributed_localization_and_formation_tracking_control.md`
   - `ref/pid_slaf_japanese_proof.tex`
   - `ref/sim_PID_v1/README_SIMPLE.md`

3. **実機実装の詳細**  
   - `src/TELLO_double_proto/README.md`
   - `src/MOCAP_for2TELLOs/Dev_Purpose.md`
   - `src/MOCAP_for2TELLOs/doc/implementation_guide.md`
   - `src/MOCAP_for2TELLOs/doc/observerguide.md`

この順番で読むことで、
- 論文レベル → MATLAB シミュレーション → Python 実機実装
という流れで、`ref` と `src` の関係と現在の実装状態を把握しやすくなります。
