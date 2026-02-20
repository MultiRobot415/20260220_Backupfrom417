# MOCAP for TELLO - 初めての方向けガイド

## システム概要

このシステムは、OptiTrackモーションキャプチャシステム（MOCAP）を使用して、Telloドローンの位置を正確に把握し、制御するためのものです。モーションキャプチャデータを基にドローンの現在位置を取得し、キーボード操作またはプログラムによる自動制御でドローンを飛行させることができます。

## システム構成

### ハードウェア構成
1. **Telloドローン** - DJI社製の小型ドローン
2. **OptiTrackモーションキャプチャシステム** - 赤外線カメラを使用した位置追跡システム
3. **制御用PC** - モーションキャプチャデータの受信とドローン制御を行うコンピュータ

### ソフトウェア構成
本システムは以下の主要なプログラムファイルで構成されています：

#### メインプログラム
- **src/tello_mocap_control.py** - システムのメインプログラム。ドローン制御とMOCAPデータ処理を統合

#### 補助モジュール
- **src/NatNetClient.py** - OptiTrackのデータ受信用クライアント
- **src/custom_tello.py** - Telloドローン制御用カスタムクラス
- **src/KeyPressModule.py** - キーボード入力検出モジュール
- **src/mocap_stream.py** - MOCAPデータストリーム処理モジュール

#### 動作確認用プログラム
`programs_for_check` フォルダ内に、各コンポーネントの動作確認用プログラムがあります：
- **test_mocap_only.py** - MOCAPとの通信確認のみを行う
- **test_tello_only.py** - Telloドローンとの通信確認のみを行う
- **mocap_continuous.py** - MOCAPとの継続的な通信を行う

## 使い方

### 基本的な実行方法

1. **環境準備**:
   - OptiTrackシステムが正常に動作していることを確認
   - Telloドローンの電源を入れ、Wi-Fi接続を確立
   - 制御用PCがモーションキャプチャシステムのネットワークに接続されていることを確認

2. **プログラム実行**:
   ```bash
   cd /home/initial/MOCAP_forTELLO
   python src/tello_mocap_control.py
   ```

3. **キー操作**:
   - **Q**: 離陸
   - **E**: 着陸
   - **W/S**: 上昇/下降
   - **A/D**: 左右回転
   - **矢印キー上/下**: 前進/後退
   - **矢印キー左/右**: 左右移動
   - **ESC**: 緊急停止
   - **スペース**: プログラム正常終了

### 動作確認

初めて使用する場合は、各コンポーネントが正常に動作するか確認することをお勧めします：

1. **MOCAPシステムのみの確認**:
   ```bash
   python programs_for_check/test_mocap_only.py
   ```

2. **Telloドローンのみの確認**:
   ```bash
   python programs_for_check/test_tello_only.py
   ```

3. **MOCAPデータのみでテスト実行**:
   ```bash
   python src/tello_mocap_control.py --mocap-only
   ```

## トラブルシューティング

### よくある問題と解決策

1. **ドローンが検出されない**:
   - Telloドローンの電源が入っているか確認
   - PCがTelloのWi-Fiネットワークに接続されているか確認
   - `test_tello_only.py` を実行して接続テスト

2. **MOCAPデータが受信されない**:
   - OptiTrackシステムが起動しているか確認
   - ネットワーク設定が正しいか確認（IPアドレスなど）
   - `test_mocap_only.py` を実行して接続テスト

3. **ドローンが安定しない**:
   - バッテリー残量を確認
   - モーションキャプチャマーカーが正しく取り付けられているか確認
   - リジッドボディの設定を確認

## 詳細情報

より詳細な情報については、以下のドキュメントを参照してください：

- **technical_specification.md** - システムの技術仕様
- **user_manual.md** - 詳細な使用方法
- **mocap_for_control.md** - モーションキャプチャを使用した制御の詳細
- **mocapdatastream.md** - MOCAPデータストリームの処理方法
