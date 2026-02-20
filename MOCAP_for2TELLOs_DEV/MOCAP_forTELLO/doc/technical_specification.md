# Tello MOCAP Control - 技術仕様書

## システム概要

本システムは、OptiTrackモーションキャプチャシステムからの位置フィードバックを利用して、Telloドローンを指定位置でホバリングさせるシステムです。モーションキャプチャデータを受信し、ドローンの現在位置と目標位置の差分に基づいてPID制御を行います。

## ソフトウェアアーキテクチャ

### コンポーネント構成

1. **tello_mocap_control.py**
   - メインプログラム
   - ドローンの制御ロジック
   - キーボード入力処理
   - 位置制御アルゴリズム

2. **NatNetClient.py**
   - OptiTrackのNatNetプロトコルを使用したデータ受信
   - モーションキャプチャデータの解析
   - コールバック関数によるデータ通知

3. **tello_manager.py**
   - Telloドローンの管理
   - ドローンの検出と接続

4. **KeyPressModule.py**
   - Pygameを使用したキーボード入力検出
   - ユーザーインターフェース
   
（以下，チェック用プログラム。\programsforcheckフォルダ内に格納）
1. **test_mocap_only.py**
   - MOCAPとの通信確認のみ行うプログラム
   - 一定時間または一定パケット数で終了
2. **test_tello_only.py**
   - TELLO(1機)との通信確認のみ行うプログラム
3. **mocap_continuous.py**
   - MOCAPとの継続的な通信を行うプログラム
   - test_mocap_only.pyをベースに設計
   - メインスレッドが終了しないよう設計
   - 接続状態の監視機能付き

### 直接用いていないが重要な参考資料
1. （今回pushしていないが）formarion_setup_tello2.pyがWi-Fiルータ接続切替用スクリプト。今は決め打ちで実験室のwifiにつながるように設定している。もし別の部屋のwifiルータを利用したい場合はこのスクリプトを用いた設定変更が必要となる


### ファイル構造

本システムは以下のようなファイル構造で構成されています：

```
MOCAP_forTELLO/
├── doc/                      # ドキュメント
│   ├── user_manual.md        # 取扱説明書
│   ├── technical_specification.md  # 技術仕様書（本ドキュメント）
│   └── mocapdatastream.md    # MOCAPデータストリーム技術詳細
├── src/                      # ソースコード
│   ├── tello_mocap_control.py  # メインプログラム
│   ├── mocap_stream.py       # MOCAPデータ受信モジュール
│   ├── NatNetClient.py       # NatNetクライアント
│   ├── tello_manager.py      # Telloドローン管理クラス
│   ├── KeyPressModule.py     # キーボード入力検出モジュール
│   ├── custom_tello.py       # Telloドローン制御用カスタムクラス
│   └── programsforcheck/     # テスト・検証用プログラム
│       ├── mocap_continuous.py  # MOCAP連続受信テスト
│       ├── test_tello_only.py   # Telloのみのテスト
│       └── NatNetClient.py      # NatNetクライアント（コピー）


このファイル構造において、主要なコンポーネントは以下のように機能します：

1. **src/tello_mocap_control.py**: メインプログラムで、MOCAPデータを受信してTelloドローンを制御します。

2. **src/mocap_stream.py**: MOCAPデータの受信を担当する独立したモジュールで、スレッドセーフなデータアクセスを提供します。

3. **src/programsforcheck/mocap_continuous.py**: MOCAPデータの受信をテストするためのスタンドアロンプログラムです。

### データフロー

```
OptiTrack → NatNetClient → tello_mocap_control → tello_manager → Telloドローン
    ↑                           ↑
    └───────── KeyPressModule ───┘
```

## 技術詳細

### モーションキャプチャデータ受信

- **プロトコル**: NatNet（OptiTrack標準プロトコル）
- **通信方式**: UDP
- **データ形式**: バイナリデータ（リジッドボディの位置・姿勢情報）
- **サンプリングレート**: モーションキャプチャシステムに依存（通常100Hz程度）

### スレッド管理とシャットダウン処理

システムの安定性を向上させるため、以下のスレッド管理とシャットダウン処理を実装しています。

#### スレッド管理

- **共有終了フラグ**: `threading.Event` を使用して、すべてのスレッドに終了シグナルを送信
- **スレッド追跡**: すべての生成されたスレッドをリストで管理
- **非デーモン化**: 長時間実行されるスレッドを非デーモン化し、メインスレッド終了後も安全に終了できるように設計

```python
# スレッド管理の例
stop_event = threading.Event()
threads = []

# スレッド生成と管理
status_thread = threading.Thread(target=check_connection_status)
status_thread.daemon = False  # 非デーモンスレッドに設定
status_thread.start()
threads.append(status_thread)  # スレッドリストに追加
```

#### シャットダウン処理

- **シグナル伝達**: `stop_event.set()` ですべてのスレッドに終了シグナルを送信
- **スレッド終了待機**: `thread.join()` で各スレッドの終了を待機
- **リソース解放**: ソケットやその他のリソースを適切に解放

```python
# シャットダウン処理の例
def graceful_shutdown():
    logging.info("シャットダウン処理を開始します...")
    
    # 終了フラグを設定
    stop_event.set()
    
    # すべてのスレッドの終了を待機
    for i, thread in enumerate(threads):
        logging.info(f"スレッド {i+1}/{len(threads)} の終了を待っています...")
        thread.join()
    
    # ソケットを閉じる
    if mocap_client:
        mocap_client.shutdown()
    
    logging.info("すべてのリソースを解放しました")
```

#### キーボード入力による終了

スペースキーを押すことでプログラムを正常終了させる機能を実装しています。

```python
# キーボード入力処理の例
if kp.getKey("SPACE"):
    logging.warning("キー入力検出: SPACE")
    stop_event.set()  # 終了フラグを設定
```

### ロギング機能

スレッド安全なロギングを実装し、システムの動作状況を記録します。

```python
# ロギング設定
import logging

log_file = f"tello_control_{time.strftime('%Y%m%d_%H%M%S')}.log"
log_format = '%(asctime)s - %(levelname)s - [TELLO] %(message)s'
logging.basicConfig(level=logging.WARNING, format=log_format,
                    handlers=[logging.FileHandler(log_file), logging.StreamHandler()])
```

### ドローン制御

- **制御方式**: PID制御（現在はP制御のみ実装）
- **制御周期**: 約20Hz（INTERVAL = 0.05秒）
- **制御変数**:
  - 前後方向速度（-100〜100）
  - 左右方向速度（-100〜100）
  - 上下方向速度（-100〜100）
  - ヨー回転速度（-100〜100）

### 位置制御アルゴリズム

```python
# 位置誤差を計算
error_x = target_pos["x"] - current_pos["x"]  # 前後方向
error_y = target_pos["y"] - current_pos["y"]  # 上下方向
error_z = target_pos["z"] - current_pos["z"]  # 左右方向

# P制御
fb = int(error_x * PID_GAIN)  # 前後速度
ud = int(error_y * PID_GAIN)  # 上下速度
lr = int(error_z * PID_GAIN)  # 左右速度

# 速度を制限
fb = max(-SPEED, min(SPEED, fb))
ud = max(-SPEED, min(SPEED, ud))
lr = max(-SPEED, min(SPEED, lr))
```

## 座標系

- **モーションキャプチャ座標系**: 右手系
  - X軸: 前後方向
  - Y軸: 上下方向
  - Z軸: 左右方向

- **Telloドローン座標系**:
  - 前後: ピッチ制御（前進/後退）
  - 左右: ロール制御（左/右）
  - 上下: スロットル制御（上昇/下降）
  - 回転: ヨー制御（左回転/右回転）

## 制御パラメータ

| パラメータ | デフォルト値 | 説明 |
|------------|--------------|------|
| SPEED | 50 | 移動速度の最大値（0-100） |
| ROTATION_SPEED | 50 | 回転速度の最大値（0-100） |
| INTERVAL | 0.05 | コマンド送信間隔（秒） |
| PID_GAIN | 20.0 | 位置制御用Pゲイン |
| RIGID_BODY_ID | 1 | モーションキャプチャのリジッドボディID |
| MAX_RUNTIME_SECONDS | 300 | 最大実行時間（秒） |
| DEBUG_LEVEL | 1 | デバッグ出力レベル（0-3） |
| GC_INTERVAL | 100 | ガベージコレクション実行間隔（パケット数） |

## 拡張性

### 複数ドローン対応

現在は1機のドローンのみ制御していますが、将来的に2機目を追加することを想定して、`tello_manager.py`内に2機目のドローン検出コードがコメントアウトされています。

```python
# 2機目のTelloドローン
if num_drones > 1:
    try:
        tello2 = Tello()
        tello2.connect()
        print(f"2機目のTelloドローンを検出しました")
        self.tello_list.append(tello2)
    except Exception as e:
        print(f"2機目のTelloドローンの検出に失敗しました: {e}")
```

### PID制御の拡張

現在はP制御のみ実装していますが、より安定したホバリングのためにI（積分）項とD（微分）項を追加することが可能です。

## 依存ライブラリ

- **pygame**: キーボード入力検出
- **socket**: ネットワーク通信
- **threading**: マルチスレッド処理
- **struct**: バイナリデータ解析
- **time**: 時間管理
- **psutil**: メモリ使用量監視（オプション）
- **gc**: ガベージコレクション制御

## スレッド管理とシャットダウンプロセス

本システムは複数のスレッドを使用して動作し、適切な終了処理を行うためのメカニズムを実装しています。

### 主要スレッド

1. **メインスレッド**: ユーザー入力処理とドローン制御を担当
2. **check_connection_status**: 接続状態を定期的に確認するスレッド
3. **check_termination_conditions**: 終了条件をチェックするスレッド
4. **NatNetClient.__dataThreadFunction**: MOCAPデータ受信用スレッド
5. **CustomTello._receive_thread**: Telloからのレスポンス受信用スレッド

### スレッド管理機構

```python
# 共有の終了イベント
# これがセットされると全スレッドが終了処理を開始する
stop_event = threading.Event()

# スレッド管理用リスト
# 全スレッドを追跡し、適切なjoinを行うために使用
threads = []
```

### シャットダウンプロセス

```python
def graceful_shutdown():
    """スレッドを適切に終了させる関数"""
    global stop_event, threads
    
    logging.info("正常終了処理を開始します")
    
    # 終了イベントをセット
    stop_event.set()
    
    # 全スレッドがjoinされるまで待機
    for t in threads:
        if t.is_alive():
            logging.info(f"スレッド {t.name} の終了を待機中...")
            t.join(timeout=2.0)  # タイムアウトを設定して無限待機を防止
    
    # MOCAPストリームのシャットダウン
    mocap_stream.shutdown()
    
    # Telloマネージャーのシャットダウン
    if tello_manager:
        tello_manager.shutdown()
    
    logging.info("全スレッドの終了処理が完了しました")
```

### キーボード入力による終了

- **スペースキー**: `stop_event`をセットし、正常終了処理を開始
- **ESCキー**: 緊急停止処理を実行（ドローンの即時停止）

### MOCAPデータのみのテストモード

ドローンなしでMOCAPデータの受信とスレッド処理のみをテストするモードを実装しています。

```bash
python3 src/tello_mocap_control.py --mocap-only
```

## 動作環境

- **Python**: 3.6以上（Python3系統で実行すること）
- **OS**: Windows, macOS, Linux
- **ネットワーク**: Wi-Fi（Telloドローン接続用）+ 有線/Wi-Fi（モーションキャプチャシステム接続用）
- **メモリ**: 最低4GB推奨（長時間実行時のメモリ管理機能あり）

## 通信安定性対策

### タイムアウト対策

ドローンとの通信安定性を向上させるため、以下のタイムアウト対策を実装しています：

#### タイムアウト設定値

| コンポーネント | パラメータ | 値 | 説明 |
|--------------|-----------|-----|------|
| CustomTello | socket.settimeout | 10.0秒 | ソケット通信のタイムアウト |
| CustomTello | 受信スレッド | 1.0秒 | 受信スレッドのタイムアウト |
| CustomTello | COMMAND_TIMEOUT | 15.0秒 | コマンド応答待機のタイムアウト |

#### 再試行ロジック

```python
# 初期化時の再試行ロジック例
def initialize_drone():
    max_retries = 3
    retry_count = 0
    
    while retry_count < max_retries:
        try:
            # ドローン接続処理
            return True
        except socket.timeout:
            retry_count += 1
            logging.warning(f"ドローン接続タイムアウト。再試行 {retry_count}/{max_retries}")
            time.sleep(1)
    
    return False
```

#### 連続エラー検出と回復

メインループでは連続エラーを検出し、一定回数（5回）のエラーが発生した場合に一時的な回復モードに入るロジックを実装しています：

```python
# メインループでのエラー回復ロジック例
consecutive_errors = 0
in_recovery_mode = False

try:
    # 通常の処理
    consecutive_errors = 0  # 成功したらリセット
except socket.timeout:
    consecutive_errors += 1
    if consecutive_errors >= 5:
        in_recovery_mode = True
        logging.warning(f"連続エラー検出: {consecutive_errors}回。回復モードに入ります")
        time.sleep(2)  # 回復のための待機
```

### 初期化順序の最適化

通信の安定性を向上させるため、以下の初期化順序を採用しています：

1. **ドローン初期化を先行**: MOCAPシステムより先にドローンを初期化することで、リソース競合を回避
2. **段階的初期化**: 各コンポーネントを段階的に初期化し、各ステップで適切なエラーハンドリングを実施
3. **初期化失敗時の代替パス**: ドローン初期化に失敗した場合でもMOCAPのみのモードで動作継続

```python
# 初期化順序の例
def main():
    # 1. ドローン初期化
    if not args.mocap_only:
        if initialize_drone():
            logging.info("ドローン初期化成功")
        else:
            logging.warning("ドローン初期化失敗。MOCAP-onlyモードで続行")
            args.mocap_only = True
    
    # 2. MOCAP初期化
    initialize_mocap()
    
    # 3. メインループ開始
    main_control_loop()
```

この初期化順序により、以下の効果が得られます：

- リソース競合の軽減
- 各コンポーネントの初期化状態の明確化
- 部分的な機能停止時でもシステム全体の継続運用
