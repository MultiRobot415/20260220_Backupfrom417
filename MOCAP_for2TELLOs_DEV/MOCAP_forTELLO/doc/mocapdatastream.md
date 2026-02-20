# モーションキャプチャデータストリーミング技術詳細

## 概要

このドキュメントでは、OptiTrackモーションキャプチャシステムからのデータを安定して継続的に受信するための技術的詳細について説明します。NatNetClientを使用したデータ受信の仕組み、スレッド管理、エラー処理などの実装方法を解説します。

## システム構成

### ハードウェア構成

- **OptiTrackシステム**: カメラ、マーカー、リジッドボディ
- **MOCAPサーバー**: OptiTrackのPCで実行されるMotive/Arenaソフトウェア（IPアドレス: 192.168.11.2）
- **クライアントPC**: Telloドローンを制御するPC（IPアドレス: 192.168.11.13）

### ソフトウェア構成

- **NatNetSDK**: OptiTrackのデータストリーミングプロトコル
- **NatNetClient.py**: Pythonで実装されたNatNetクライアント
- **mocap_stream.py**: MOCAPデータを継続的に受信するためのモジュール

## NatNetClientの仕組み

### 通信プロトコル

NatNetは、OptiTrackのモーションキャプチャシステムからリアルタイムでデータを受信するためのプロトコルです。UDPを使用して高速なデータ転送を実現しています。

- **コマンドポート**: 1510（デフォルト）
- **データポート**: 1511（デフォルト）

### データフォーマット

NatNetから受信するデータには以下の情報が含まれます：

1. **フレーム情報**: フレーム番号、タイムスタンプなど
2. **マーカーデータ**: 各マーカーの3D位置
3. **リジッドボディデータ**: ID、位置（x, y, z）、回転（クォータニオン: x, y, z, w）、マーカーエラーなど
4. **スケルトンデータ**: 複数のリジッドボディから構成される階層構造

### 座標系

OptiTrackの座標系は右手系で、以下のように定義されています：

- **X軸**: 前後方向（前がプラス）
- **Y軸**: 上下方向（上がプラス）
- **Z軸**: 左右方向（右がプラス）

## 実装詳細

### スレッド安全なロギング

システムの安定性を向上させるため、すべての`print`文を`logging`モジュールに置き換えました。これにより、マルチスレッド環境でも安全にログ出力が行えるようになりました。

```python
# ロギング設定
import logging

log_format = '%(asctime)s - %(levelname)s - [NATNET] %(message)s'
logging.basicConfig(level=logging.INFO, format=log_format)
logger = logging.getLogger()

# ログ出力例
logging.info("NatNetClient: 接続を開始します")
logging.warning("パケットロスを検出しました")
logging.error(f"ソケットエラー: {e}")
```

### 安全なシャットダウン処理

適切なシャットダウン処理を実装し、すべてのスレッドが正常に終了するようにしました。これにより、「could not acquire lock for stdout」などのエラーを防止します。

```python
# 終了フラグ（スレッド間で共有）
stop_event = threading.Event()

# スレッド管理リスト
threads = []

# シャットダウン処理
def shutdown():
    logging.info("モーションキャプチャシステムの終了処理を開始します...")
    
    # 終了フラグを設定
    stop_event.set()
    logging.info("stop_eventを設定しました")
    
    # すべてのスレッドの終了を待機
    for i, thread in enumerate(threads):
        logging.info(f"スレッド {i+1}/{len(threads)} の終了を待っています...")
        thread.join()
    
    # ソケットを閉じる
    if mocap_client:
        mocap_client.shutdown()
    
    logging.info("モーションキャプチャシステムとの接続を閉じました。")
```

### 例外処理の改善

`socket.timeout`などの例外を適切に処理し、エラーメッセージをログに記録するようにしました。

```python
try:
    data, addr = self.data_socket.recvfrom(SOCKET_BUFFER_SIZE)
    # データ処理...
except timeout_error:
    wait_count += 1
    # 5回のタイムアウトごとにメッセージを表示
except OSError as e:
    if self.stop_event.is_set():
        logging.info("__dataThreadFunction: ソケットが閉じられました")
        return
    else:
        logging.error(f"ソケットエラー: {e}")
```

### NatNetClientの初期化と接続

```python
# NatNetClientを初期化
mocap_client = NatNetClient()

# サーバーのIPアドレスを設定（OptiTrackのPCのIPアドレス）
server_ip = "192.168.11.2"
mocap_client.set_server_address(server_ip)

# ローカルのIPアドレスを設定（このPCのIPアドレス）
local_ip = "192.168.11.13"
mocap_client.set_local_address(local_ip)

# コールバック関数を設定
mocap_client.rigidBodyListener = receive_rigid_body_frame

# 非同期モードで実行（スレッドをブロックしない）
success = mocap_client.run()
```

### コールバック関数の実装

NatNetClientは非同期でデータを受信し、設定されたコールバック関数を呼び出します。リジッドボディデータを受信するためのコールバック関数は以下のように実装します：

```python
def receive_rigid_body_frame(id, position, rotation, mean_error=0.0, tracking_valid=True):
    """
    モーションキャプチャからリジッドボディフレームを受信するコールバック関数
    :param id: リジッドボディID
    :param position: 位置 (x, y, z)
    :param rotation: 回転（クォータニオン: x, y, z, w）
    :param mean_error: 平均マーカーエラー（オプション）
    :param tracking_valid: トラッキングが有効かどうか（オプション）
    """
    global current_pos, current_rot, current_error, received_packets, last_packet_time
    
    # 指定されたリジッドボディIDのデータのみ処理
    if RIGID_BODY_ID is not None and id != RIGID_BODY_ID:
        return
    
    # トラッキングが無効な場合はスキップ
    if not tracking_valid:
        return
    
    # パケット数をカウントと最終受信時間の更新
    received_packets += 1
    last_packet_time = time.time()
    
    # データの更新（スレッドセーフに）
    with data_lock:
        current_pos["x"] = position[0]
        current_pos["y"] = position[1]
        current_pos["z"] = position[2]
        
        current_rot["x"] = rotation[0]
        current_rot["y"] = rotation[1]
        current_rot["z"] = rotation[2]
        current_rot["w"] = rotation[3]
        
        current_error = mean_error
```

### スレッド管理

MOCAPデータの受信は非同期で行われるため、複数のスレッドを適切に管理する必要があります：

1. **メインスレッド**: プログラムの実行を制御
2. **NatNetClientスレッド**: データ受信を担当（内部的に作成される）
3. **接続状態確認スレッド**: 定期的に接続状態を確認

```python
# 接続状態確認スレッドを開始
status_thread = threading.Thread(target=check_connection_status)
status_thread.daemon = True  # メインスレッドが終了したら一緒に終了
status_thread.start()
```

### 接続状態の監視

接続状態を監視するためのスレッド関数を実装し、定期的にパケット受信レートを計算します：

```python
def check_connection_status():
    """
    接続状態を定期的に確認するスレッド関数
    """
    last_check_packets = 0
    last_check_time = time.time()
    
    while not should_stop:
        # 10秒ごとに接続状態を確認
        time.sleep(10)
        
        current_time = time.time()
        elapsed_time = current_time - last_check_time
        new_packets = received_packets - last_check_packets
        
        # パケット受信レートを計算
        rate = new_packets / elapsed_time if elapsed_time > 0 else 0
        
        # 5秒以上パケットが受信されていない場合は警告
        if last_packet_time is not None and current_time - last_packet_time > 5:
            print(f"警告: {current_time - last_packet_time:.1f}秒間データが受信されていません")
        
        # 状態更新
        last_check_packets = received_packets
        last_check_time = current_time
```

### スレッドセーフなデータアクセス

複数のスレッドからデータにアクセスする場合、競合状態を避けるためにロックを使用します：

```python
# スレッド同期用のロック
data_lock = threading.Lock()

def get_current_position():
    """
    現在の位置データを取得する
    :return: 位置データの辞書 {"x": x, "y": y, "z": z}
    """
    with data_lock:
        return dict(current_pos)
```

### メモリ管理

長時間の実行でメモリリークを防ぐため、定期的にガベージコレクションを実行します：

```python
# 定期的にガベージコレクションを実行
if received_packets % GC_INTERVAL == 0:
    gc.collect()
```

### エラー処理とリトライ

接続に失敗した場合のリトライ処理を実装します：

```python
# 接続を試行（3回まで再試行）
max_retries = 3
success = False

for attempt in range(1, max_retries + 1):
    try:
        success = mocap_client.run()
        if success:
            break
    except Exception as e:
        print(f"接続試行 {attempt} 中にエラー発生: {e}")
    
    if attempt < max_retries:
        time.sleep(1)  # 1秒待機してから再試行
```

## 一般的な問題と解決策

### 接続エラー

- **問題**: NatNetClientがサーバーに接続できない
- **解決策**: 
  - IPアドレスの設定を確認
  - ファイアウォールの設定を確認
  - OptiTrackのMotiveソフトウェアでデータストリーミングが有効になっているか確認

### データが受信されない

- **問題**: 接続は成功するがデータが受信されない
- **解決策**:
  - リジッドボディが正しく定義されているか確認
  - リジッドボディIDが正しいか確認
  - カメラがリジッドボディを追跡できているか確認

### パケットロス

- **問題**: データが断続的に受信される
- **解決策**:
  - ネットワーク負荷を確認
  - UDPパケットサイズを調整
  - バッファサイズを増やす

## モジュールの使用方法

### 基本的な使用方法

```python
import mocap_stream

# 初期化
mocap_stream.initialize(rigid_body_id=1)

# データ取得
position = mocap_stream.get_current_position()
rotation = mocap_stream.get_current_rotation()

# 終了
mocap_stream.shutdown()
```

### 接続状態の確認

```python
status = mocap_stream.get_connection_status()
if status["connected"]:
    print("MOCAPシステムに接続中")
else:
    print("MOCAPシステムとの接続が切断されています")
```

### バッファデータの取得

```python
# 過去のデータを取得
position_history = mocap_stream.get_position_buffer()
rotation_history = mocap_stream.get_rotation_buffer()
```

## まとめ

OptiTrackのモーションキャプチャシステムからデータを安定して継続的に受信するためには、以下の点が重要です：

1. **適切な初期化**: 正しいIPアドレスの設定とコールバック関数の登録
2. **非同期処理**: スレッドを適切に管理し、データの競合を避ける
3. **接続監視**: 定期的に接続状態を確認し、問題を検出する
4. **エラー処理**: 例外を適切に処理し、必要に応じてリトライする
5. **メモリ管理**: 長時間の実行でもメモリリークを防ぐ

これらの技術を適切に実装することで、MOCAPデータを安定して受信し、ドローン制御などのリアルタイムアプリケーションに活用することができます。
