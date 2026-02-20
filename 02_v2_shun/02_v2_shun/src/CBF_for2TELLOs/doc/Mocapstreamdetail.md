# MOCAP Stream モジュール詳細仕様書

## 概要

`mocap_stream.py`は、OptiTrackモーションキャプチャシステムからNatNetプロトコルを使用してリアルタイムでポーズデータを受信・処理するためのPythonモジュールです。このモジュールはTelloドローンの自律飛行制御において位置情報のフィードバックを提供する中核コンポーネントです。

## 依存関係

* **NatNetClient.py**: OptiTrack NatNetプロトコルを実装したクライアントライブラリ
* **socket**: UDPソケット通信用の標準ライブラリ
* **threading**: マルチスレッド処理用の標準ライブラリ
* **time, datetime**: タイムスタンプと時間管理用の標準ライブラリ
* **logging**: ログ出力管理用の標準ライブラリ
* **signal**: シグナル処理（終了検出）用の標準ライブラリ

## アーキテクチャ

```
                   +-------------------+
                   | OptiTrack System  |
                   | (Server: 1.1.1.1) |
                   +--------+----------+
                            |
                            | NatNet Protocol
                            | (UDP Multicast)
                            |
              +-------------v--------------+
              | NatNetClient.py            |
              | - Connection management    |
              | - Protocol parsing         |
              | - Packet handling          |
              +-------------+--------------+
                            |
                            | Callbacks
                            |
              +-------------v--------------+
              | mocap_stream.py            |
              | - Position data processing |
              | - ID-based data storage    |
              | - Thread management        |
              +-------------+--------------+
                            |
                            | API Calls
                            |
+---------------+  +--------v-----------+  +----------------+
| mocap_for_    |  | mocap_stream_test  |  | Other control  |
| 2tellos.py    |  | .py                |  | applications   |
+---------------+  +--------------------+  +----------------+
```

## ネットワーク構成

* **サーバー（OptiTrackシステム）**: `192.168.11.2`
* **コマンドポート**: `1510` (UDP)
* **データポート**: `1511` (UDP マルチキャスト)
* **マルチキャストアドレス**: `239.255.42.99`
* **クライアント（ドローン制御PC）**: `192.168.11.13`

## 主要なデータ構造

### グローバル変数

* `rigid_body_positions`: 各リジッドボディIDの位置データを保持する辞書
* `rigid_body_rotations`: 各リジッドボディIDの回転データを保持する辞書
* `rigid_body_errors`: 各リジッドボディIDのエラー値を保持する辞書
* `rigid_body_tracked_ids`: 現在追跡中のリジッドボディIDのセット
* `current_pos`: 互換性のための従来の位置データ変数
* `current_rot`: 互換性のための従来の回転データ変数
* `current_error`: 互換性のための従来のエラー値変数

## 主要API

### 初期化・終了

* `initialize(rigid_body_id=None, debug_level=1, server_ip="192.168.11.2", local_ip="192.168.11.13")`
  - MOCAPシステムへの接続を初期化
  - `rigid_body_id`: 追跡するリジッドボディID（Noneの場合はすべてのIDを追跡）
  - `debug_level`: ログ出力レベル（0=最小限、3=最大）
  - 戻り値: 初期化が成功したかどうかのブール値

* `shutdown()`
  - MOCAPシステムとの接続を終了
  - ソケットを閉じ、スレッドを停止

### データ取得

* `get_current_position(rigid_body_id=None)`
  - 指定したリジッドボディIDの最新位置データを取得
  - `rigid_body_id`: 取得したいリジッドボディID
  - 戻り値: 位置データの辞書 `{"x": x, "y": y, "z": z}` またはNone

* `get_current_rotation(rigid_body_id=None)`
  - 指定したリジッドボディIDの最新回転データを取得
  - `rigid_body_id`: 取得したいリジッドボディID
  - 戻り値: 回転データの辞書 `{"x": x, "y": y, "z": z, "w": w}` またはNone

* `get_current_error(rigid_body_id=None)`
  - 指定したリジッドボディIDの最新エラー値を取得
  - `rigid_body_id`: 取得したいリジッドボディID
  - 戻り値: エラー値またはNone

* `get_tracked_rigid_body_ids()`
  - 現在追跡中のリジッドボディIDのリストを取得
  - 戻り値: リジッドボディIDのリスト

* `get_connection_status()`
  - 接続状態を取得
  - 戻り値: 接続状態の辞書 `{"connected": bool, "last_packet_time": float, "packets": int, "tracked_ids": list}`

### コールバック関数

* `receive_rigid_body_frame(id, position, rotation, mean_error=0.0, tracking_valid=True)`
  - NatNetClientからリジッドボディフレームを受信するコールバック関数
  - `id`: リジッドボディID
  - `position`: 位置 (x, y, z)
  - `rotation`: 回転（クォータニオン: x, y, z, w）
  - `mean_error`: 平均マーカーエラー
  - `tracking_valid`: トラッキングが有効かどうか

## スレッド

* **データ受信スレッド**: NatNetClientによって内部的に作成され、MOCAPシステムからのUDPパケットを非同期で受信
* **接続状態確認スレッド**: `check_connection_status()`関数によって実行され、定期的に接続状態を確認

## エラーハンドリング

* スレッドセーフなデータアクセスのための`data_lock`（RLock）
* パケット受信タイムアウト検出
* 接続エラー時の再接続試行
* シグナルハンドラによる正常終了処理

## 使用例

### 基本的な使用法

```python
import mocap_stream as ms

# 初期化
ms.initialize(rigid_body_id=None)  # すべてのリジッドボディを追跡

# 追跡中のIDを取得
tracked_ids = ms.get_tracked_rigid_body_ids()
print(f"追跡中のID: {tracked_ids}")

# 特定IDの位置データを取得
for rb_id in tracked_ids:
    pos = ms.get_current_position(rigid_body_id=rb_id)
    if pos:
        print(f"ID {rb_id}: X={pos['x']:.3f}, Y={pos['y']:.3f}, Z={pos['z']:.3f}")

# 終了
ms.shutdown()
```

### mocap_stream_test.pyとの関係

`mocap_stream_test.py`は`mocap_stream.py`モジュールのテスト・実演用スクリプトで、以下の機能を提供します：

1. 複数のリジッドボディID（デフォルトでID 1と2）の同時監視
2. IDごとの位置データのリアルタイム表示
3. IDごとの位置データを別々のJSONファイルに定期的に保存
4. キーボード割り込みによる安全な終了処理

## トラブルシューティング

### データが受信できない場合

1. ネットワーク設定（IPアドレス、ポート）を確認
2. OptiTrackシステムが起動し、NatNetストリーミングが有効になっていることを確認
3. マルチキャストグループへの参加が成功しているか確認（`IP_ADD_MEMBERSHIP`の設定）
4. ファイアウォール設定を確認（UDPポート1510, 1511が開いているか）

### 特定のIDのデータが受信できない場合

1. OptiTrack Motive上で対象のリジッドボディが正しく定義されているか確認
2. リジッドボディのマーカーが適切に配置され、トラッキングが安定しているか確認
3. `rigid_body_id`パラメータが正しく設定されているか確認

### データの更新が遅い場合

1. ネットワーク負荷を確認
2. CPU使用率を確認（パケット処理が追いついていない可能性）
3. デバッグレベルを下げてログ出力を減らす
