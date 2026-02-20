#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mocap_stream.py - モーションキャプチャデータストリーミングモジュール

このモジュールは、OptiTrackのモーションキャプチャシステムからのデータを
継続的に受信するための機能を提供します。NatNetClientを使用して
非同期でデータを受信し、スレッドセーフな方法でデータを管理します。

主な機能:
- モーションキャプチャシステムへの接続
- リジッドボディデータの継続的な受信
- 接続状態の監視
- デバッグ出力の制御

使用例:
    import mocap_stream
    
    # 初期化
    mocap_stream.initialize(rigid_body_id=1)
    
    # データ取得
    position = mocap_stream.get_current_position()
    rotation = mocap_stream.get_current_rotation()
    
    # 終了
    mocap_stream.shutdown()
"""

import time
import threading
import gc
import signal
import sys
import os
import psutil
import logging
from collections import deque

# NatNetClientのインポート
try:
    from NatNetClient import NatNetClient
except ImportError:
    # 現在のディレクトリにNatNetClient.pyがある場合
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if os.path.exists(os.path.join(current_dir, "NatNetClient.py")):
        sys.path.append(current_dir)
        from NatNetClient import NatNetClient
    else:
        print("エラー: NatNetClient.pyが見つかりません。")
        print("NatNetClient.pyをこのスクリプトと同じディレクトリに配置するか、")
        print("PYTHONPATHにNatNetClient.pyのあるディレクトリを追加してください。")
        sys.exit(1)

# グローバル変数
mocap_client = None
should_stop = False
start_time = None
last_packet_time = None
received_packets = 0
mocap_threads = []  # スレッド管理用リスト

# IDごとのデータ保持
rigid_body_positions = {}  # IDごとの位置を保持する辞書 {ID: {"x": x, "y": y, "z": z}}
rigid_body_rotations = {}  # IDごとの回転を保持する辞書 {ID: {"x": x, "y": y, "z": z, "w": w}}
rigid_body_errors = {}     # IDごとのエラーを保持する辞書 {ID: error}
rigid_body_tracked_ids = set()  # 追跡中のID一覧

# 従来の互換性のために維持する変数
current_pos = {"x": 0.0, "y": 0.0, "z": 0.0}   # 現在の位置
current_rot = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}  # 現在の回転
current_error = 0.0               # 現在のエラー

# バッファサイズとインターバル設定
BUFFER_SIZE = 100      # 位置と回転のバッファに保持するフレーム数
PRINT_INTERVAL = 10000   # デバッグメッセージを出力する間隔を非常に長く設定
DETAIL_DEBUG_INTERVAL = 100000  # 詳細なデバッグ情報は実質的に出力しない
CONNECTION_TIMEOUT = 3.0  # 接続タイムアウト時間（秒）
GC_INTERVAL = 10000     # ガベージコレクションも最低限にする

# デバッグレベル設定
DEBUG_LEVEL = 0  # デバッグ出力を完全に無効化

# データバッファを再定義
position_buffer = deque(maxlen=BUFFER_SIZE)  # 位置データバッファ
rotation_buffer = deque(maxlen=BUFFER_SIZE)  # 回転データバッファ
error_buffer = deque(maxlen=BUFFER_SIZE)     # エラーデータバッファ

# 実行時間の制限（秒）
MAX_RUNTIME_SECONDS = 3600  # デフォルト: 1時間

# リジッドボディID（Noneの場合はすべてのリジッドボディを受信）
RIGID_BODY_ID = None

# スレッド同期用のロック
data_lock = threading.Lock()

# フレームカウンター（デバッグ出力抑制用）
frame_counter = 0

def debug_print(level, message):
    """
    デバッグレベルに応じて出力を制御する関数
    :param level: メッセージのデバッグレベル
    :param message: 出力するメッセージ
    """
    # デバッグ出力を完全に無効化
    pass
    
    # 元のデバッグコード（必要に応じて復活させる）
    # global frame_counter
    # if level >= 3 and frame_counter % DETAIL_DEBUG_INTERVAL != 0:
    #     return
    # if DEBUG_LEVEL >= level and (not 'stop_event' in globals() or not stop_event.is_set()):
    #     if level == 1:
    #         logging.warning(message)
    #     elif level == 2:
    #         logging.info(message)
    #     elif level >= 3:
    #         logging.debug(message)
    #     else:
    #         logging.info(message)

def monitor_memory_usage():
    """
    現在のプロセスのメモリ使用量を監視する関数
    :return: 使用メモリ量（MB）
    """
    try:
        process = psutil.Process()
        memory_info = process.memory_info()
        memory_mb = memory_info.rss / 1024 / 1024  # バイトからMBに変換
        return memory_mb
    except:
        return 0.0  # psutilがインストールされていない場合

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
    global rigid_body_positions, rigid_body_rotations, rigid_body_errors, rigid_body_tracked_ids
    global frame_counter
    
    try:
        # フレームカウンター更新
        frame_counter += 1
        
        # 指定されたリジッドボディIDのデータのみ処理
        if RIGID_BODY_ID is not None and id != RIGID_BODY_ID:
            return
        
        # トラッキングが無効な場合はスキップ
        if not tracking_valid:
            # トラッキング警告も間引く
            if frame_counter % PRINT_INTERVAL == 0:
                debug_print(2, f"警告: ID {id} のトラッキングが無効です。データをスキップします。")
            return
        
        # パケット数をカウントと最終受信時間の更新
        received_packets += 1
        last_packet_time = time.time()
        
        with data_lock:
            # IDごとのデータを更新
            # 位置を更新
            rigid_body_positions[id] = {
                "x": position[0],
                "y": position[1],
                "z": position[2]
            }
            
            # 回転を更新
            rigid_body_rotations[id] = {
                "x": rotation[0],
                "y": rotation[1],
                "z": rotation[2],
                "w": rotation[3]
            }
            
            # エラーを更新
            rigid_body_errors[id] = mean_error
            
            # 追跡中のIDを記録
            rigid_body_tracked_ids.add(id)
            
            # 従来の互換性のために保持する変数も更新
            current_pos["x"] = position[0]
            current_pos["y"] = position[1]
            current_pos["z"] = position[2]
            
            current_rot["x"] = rotation[0]
            current_rot["y"] = rotation[1]
            current_rot["z"] = rotation[2]
            current_rot["w"] = rotation[3]
            
            current_error = mean_error
            
            # バッファにデータを追加
            position_buffer.append(dict(current_pos))  # 辞書のコピーを保存
            rotation_buffer.append(dict(current_rot))  # 辞書のコピーを保存
            error_buffer.append(current_error)
        
        # 一定間隔で情報を出力（間引く）
        if received_packets % PRINT_INTERVAL == 0 or received_packets <= 3:
            debug_print(1, f"MOCAPデータ受信: ID={id}, 位置: x={current_pos['x']:.2f}, y={current_pos['y']:.2f}, z={current_pos['z']:.2f} - エラー: {current_error:.6f}")
            
            # 受信レートを計算 (こちらも間引く)
            if start_time is not None and last_packet_time is not None:
                elapsed_time = last_packet_time - start_time
                rate = received_packets / elapsed_time if elapsed_time > 0 else 0
                debug_print(1, f"受信パケット数: {received_packets}, 平均レート: {rate:.2f} パケット/秒")
            
            # メモリ使用量を表示（レベル2以上かつ間引く）
            if DEBUG_LEVEL >= 2:
                memory_usage = monitor_memory_usage()
                debug_print(2, f"メモリ使用量: {memory_usage:.2f} MB - バッファサイズ: {len(position_buffer)}")
        
        # 詳細な回転情報（レベル3のみ）- 追加の間引きは debug_print 内で実施
        if frame_counter % DETAIL_DEBUG_INTERVAL == 0:  # フレーム間引き
            debug_print(3, f"MOCAP回転情報: x={current_rot['x']:.2f}, y={current_rot['y']:.2f}, z={current_rot['z']:.2f}, w={current_rot['w']:.2f}")
        
        # 定期的にガベージコレクションを実行（間隔を広げる）
        if received_packets % GC_INTERVAL == 0:
            debug_print(2, "ガベージコレクションを実行中...")
            gc.collect()
            
    except Exception as e:
        debug_print(1, f"エラー: リジッドボディデータの処理中に例外が発生しました: {e}")

def check_connection_status():
    """
    接続状態を定期的に確認するスレッド関数
    """
    global last_packet_time, stop_event
    last_check_packets = 0
    last_check_time = time.time()
    
    debug_print(2, "接続状態確認スレッドを開始しました")
    
    while not should_stop and not stop_event.is_set():
        # 10秒ごとに接続状態を確認
        # 1秒ごとにstop_eventをチェックして素早く終了できるようにする
        for _ in range(10):
            if should_stop or stop_event.is_set():
                logging.info("接続状態確認スレッドを終了します")
                return
            time.sleep(1)
        
        current_time = time.time()
        elapsed_time = current_time - last_check_time
        new_packets = received_packets - last_check_packets
        
        # パケット受信レートを計算
        rate = new_packets / elapsed_time if elapsed_time > 0 else 0
        
        debug_print(1, f"接続状態: 過去{elapsed_time:.1f}秒間に{new_packets}パケット受信 (レート: {rate:.2f}パケット/秒)")
        
        # 5秒以上パケットが受信されていない場合は警告
        if last_packet_time is not None and current_time - last_packet_time > 5:
            debug_print(1, f"警告: {current_time - last_packet_time:.1f}秒間データが受信されていません")
        
        # 状態更新
        last_check_packets = received_packets
        last_check_time = current_time

def initialize(rigid_body_id=None, debug_level=1, server_ip="192.168.11.2", local_ip="192.168.11.13"):
    """
    モーションキャプチャシステムの初期化と接続
    
    :param rigid_body_id: 追跡するリジッドボディのID（Noneの場合はすべてのリジッドボディを受信）
    :param debug_level: デバッグ出力レベル（0=最小限, 1=通常, 2=詳細, 3=すべて）
    :param server_ip: MOCAPサーバー（OptiTrack PC）のIPアドレス
    :param local_ip: ローカルPCのIPアドレス
    :return: 初期化が成功したかどうか
    """
    global mocap_client, RIGID_BODY_ID, DEBUG_LEVEL, start_time, should_stop, stop_event, mocap_threads
    
    # ロギング設定
    log_format = '%(asctime)s - %(levelname)s - [MOCAP] %(message)s'
    if not logging.getLogger().handlers:
        logging.basicConfig(level=logging.DEBUG, format=log_format)
    
    # グローバル変数の初期化
    RIGID_BODY_ID = rigid_body_id
    DEBUG_LEVEL = debug_level
    start_time = time.time()
    stop_event = threading.Event()
    mocap_threads = []
    should_stop = False
    
    try:
        debug_print(1, "モーションキャプチャシステムへの接続を開始します...")
        
        # NatNetClientを初期化
        mocap_client = NatNetClient()
        
        # サーバーのIPアドレスを設定（OptiTrackのPCのIPアドレス）
        mocap_client.set_server_address(server_ip)
        debug_print(1, f"MOCAPサーバーのIPアドレス: {server_ip}")
        
        # ローカルのIPアドレスを設定（このPCのIPアドレス）
        mocap_client.set_local_address(local_ip)
        debug_print(1, f"ローカルPCのIPアドレス: {local_ip}")
        
        # コールバック関数を設定
        mocap_client.rigidBodyListener = receive_rigid_body_frame
        
        # 接続を試行（3回まで再試行）
        max_retries = 3
        success = False
        
        for attempt in range(1, max_retries + 1):
            debug_print(1, f"接続試行 {attempt}/{max_retries}...")
            try:
                # 非同期モードで実行（スレッドをブロックしない）
                success = mocap_client.run()
                if success:
                    debug_print(1, "モーションキャプチャクライアントが正常に開始されました")
                    break
            except Exception as e:
                debug_print(1, f"接続試行 {attempt} 中にエラー発生: {e}")
            
            if attempt < max_retries:
                debug_print(1, "接続に失敗しました。再試行します...")
                time.sleep(1)  # 1秒待機してから再試行
        
        if not success:
            debug_print(1, "モーションキャプチャシステムへの接続に失敗しました。")
            debug_print(1, "IPアドレスの設定を確認してください。")
            return False
        
        # データ受信の確認
        debug_print(1, "モーションキャプチャシステムに接続しました。データ受信を待機します...")
        
        # データ受信を確認するために少し待機
        wait_start = time.time()
        wait_timeout = 5.0  # 5秒間待機
        
        while time.time() - wait_start < wait_timeout:
            if received_packets > 0:
                debug_print(1, f"モーションキャプチャデータの受信を確認しました ({received_packets} パケット)")
                
                # 接続状態確認スレッドを開始
                status_thread = threading.Thread(target=check_connection_status)
                status_thread.daemon = False  # 非デーモンスレッドに変更
                status_thread.start()
                mocap_threads.append(status_thread)  # スレッドリストに追加
                
                return True
            time.sleep(0.1)
        
        # タイムアウト後もデータが受信されない場合
        if received_packets == 0:
            debug_print(1, "警告: モーションキャプチャデータが受信されませんでした。")
            debug_print(1, "リジッドボディの設定やネットワーク接続を確認してください。")
            # 接続自体は成功しているのでTrueを返す
            return True
        
        return True
        
    except Exception as e:
        debug_print(1, f"モーションキャプチャシステムの初期化中にエラーが発生しました: {e}")
        return False

def get_current_position(rigid_body_id=None):
    """
    現在の位置データを取得する
    :param rigid_body_id: 取得したいリジッドボディのID。Noneの場合は当初の単一RB用のデータを返す。
    :return: 位置データの辞書 {"x": x, "y": y, "z": z}
    """
    with data_lock:
        if rigid_body_id is not None:
            if rigid_body_id in rigid_body_positions:
                return dict(rigid_body_positions[rigid_body_id])
            return None
        return dict(current_pos)  # 従来の互換性用

def get_current_rotation(rigid_body_id=None):
    """
    現在の回転データを取得する
    :param rigid_body_id: 取得したいリジッドボディのID。Noneの場合は当初の単一RB用のデータを返す。
    :return: 回転データの辞書 {"x": x, "y": y, "z": z, "w": w}
    """
    with data_lock:
        if rigid_body_id is not None:
            if rigid_body_id in rigid_body_rotations:
                return dict(rigid_body_rotations[rigid_body_id])
            return None
        return dict(current_rot)  # 従来の互換性用

def get_current_error(rigid_body_id=None):
    """
    現在のエラー値を取得する
    :param rigid_body_id: 取得したいリジッドボディのID。Noneの場合は当初の単一RB用のデータを返す。
    :return: エラー値
    """
    with data_lock:
        if rigid_body_id is not None:
            if rigid_body_id in rigid_body_errors:
                return rigid_body_errors[rigid_body_id]
            return None
        return current_error  # 従来の互換性用

def get_position_buffer():
    """
    位置データのバッファを取得する
    :return: 位置データのリスト
    """
    with data_lock:
        return list(position_buffer)

def get_rotation_buffer():
    """
    回転データのバッファを取得する
    :return: 回転データのリスト
    """
    with data_lock:
        return list(rotation_buffer)

def get_error_buffer():
    """
    エラーデータのバッファを取得する
    :return: エラーデータのリスト
    """
    with data_lock:
        return list(error_buffer)

def get_received_packets():
    """
    受信したパケット数を取得する
    :return: パケット数
    """
    return received_packets

def get_tracked_rigid_body_ids():
    """
    追跡中のリジッドボディIDのリストを取得する
    :return: リジッドボディIDのリスト
    """
    with data_lock:
        return list(rigid_body_tracked_ids)

def get_connection_status():
    """
    接続状態を取得する
    :return: 接続状態の辞書 {"connected": bool, "last_packet_time": float, "packets": int, "tracked_ids": list}
    """
    with data_lock:
        # 最後にパケットを受信してからの経過時間を計算
        elapsed = 0.0
        if last_packet_time is not None and last_packet_time > 0:
            elapsed = time.time() - last_packet_time
        
        return {
            "connected": True if received_packets > 0 and elapsed < 5.0 else False,
            "last_packet_time": last_packet_time if last_packet_time is not None else 0.0,
            "packets": received_packets,
            "tracked_ids": list(rigid_body_tracked_ids)
        }

def shutdown():
    """
    モーションキャプチャシステムとの接続を終了する
    """
    global should_stop, mocap_client, mocap_threads, stop_event
    
    logging.info("モーションキャプチャシステムの終了処理を開始します...")
    
    # 終了フラグを設定
    should_stop = True
    
    # stop_eventを設定して全スレッドに終了を通知
    if 'stop_event' in globals() and stop_event is not None:
        stop_event.set()
        logging.info("stop_eventを設定しました")
    
    # 各スレッドが終了するのを待つ
    if 'mocap_threads' in globals() and mocap_threads:
        for i, thread in enumerate(mocap_threads):
            if thread and thread.is_alive():
                logging.info(f"スレッド {i+1}/{len(mocap_threads)} の終了を待っています...")
                thread.join(timeout=2.0)  # 最大2秒間待つ
                if thread.is_alive():
                    logging.warning(f"警告: スレッド {i+1} がタイムアウトしました")
    
    # NatNetClientをシャットダウン
    if mocap_client:
        mocap_client.shutdown()
        logging.info("モーションキャプチャシステムとの接続を閉じました。")

def signal_handler(sig, frame):
    """
    シグナルハンドラ関数（Ctrl+Cで終了するため）
    """
    global should_stop
    print("\nプログラムを終了します...")
    should_stop = True
    shutdown()
    sys.exit(0)

def main():
    """
    メイン関数 - モジュールを単体で実行した場合のテスト用
    """
    global start_time
    
    print("MOCAP連続受信テスト - モーションキャプチャシステムからのデータを継続的に受信するプログラム")
    print("-----------------------------------------------------------------------")
    print("Ctrl+Cで終了します")
    
    # シグナルハンドラを設定（Ctrl+Cで終了するため）
    signal.signal(signal.SIGINT, signal_handler)
    
    # 開始時間を記録
    start_time = time.time()
    
    # モーションキャプチャシステムの初期化
    if not initialize():
        print("モーションキャプチャシステムの初期化に失敗しました。プログラムを終了します。")
        return
    
    print("モーションキャプチャデータの受信を開始しました。Ctrl+Cで終了します。")
    
    # メインループ - プログラムが終了しないようにする
    try:
        # should_stopが設定されるまで待機
        while not should_stop:
            # CPUを占有しないように少し待機
            time.sleep(0.1)
    except KeyboardInterrupt:
        # Ctrl+Cが押された場合
        print("\nユーザーによりプログラムが中断されました。")
    finally:
        # 実行時間とパケット数の統計を表示
        elapsed_time = time.time() - start_time
        print(f"\n実行統計:")
        print(f"- 実行時間: {elapsed_time:.2f}秒")
        print(f"- 受信パケット数: {received_packets}")
        print(f"- 平均パケットレート: {received_packets/elapsed_time if elapsed_time > 0 else 0:.2f}パケット/秒")
        
        # クリーンアップ
        shutdown()

if __name__ == "__main__":
    main()
