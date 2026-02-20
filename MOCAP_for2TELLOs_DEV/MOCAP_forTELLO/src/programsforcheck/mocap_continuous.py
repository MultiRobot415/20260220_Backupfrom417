#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
MOCAP連続受信テスト - モーションキャプチャシステムからのデータを継続的に受信するプログラム

このプログラムは、OptiTrackモーションキャプチャシステムからの位置データを継続的に受信し、
メインスレッドが終了しないようにします。ユーザーが明示的に終了するまで実行し続けます。

使用方法:
1. OptiTrackのMotiveソフトウェアでデータストリーミングを有効にする
2. このプログラムを実行する
3. Ctrl+Cで終了する

注意:
- IPアドレス設定を環境に合わせて変更してください
- リジッドボディIDを適切に設定してください
"""

import sys
import time
import threading
import signal
from NatNetClient import NatNetClient

# グローバル変数
mocap_client = None  # NatNetClientオブジェクト
exit_event = threading.Event()  # 終了イベント

# ドローンの現在位置（モーションキャプチャから取得）
current_pos = {"x": 0.0, "y": 0.0, "z": 0.0}  # x: 前後, y: 上下, z: 左右
current_rot = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}  # クォータニオン

# モーションキャプチャのリジッドボディID
RIGID_BODY_ID = 1  # 追跡するリジッドボディID（必要に応じて変更）

# データ受信の統計情報
received_packets = 0     # 受信したパケット数
start_time = None        # 開始時間
last_packet_time = None  # 最後のパケット受信時間

# デバッグ設定
DEBUG_LEVEL = 1  # 0=最小限, 1=通常, 2=詳細
PRINT_INTERVAL = 10  # 何パケットごとに情報を表示するか

def debug_print(level, message):
    """
    デバッグレベルに応じて出力を制御する関数
    :param level: メッセージのデバッグレベル
    :param message: 出力するメッセージ
    """
    if DEBUG_LEVEL >= level:
        print(message)

def receive_rigid_body_frame(id, position, rotation):
    """
    モーションキャプチャからリジッドボディフレームを受信するコールバック関数
    :param id: リジッドボディID
    :param position: 位置 (x, y, z)
    :param rotation: 回転（クォータニオン: x, y, z, w）
    """
    global current_pos, current_rot, received_packets, last_packet_time
    
    # 指定されたリジッドボディIDのデータのみ処理
    if RIGID_BODY_ID is not None and id != RIGID_BODY_ID:
        return
    
    # パケット数をカウント
    received_packets += 1
    last_packet_time = time.time()
    
    # モーションキャプチャの座標系が右手系で、前後方向がx，上下方向がy，左右方向がzであることに注意
    current_pos["x"] = position[0]  # 前後方向
    current_pos["y"] = position[1]  # 上下方向
    current_pos["z"] = position[2]  # 左右方向
    
    current_rot["x"] = rotation[0]
    current_rot["y"] = rotation[1]
    current_rot["z"] = rotation[2]
    current_rot["w"] = rotation[3]
    
    # 一定間隔で情報を出力
    if received_packets % PRINT_INTERVAL == 0 or received_packets <= 3:
        debug_print(1, f"MOCAPデータ受信: ID={id}, 位置: x={current_pos['x']:.2f}, y={current_pos['y']:.2f}, z={current_pos['z']:.2f}")
        debug_print(2, f"回転: x={current_rot['x']:.2f}, y={current_rot['y']:.2f}, z={current_rot['z']:.2f}, w={current_rot['w']:.2f}")
        
        # 受信レートを計算
        if start_time is not None and last_packet_time is not None:
            elapsed_time = last_packet_time - start_time
            rate = received_packets / elapsed_time if elapsed_time > 0 else 0
            debug_print(1, f"受信パケット数: {received_packets}, 平均レート: {rate:.2f} パケット/秒")

def initialize_mocap():
    """モーションキャプチャシステムの初期化と接続"""
    global mocap_client
    
    debug_print(1, "モーションキャプチャシステムへの接続を開始します...")
    
    # NatNetClientを初期化
    mocap_client = NatNetClient()
    
    # サーバーのIPアドレスを設定（OptiTrackのPCのIPアドレス）
    # MOCAP PCのIPアドレス: 192.168.11.2
    server_ip = "192.168.11.2"
    mocap_client.set_server_address(server_ip)
    debug_print(1, f"MOCAPサーバーのIPアドレス: {server_ip}")
    
    # ローカルのIPアドレスを設定（このPCのIPアドレス）
    # ローカルPCのIPアドレス: 192.168.11.13
    local_ip = "192.168.11.13"
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
            return True
        time.sleep(0.1)
    
    # タイムアウト後もデータが受信されない場合
    if received_packets == 0:
        debug_print(1, "警告: モーションキャプチャデータが受信されませんでした。")
        debug_print(1, "リジッドボディの設定やネットワーク接続を確認してください。")
        # 接続自体は成功しているのでTrueを返す
        return True
    
    return True

def signal_handler(sig, frame):
    """
    シグナルハンドラ関数（Ctrl+Cで終了するため）
    """
    print("\nプログラムを終了します...")
    exit_event.set()

def check_connection_status():
    """
    接続状態を定期的に確認するスレッド関数
    """
    last_check_packets = 0
    last_check_time = time.time()
    
    while not exit_event.is_set():
        # 10秒ごとに接続状態を確認
        time.sleep(10)
        
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

def main():
    """メイン関数"""
    global start_time
    
    print("MOCAP連続受信テスト - モーションキャプチャシステムからのデータを継続的に受信するプログラム")
    print("-----------------------------------------------------------------------")
    print("Ctrl+Cで終了します")
    
    # シグナルハンドラを設定（Ctrl+Cで終了するため）
    signal.signal(signal.SIGINT, signal_handler)
    
    # 開始時間を記録
    start_time = time.time()
    
    # モーションキャプチャシステムの初期化
    if not initialize_mocap():
        print("モーションキャプチャシステムの初期化に失敗しました。プログラムを終了します。")
        return
    
    # 接続状態確認スレッドを開始
    status_thread = threading.Thread(target=check_connection_status)
    status_thread.daemon = True
    status_thread.start()
    
    print("モーションキャプチャデータの受信を開始しました。Ctrl+Cで終了します。")
    
    # メインループ - プログラムが終了しないようにする
    try:
        # exit_eventが設定されるまで待機
        while not exit_event.is_set():
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
        if mocap_client:
            mocap_client.shutdown()
            print("モーションキャプチャシステムとの接続を閉じました。")

if __name__ == "__main__":
    main()
