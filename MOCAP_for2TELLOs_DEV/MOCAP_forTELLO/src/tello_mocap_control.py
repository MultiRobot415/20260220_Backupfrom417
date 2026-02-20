#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tello MOCAP Control - モーションキャプチャシステムを使用したTelloドローンの制御プログラム

このプログラムは、OptiTrackモーションキャプチャシステムからの位置フィードバックを使用して、
Telloドローンを指定位置でホバリングさせます。

キー操作:
- Q: 離陸
- E: 着陸
- W/S: 上昇/下降
- A/D: 左右回転
- 矢印キー上/下: 前進/後退
- 矢印キー左/右: 左右移動
- ESC: 緊急停止

作成日: 2025-06-02
更新日: 2025-06-13 - メモリ最適化機能を追加
"""

import sys
import time
import math
import socket
import threading
import gc  # ガベージコレクション用
import psutil  # メモリ使用量監視用
import pygame  # Pygameウィンドウ表示用
import logging  # ロギング用
import numpy as np
from datetime import datetime
from NatNetClient import NatNetClient
from custom_tello import TelloManager
import KeyPressModule as kp
from custom_tello import CustomTello, TelloManager
import mocap_stream  # MOCAPデータストリームモジュール
from mocap_stream import MAX_BUFFER_SIZE  # バッファサイズ定数をインポート

# ロギング設定
log_file = f"tello_control_{time.strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)

# グローバル変数
tello_manager = None  # TelloManagerオブジェクト
drone = None  # Telloオブジェクト
is_flying = False  # 飛行状態
should_stop = False  # プログラム終了フラグ
received_packets = 0  # 受信したMOCAPパケット数
stop_event = threading.Event()  # スレッド終了用イベント
threads = []  # スレッド管理用リスト

# 目標位置（デフォルト値）
target_pos = {"x": 0.0, "y": 1.0, "z": 0.0}  # x=0m, y=1m, z=0m

# 制御パラメータ（デフォルト値）
SPEED = 50  # 移動速度
ROTATION_SPEED = 50  # 回転速度
INTERVAL = 0.05  # コマンド送信間隔

# 実行パラメータ
MAX_RUNTIME_SECONDS = 300  # 最大実行時間（秒）
RIGID_BODY_ID = 1  # 追跡するリジッドボディID
PID_GAIN = 20.0  # PIDゲイン（位置制御用）

# モーションキャプチャのリジッドボディID
RIGID_BODY_ID = 1  # モーションキャプチャで設定したリジッドボディID

# デバッグ出力の設定
DEBUG_LEVEL = 1  # 0=最小限, 1=通常, 2=詳細, 3=すべて

# メモリ管理の設定
GC_INTERVAL = 100  # 何秒ごとにガベージコレクションを実行するか

def debug_print(level, message):
    """
    デバッグレベルに応じて出力を制御する関数
    :param level: メッセージのデバッグレベル
    :param message: 出力するメッセージ
    """
    if DEBUG_LEVEL >= level and not stop_event.is_set():
        if level == 1:
            logging.warning(message)
        elif level == 2:
            logging.info(message)
        elif level >= 3:
            logging.debug(message)
        else:
            logging.info(message)

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

# receive_rigid_body_frame関数は削除され、mocap_stream.pyモジュールに移行されました


def initialize_mocap():
    """モーションキャプチャシステムの初期化と接続"""
    try:
        debug_print(1, "モーションキャプチャシステムへの接続を開始します...")
        
        # MOCAPサーバーのIPアドレス: 192.168.11.2
        server_ip = "192.168.11.2"
        # ローカルPCのIPアドレス: 192.168.11.13
        local_ip = "192.168.11.13"
        
        # mocap_streamモジュールを初期化
        success = mocap_stream.initialize(
            rigid_body_id=RIGID_BODY_ID,
            debug_level=DEBUG_LEVEL,
            server_ip=server_ip,
            local_ip=local_ip
        )
        
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
            # 接続状態を取得
            status = mocap_stream.get_connection_status()
            if status["packets"] > 0:
                packets = status["packets"]
                debug_print(1, f"モーションキャプチャデータの受信を確認しました ({packets} パケット)")
                return True
            time.sleep(0.1)
        
        # タイムアウト後もデータが受信されない場合
        status = mocap_stream.get_connection_status()
        if status["packets"] == 0:
            debug_print(1, "警告: モーションキャプチャデータが受信されませんでした。")
            debug_print(1, "リジッドボディの設定やネットワーク接続を確認してください。")
            # 接続自体は成功しているのでTrueを返す
            return True
        
        return True
        
    except Exception as e:
        debug_print(1, f"モーションキャプチャシステムの初期化中にエラーが発生しました: {e}")
        return False
        return False

def initialize_drone():
    """ドローンの初期化と接続"""
    global tello_manager, drone
    
    debug_print(1, "ドローンへの接続を開始します...")
    
    # TelloManagerを初期化
    debug_print(1, "TelloManagerを初期化しています...")
    
    # 他のスレッドがソケットを使用していないことを確認するために少し待機
    time.sleep(0.5)
    
    try:
        # ソケットリソース確保のための再試行ロジック
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                tello_manager = TelloManager()
                debug_print(1, "TelloManagerの初期化に成功しました")
                break
            except OSError as e:
                if attempt < max_attempts - 1:
                    debug_print(1, f"TelloManagerの初期化中にソケットエラーが発生しました: {e}. 再試行 {attempt+1}/{max_attempts}")
                    time.sleep(1)  # 再試行前に待機
                else:
                    debug_print(1, f"TelloManagerの初期化に失敗しました: {e}")
                    return False
    except Exception as e:
        debug_print(1, f"TelloManagerの初期化中に予期しないエラーが発生しました: {e}")
        return False
    
    # MOCAPデータのみのテストモードか確認
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--mocap-only', action='store_true', help='MOCAPデータのみでテストするモード')
    args, _ = parser.parse_known_args()
    
    if args.mocap_only:
        logging.info("MOCAPデータのみのテストモードで実行します。ドローン接続をスキップします。")
        return True
    
    # 1機のドローンを検索
    logging.info("1機のTelloドローンを検索しています...")
    
    # ドローン検索の再試行ロジック
    max_search_attempts = 2
    for search_attempt in range(max_search_attempts):
        try:
            tello_manager.find_available_tello(1)
            break
        except socket.timeout:
            if search_attempt < max_search_attempts - 1:
                logging.warning(f"ドローン検索中にタイムアウトが発生しました。再試行 {search_attempt+1}/{max_search_attempts}")
                time.sleep(1)  # 再試行前に待機
            else:
                logging.error("ドローン検索に失敗しました。タイムアウトが発生しました。")
        except Exception as e:
            logging.error(f"ドローン検索中にエラーが発生しました: {e}")
            if search_attempt < max_search_attempts - 1:
                logging.warning(f"再試行 {search_attempt+1}/{max_search_attempts}")
                time.sleep(1)  # 再試行前に待機
            else:
                logging.warning("MOCAPデータのみで続行します。")
                return True
    
    # 検出されたドローンを取得
    drones = tello_manager.get_tello_list()
    
    if len(drones) == 0:
        logging.warning("ドローンが見つかりません。MOCAPデータのみで続行します。")
        return True  # ドローンが見つからなくても続行するように変更
    
    # 1機目のドローンを使用
    drone = drones[0]
    
    # 接続確認
    logging.info("ドローンに接続しています...")
    
    # 接続の再試行ロジック
    max_connect_attempts = 3
    for connect_attempt in range(max_connect_attempts):
        try:
            # 接続コマンド送信
            result = drone.connect()
            if not result:
                if connect_attempt < max_connect_attempts - 1:
                    logging.warning(f"ドローン接続に失敗しました。再試行 {connect_attempt+1}/{max_connect_attempts}")
                    time.sleep(1)  # 再試行前に待機
                    continue
                else:
                    logging.error("ドローン接続に失敗しました。MOCAPデータのみで続行します。")
                    return True
            
            # バッテリー残量を確認
            battery = drone.get_battery()
            if battery is None or battery == 0:
                if connect_attempt < max_connect_attempts - 1:
                    logging.warning(f"バッテリー情報の取得に失敗しました。再試行 {connect_attempt+1}/{max_connect_attempts}")
                    time.sleep(1)  # 再試行前に待機
                    continue
            
            logging.info(f"ドローン (IP: {drone.tello_ip}) に接続しました。バッテリー残量: {battery}%")
            
            # バッテリー残量が少ない場合は警告
            if battery < 20:
                logging.warning(f"警告: バッテリー残量が少なくなっています ({battery}%)")
            
            # 接続成功
            break
            
        except socket.timeout:
            if connect_attempt < max_connect_attempts - 1:
                logging.warning(f"ドローン接続中にタイムアウトが発生しました。再試行 {connect_attempt+1}/{max_connect_attempts}")
                time.sleep(1)  # 再試行前に待機
            else:
                logging.error(f"ドローン接続に失敗しました: タイムアウト")
                logging.info("ドローン接続なしでMOCAPデータのみで続行します。")
                return True
        except Exception as e:
            if connect_attempt < max_connect_attempts - 1:
                logging.warning(f"ドローン接続中にエラーが発生しました: {e}. 再試行 {connect_attempt+1}/{max_connect_attempts}")
                time.sleep(1)  # 再試行前に待機
            else:
                logging.error(f"ドローン接続に失敗しました: {e}")
                logging.info("ドローン接続なしでMOCAPデータのみで続行します。")
                return True
    
    return True

def takeoff():
    """ドローンを離陸させる"""
    global is_flying
    if is_flying:
        print("既に飛行中です")
        return
        
    print("ドローンを離陸させます...")
    try:
        drone.takeoff()
        time.sleep(2)  # 安定化のための待機
        is_flying = True
        print("ドローンが離陸しました")
    except Exception as e:
        print(f"ドローンの離陸に失敗しました: {e}")

def land():
    """ドローンを着陸させる"""
    global is_flying
    if not is_flying:
        print("飛行していません")
        return
        
    # 着陸前にRCコマンドをリセット
    drone.send_rc_control(0, 0, 0, 0)
    
    print("ドローンを着陸させます...")
    try:
        drone.land()
        time.sleep(2)  # 安全のための待機
        is_flying = False
        print("ドローンが着陸しました")
    except Exception as e:
        print(f"ドローンの着陸に失敗しました: {e}")

def emergency_stop():
    """ドローンを緊急停止させる"""
    debug_print(1, "緊急停止します！")
    
    try:
        drone.emergency()
        debug_print(1, "ドローンに緊急停止コマンドを送信しました")
    except Exception as e:
        debug_print(1, f"緊急停止コマンドの送信に失敗しました: {e}")
    
    global is_flying
    is_flying = False

def get_keyboard_input():
    """キーボード入力を取得して移動値を返す"""
    lr, fb, ud, yv = 0, 0, 0, 0  # 左右, 前後, 上下, ヨー回転
    speed = SPEED
    
    # キー入力をチェック
    keys_to_check = ["a", "d", "w", "s", "UP", "DOWN", "LEFT", "RIGHT", "q", "e", "ESCAPE", "SPACE"]
    pressed_keys = []
    
    for key in keys_to_check:
        if kp.getKey(key):
            pressed_keys.append(key)
            debug_print(1, f"キー入力検出: {key}")
    
    if "q" in pressed_keys:
        debug_print(1, "Qキーが押されました。離陸を試みます。")
    
    # 左右移動 (左右キー)
    if "LEFT" in pressed_keys: lr = -speed
    elif "RIGHT" in pressed_keys: lr = speed
    
    # 前後移動 (上下キー)
    if "UP" in pressed_keys: fb = speed
    elif "DOWN" in pressed_keys: fb = -speed
    
    # 上下移動 (w/s)
    if "w" in pressed_keys: ud = speed
    elif "s" in pressed_keys: ud = -speed
    
    # 左右回転 (a/d)
    if "a" in pressed_keys: yv = -ROTATION_SPEED
    elif "d" in pressed_keys: yv = ROTATION_SPEED
    
    # 離陸と着陸
    if "q" in pressed_keys: takeoff()
    if "e" in pressed_keys: land()
    
    # 緊急停止
    if "ESCAPE" in pressed_keys:
        emergency_stop()
        global should_stop
        should_stop = True
        debug_print(1, "ESCキーが押されました。プログラムを終了します。")
        
    # 通常終了（スペースキー）
    if "SPACE" in pressed_keys:
        global stop_event
        stop_event.set()
        should_stop = True
        debug_print(1, "スペースキーが押されました。プログラムを正常終了します。")
    
    return [lr, fb, ud, yv]

def calculate_position_control():
    """
    現在位置と目標位置に基づいて位置制御値を計算
    :return: [左右速度, 前後速度, 上下速度, ヨー回転速度]
    """
    # ホバリング中でない場合は制御しない
    if not is_flying:
        return [0, 0, 0, 0]
    
    # mocap_streamから現在位置を取得
    current_position = mocap_stream.get_current_position()
    
    # 位置誤差を計算
    error_x = target_pos["x"] - current_position["x"]  # 前後方向
    error_y = target_pos["y"] - current_position["y"]  # 上下方向
    error_z = target_pos["z"] - current_position["z"]  # 左右方向
    
    # デバッグ出力
    if DEBUG_LEVEL >= 2:
        debug_print(2, f"現在位置: x={current_position['x']:.2f}, y={current_position['y']:.2f}, z={current_position['z']:.2f}")
        debug_print(2, f"目標位置: x={target_pos['x']:.2f}, y={target_pos['y']:.2f}, z={target_pos['z']:.2f}")
        debug_print(2, f"誤差: x={error_x:.2f}, y={error_y:.2f}, z={error_z:.2f}")
    
    # PID制御（ここでは簡単のためP制御のみ実装）
    fb = int(error_x * PID_GAIN)  # 前後速度
    ud = int(error_y * PID_GAIN)  # 上下速度
    lr = int(error_z * PID_GAIN)  # 左右速度
    
    # 速度を制限
    fb = max(-SPEED, min(SPEED, fb))
    ud = max(-SPEED, min(SPEED, ud))
    lr = max(-SPEED, min(SPEED, lr))
    
    # ヨー回転は0（一定方向を向いたまま）
    yv = 0
    
    return [lr, fb, ud, yv]

def control_drone():
    """メインのドローン制御ループ"""
    global should_stop
    
    logging.info("ドローン制御を開始します...")
    logging.info("Pygameウィンドウにフォーカスを当ててキーボード操作してください")
    
    last_key_press_time = time.time()
    error_count = 0
    max_consecutive_errors = 5
    
    while not should_stop:
        try:
            # キーボード入力を取得
            vals = get_keyboard_input()
            
            # キー入力がある場合はそれを優先
            if any(vals):
                last_key_press_time = time.time()
                try:
                    drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])
                    # エラーがなければカウンタをリセット
                    error_count = 0
                except socket.timeout:
                    error_count += 1
                    logging.warning(f"RCコマンド送信中にタイムアウトが発生しました (エラー {error_count}/{max_consecutive_errors})")
                    time.sleep(0.2)  # 少し待機
                except Exception as e:
                    error_count += 1
                    logging.warning(f"RCコマンド送信中にエラーが発生しました: {e} (エラー {error_count}/{max_consecutive_errors})")
            else:
                # 最後のキー入力から一定時間経過したら位置制御
                if time.time() - last_key_press_time > 1.0 and is_flying:
                    control_vals = calculate_position_control()
                    try:
                        drone.send_rc_control(control_vals[0], control_vals[1], control_vals[2], control_vals[3])
                        # エラーがなければカウンタをリセット
                        error_count = 0
                    except socket.timeout:
                        error_count += 1
                        logging.warning(f"位置制御コマンド送信中にタイムアウトが発生しました (エラー {error_count}/{max_consecutive_errors})")
                    except Exception as e:
                        error_count += 1
                        logging.warning(f"位置制御コマンド送信中にエラーが発生しました: {e} (エラー {error_count}/{max_consecutive_errors})")
            
            # 連続エラーが多すぎる場合は少し長めに待機して回復を試みる
            if error_count >= max_consecutive_errors:
                logging.error(f"連続エラーが{max_consecutive_errors}回発生しました。一時停止して回復を試みます。")
                time.sleep(2.0)  # 2秒間待機
                error_count = 0  # カウンタをリセット
            
            # 一定間隔で処理
            time.sleep(INTERVAL)
            
        except KeyboardInterrupt:
            logging.info("プログラムが中断されました")
            should_stop = True
        except socket.timeout:
            logging.warning("ソケットタイムアウトが発生しましたが、処理を継続します。")
            time.sleep(0.5)  # 少し待機
        except Exception as e:
            logging.error(f"エラーが発生しました: {e}")
            time.sleep(INTERVAL)

def analyze_data():
    """
    収集したデータの簡単な分析を行う関数
    """
    # mocap_streamからデータを取得
    data = mocap_stream.get_data_for_analysis()
    
    if not data or len(data["positions"]) == 0:
        debug_print(1, "分析するデータがありません")
        return
    
    # 位置データの統計情報
    position_buffer = data["positions"]
    error_buffer = data["errors"]
    received_packets = data["packets"]
    
    x_values = [pos["x"] for pos in position_buffer]
    y_values = [pos["y"] for pos in position_buffer]
    z_values = [pos["z"] for pos in position_buffer]
    
    # 平均位置
    avg_x = sum(x_values) / len(x_values)
    avg_y = sum(y_values) / len(y_values)
    avg_z = sum(z_values) / len(z_values)
    
    # 最小値と最大値
    min_x, max_x = min(x_values), max(x_values)
    min_y, max_y = min(y_values), max(y_values)
    min_z, max_z = min(z_values), max(z_values)
    
    # エラーの平均
    avg_error = sum(error_buffer) / len(error_buffer) if error_buffer else 0
    
    debug_print(1, "\nデータ分析結果:")
    debug_print(1, f"- 受信パケット数: {received_packets}")
    debug_print(1, f"- 保存されたデータフレーム数: {len(position_buffer)}")
    debug_print(1, f"- 平均位置: x={avg_x:.2f}, y={avg_y:.2f}, z={avg_z:.2f}")
    debug_print(1, f"- X範囲: {min_x:.2f} から {max_x:.2f}")
    debug_print(1, f"- Y範囲: {min_y:.2f} から {max_y:.2f}")
    debug_print(1, f"- Z範囲: {min_z:.2f} から {max_z:.2f}")
    debug_print(1, f"- 平均マーカーエラー: {avg_error:.6f}")
    
    # メモリ使用量の表示
    memory_usage = monitor_memory_usage()
    debug_print(1, f"- メモリ使用量: {memory_usage:.2f} MB")

def check_connection_status():
    """
    接続状態を定期的に確認するスレッド関数
    """
    last_check_packets = 0
    last_check_time = time.time()
    
    debug_print(2, "接続状態確認スレッドを開始しました")
    
    while not should_stop and not stop_event.is_set():
        # 10秒ごとに接続状態を確認
        # 1秒ごとにstop_eventをチェックして素早く終了できるようにする
        for _ in range(10):
            if stop_event.is_set():
                break
            time.sleep(1)
        
        # mocap_streamから接続状態を取得
        status = mocap_stream.get_connection_status()
        current_packets = status["packets"]
        last_packet_time = status["last_packet_time"]
        
        current_time = time.time()
        elapsed_time = current_time - last_check_time
        new_packets = current_packets - last_check_packets
        
        # パケット受信レートを計算
        rate = new_packets / elapsed_time if elapsed_time > 0 else 0
        
        debug_print(1, f"接続状態: 過去{elapsed_time:.1f}秒間に{new_packets}パケット受信 (レート: {rate:.2f}パケット/秒)")
        
        # 5秒以上パケットが受信されていない場合は警告
        if last_packet_time is not None and current_time - last_packet_time > 5:
            debug_print(1, f"警告: {current_time - last_packet_time:.1f}秒間データが受信されていません")
        
        # 状態更新
        last_check_packets = current_packets
        last_check_time = current_time
        last_check_time = current_time

def check_termination_conditions():
    """
    終了条件をチェックするスレッド関数
    """
    global should_stop, received_packets, start_time
    
    # 最大パケット数の設定
    MAX_PACKETS = 100000       # 最大パケット数
    
    debug_print(2, "終了条件チェックスレッドを開始しました")
    
    while not should_stop and not stop_event.is_set():
        # 少し待機してCPU負荷を下げる
        # 0.2秒ごとにstop_eventをチェックして素早く終了できるようにする
        for _ in range(5):
            if stop_event.is_set():
                break
            time.sleep(0.2)
        
        # 最大実行時間のチェック
        elapsed_time = time.time() - start_time
        if elapsed_time > MAX_RUNTIME_SECONDS:
            debug_print(1, f"\
最大実行時間 {MAX_RUNTIME_SECONDS} 秒に達しました。プログラムを終了します。")
            should_stop = True
            break
        
        # パケット数のチェックはもう必要ない
        # mocap_streamが内部で管理している
        
        # デバッグ情報を表示
        if DEBUG_LEVEL >= 3:
            # mocap_streamからパケット数を取得
            status = mocap_stream.get_connection_status()
            packets = status["packets"]
            debug_print(3, f"[THREAD] 終了条件チェック: 経過時間={elapsed_time:.1f}秒, 受信パケット数={packets}")

        
        # 定期的にメモリ使用量をチェック
        if DEBUG_LEVEL >= 2 and int(elapsed_time) % 30 == 0:  # 30秒ごと
            memory_usage = monitor_memory_usage()
            # mocap_streamからパケット数を取得
            status = mocap_stream.get_connection_status()
            packets = status["packets"]
            debug_print(2, f"\
経過時間: {int(elapsed_time)}秒, 受信パケット数: {packets}, メモリ使用量: {memory_usage:.2f} MB")
        
        # スレッドの負荷を下げるために少し待機
        time.sleep(1)

def graceful_shutdown():
    """スレッドを適切に終了させる関数"""
    global stop_event, threads
    
    debug_print(1, "スレッドの終了処理を開始します...")
    
    # 終了フラグを設定
    stop_event.set()
    
    # 各スレッドが終了するのを待つ
    for i, thread in enumerate(threads):
        if thread.is_alive():
            debug_print(2, f"スレッド {i+1}/{len(threads)} の終了を待っています...")
            thread.join(timeout=2.0)  # 最大2秒間待つ
            if thread.is_alive():
                debug_print(1, f"警告: スレッド {i+1} がタイムアウトしました")
    
    debug_print(1, "すべてのスレッドの終了処理が完了しました")

def cleanup():
    """リソースをクリーンアップする"""
    global tello_manager, drone, is_flying
    
    debug_print(1, "リソースをクリーンアップしています...")
    
    # スレッドの終了処理を行う
    graceful_shutdown()
    
    # モーションキャプチャシステムをシャットダウン
    mocap_stream.shutdown()
    
    # ドローンを着陸させる
    if is_flying and drone:
        debug_print(1, "ドローンを着陸させています...")
        drone.land()
    
    # TelloManagerのスレッドを終了
    if tello_manager:
        debug_print(1, "TelloManagerのスレッドを終了しています...")
        tello_manager.shutdown()

    # ドローンとの接続を閉じる
    if drone:
        try:
            drone.end()
            debug_print(1, f"ドローンとの接続を閉じました")
        except:
            pass
    
    # mocap_streamモジュールをシャットダウン
    try:
        mocap_stream.shutdown()
        debug_print(1, "モーションキャプチャシステムとの接続を閉じました")
    except Exception as e:
        debug_print(1, f"モーションキャプチャシステムのシャットダウン中にエラーが発生しました: {e}")
    
    # ソケットを閉じる
    if tello_manager:
        del tello_manager
    
    # 最終的なガベージコレクションを実行
    gc.collect()
    
    # 最終的なメモリ使用量を表示
    memory_usage = monitor_memory_usage()
    debug_print(1, f"最終メモリ使用量: {memory_usage:.2f} MB")
    
    debug_print(1, "クリーンアップが完了しました")

def main():
    """メイン関数"""
    global should_stop, start_time
    
    try:
        # 開始時間を記録
        start_time = time.time()
        
        debug_print(1, "Tello MOCAP Control - モーションキャプチャシステムを使用したTelloドローンの制御プログラム")
        debug_print(1, "-----------------------------------------------------------------------")
        debug_print(1, f"設定: リジッドボディID = {RIGID_BODY_ID}, 最大実行時間 = {MAX_RUNTIME_SECONDS}秒")
        
        # Pygameの初期化
        try:
            kp.init()
            debug_print(1, "Pygameを初期化しました")
            # ウィンドウが表示されていることを確認
            pygame.display.update()
            debug_print(1, "Pygameウィンドウを表示しました。このウィンドウにフォーカスを当ててキーボード入力を行ってください。")
        except Exception as e:
            debug_print(1, f"Pygameの初期化中にエラーが発生しました: {e}")
            return
        
        # 初期化順序を変更: ドローン→MOCAP
        # まずドローンの初期化
        debug_print(1, "ドローンの初期化を開始します...")
        drone_initialized = False
        drone_init_attempts = 0
        max_drone_init_attempts = 3
        
        while not drone_initialized and drone_init_attempts < max_drone_init_attempts:
            try:
                drone_initialized = initialize_drone()
                if not drone_initialized:
                    drone_init_attempts += 1
                    debug_print(1, f"ドローンの初期化に失敗しました。再試行 {drone_init_attempts}/{max_drone_init_attempts}")
                    time.sleep(1)  # 再試行前に少し待機
                else:
                    debug_print(1, "ドローンの初期化に成功しました")
            except Exception as e:
                drone_init_attempts += 1
                debug_print(1, f"ドローンの初期化中に例外が発生しました: {e}. 再試行 {drone_init_attempts}/{max_drone_init_attempts}")
                time.sleep(1)  # 再試行前に少し待機
        
        if not drone_initialized:
            debug_print(1, f"{max_drone_init_attempts}回の試行後もドローンの初期化に失敗しました")
        
        # ドローン初期化後、少し待機してからMOCAPシステムの初期化
        time.sleep(1)
        
        # モーションキャプチャシステムの初期化
        debug_print(1, "モーションキャプチャシステムの初期化を開始します...")
        mocap_initialized = False
        mocap_init_attempts = 0
        max_mocap_init_attempts = 3
        
        while not mocap_initialized and mocap_init_attempts < max_mocap_init_attempts:
            try:
                mocap_initialized = initialize_mocap()
                if not mocap_initialized:
                    mocap_init_attempts += 1
                    debug_print(1, f"モーションキャプチャシステムの初期化に失敗しました。再試行 {mocap_init_attempts}/{max_mocap_init_attempts}")
                    time.sleep(1)  # 再試行前に少し待機
                else:
                    debug_print(1, "モーションキャプチャシステムの初期化に成功しました")
            except Exception as e:
                mocap_init_attempts += 1
                debug_print(1, f"モーションキャプチャシステムの初期化中に例外が発生しました: {e}. 再試行 {mocap_init_attempts}/{max_mocap_init_attempts}")
                time.sleep(1)  # 再試行前に少し待機
        
        if not mocap_initialized:
            debug_print(1, f"{max_mocap_init_attempts}回の試行後もモーションキャプチャシステムの初期化に失敗しました")
        
        # 両方の初期化が失敗した場合は終了
        if not mocap_initialized and not drone_initialized:
            debug_print(1, "MOCAPとドローンの両方の初期化に失敗しました。プログラムを終了します。")
            return
            
        # 終了条件をチェックするスレッドを開始
        termination_thread = threading.Thread(target=check_termination_conditions)
        termination_thread.daemon = False  # 非デーモンスレッドに変更
        termination_thread.start()
        threads.append(termination_thread)  # スレッドリストに追加
        
        # 接続状態確認スレッドを開始
        if mocap_initialized:
            status_thread = threading.Thread(target=check_connection_status)
            status_thread.daemon = False  # 非デーモンスレッドに変更
            status_thread.start()
            threads.append(status_thread)  # スレッドリストに追加
            debug_print(1, "MOCAP接続状態確認スレッドを開始しました")
        
        # 初期設定情報を表示
        debug_print(1, f"デバッグレベル: {DEBUG_LEVEL}, バッファサイズ: {MAX_BUFFER_SIZE}")
        memory_usage = monitor_memory_usage()
        debug_print(1, f"初期メモリ使用量: {memory_usage:.2f} MB")
        
        debug_print(1, "プログラムを開始しました。ESCキーで終了します。")
        debug_print(1, "キー操作: Q=離陸, E=着陸, W/S=上昇/下降, 矢印キー=移動")
        
        # メインループ
        last_status_time = time.time()
        consecutive_errors = 0  # 連続エラーカウンタ
        max_consecutive_errors = 5  # 最大連続エラー数
        error_recovery_time = 0  # エラー回復時間
        
        
        while not should_stop:
            current_time = time.time()
            
            # エラー回復モードの場合は待機
            if error_recovery_time > 0:
                if current_time < error_recovery_time:
                    debug_print(1, "エラー回復モード: 通信を一時停止しています...")
                    time.sleep(1.0)  # 回復中は長めの間隔で待機
                    continue
                else:
                    debug_print(1, "エラー回復モード終了: 通常通信を再開します")
                    consecutive_errors = 0  # カウンタリセット
                    error_recovery_time = 0
            
            # 5秒ごとに状態を表示
            if current_time - last_status_time >= 5:
                last_status_time = current_time
                elapsed_time = current_time - start_time
                remaining_time = MAX_RUNTIME_SECONDS - elapsed_time
                
                if remaining_time > 0 and DEBUG_LEVEL >= 1:
                    try:
                        # mocap_streamからパケット数を取得
                        status = mocap_stream.get_connection_status()
                        packets = status["packets"]
                        debug_print(1, f"実行中: 経過時間 {elapsed_time:.1f}秒, 受信パケット数: {packets}, 連続エラー: {consecutive_errors}")
                        if is_flying:
                            # mocap_streamから現在位置を取得
                            current_position = mocap_stream.get_current_position()
                            debug_print(1, f"現在位置: x={current_position['x']:.2f}, y={current_position['y']:.2f}, z={current_position['z']:.2f}")
                    except Exception as e:
                        debug_print(1, f"状態表示中にエラーが発生しました: {e}")
            
            try:
                # キーボード入力の処理
                vals = get_keyboard_input()
                
                # ドローン制御ループ
                if is_flying and drone_initialized:
                    # キーボード入力に基づいてドローンを制御
                    lr, fb, ud, yv = vals
                    try:
                        drone.send_rc_control(lr, fb, ud, yv)
                        debug_print(3, f"RCコマンド送信: lr={lr}, fb={fb}, ud={ud}, yv={yv}")
                        consecutive_errors = 0  # 成功した場合はカウンタをリセット
                    except socket.timeout:
                        consecutive_errors += 1
                        debug_print(1, f"RCコマンド送信中にタイムアウトが発生しました。連続エラー: {consecutive_errors}/{max_consecutive_errors}")
                        if consecutive_errors >= max_consecutive_errors:
                            debug_print(1, f"連続{max_consecutive_errors}回のタイムアウトが発生しました。回復モードに入ります。")
                            error_recovery_time = current_time + 3.0  # 3秒間の回復時間
                    except Exception as e:
                        consecutive_errors += 1
                        debug_print(1, f"RCコマンド送信中にエラーが発生しました: {e}. 連続エラー: {consecutive_errors}/{max_consecutive_errors}")
                        if consecutive_errors >= max_consecutive_errors:
                            debug_print(1, f"連続{max_consecutive_errors}回のエラーが発生しました。回復モードに入ります。")
                            error_recovery_time = current_time + 3.0  # 3秒間の回復時間
            except socket.timeout:
                consecutive_errors += 1
                debug_print(1, f"ソケットタイムアウトが発生しました。連続エラー: {consecutive_errors}/{max_consecutive_errors}")
                if consecutive_errors >= max_consecutive_errors:
                    debug_print(1, f"連続{max_consecutive_errors}回のタイムアウトが発生しました。回復モードに入ります。")
                    error_recovery_time = current_time + 3.0  # 3秒間の回復時間
                time.sleep(0.5)  # 少し待機してから再試行
            except Exception as e:
                consecutive_errors += 1
                debug_print(1, f"メインループ内でエラーが発生しました: {e}. 連続エラー: {consecutive_errors}/{max_consecutive_errors}")
                if consecutive_errors >= max_consecutive_errors:
                    debug_print(1, f"連続{max_consecutive_errors}回のエラーが発生しました。回復モードに入ります。")
                    error_recovery_time = current_time + 3.0  # 3秒間の回復時間
            
            # 少し待機
            time.sleep(INTERVAL)
    
    except KeyboardInterrupt:
        debug_print(1, "プログラムが中断されました")
        should_stop = True
    
    except Exception as e:
        debug_print(1, f"重大なエラーが発生しました: {e}")
        should_stop = True
    
    finally:
        # リソースのクリーンアップ
        try:
            cleanup()
        except Exception as e:
            debug_print(1, f"クリーンアップ中にエラーが発生しました: {e}")
        
        # 実行統計を表示
        elapsed_time = time.time() - start_time
        debug_print(1, f"\n実行統計:")
        debug_print(1, f"- 実行時間: {elapsed_time:.2f}秒")
        debug_print(1, f"- 受信パケット数: {received_packets}")
        debug_print(1, f"- 平均パケットレート: {received_packets/elapsed_time if elapsed_time > 0 else 0:.2f}パケット/秒")
        
        debug_print(1, "プログラムを終了しました。")


if __name__ == "__main__":
    main()
