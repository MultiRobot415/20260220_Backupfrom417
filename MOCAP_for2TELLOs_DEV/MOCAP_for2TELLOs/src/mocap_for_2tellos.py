#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MOCAP for 2 TELLOs - 2機のTelloドローンをMOCAPでホバリング制御するメインプログラム

このプログラムは、モーションキャプチャシステムからの位置情報を使用して、
2機のTelloドローンを同時に指定位置にホバリングさせるためのメインプログラムです。

実行モード:
- 通信のみモード: python3 mocap_for_2tellos.py --mode=comm
- MOCAPのみモード: python3 mocap_for_2tellos.py --mode=mocap
- 統合テストモード: python3 mocap_for_2tellos.py --mode=full (またはオプションなし)

キー操作:
- Q: 離陸
- E: 着陸
- W/S: 上下移動
- A/D: 左右回転
- 矢印キー: 前後左右移動
- T: 目標位置に移動
- Z: 目標位置をリセット
- 1/2: ドローン選択（1=ドローン1, 2=ドローン2, その他=両方）
- ESC: 緊急停止
- SPACE: 正常終了

作成日: 2025-06-26
更新日: 2025-07-08 - モード切替機能追加
"""

import datetime
import glob
import logging

# ログ設定
log_format = '%(asctime)s - %(levelname)s - [MAIN] %(message)s'
if not logging.getLogger().handlers:
    logging.basicConfig(level=logging.DEBUG, format=log_format)
import os
import time
import argparse
import threading
import random
import sys
import json
import csv
import math
from pathlib import Path
from datetime import datetime
import keyboard_control as kp
from custom_tello import CustomTello, TelloManager
from position_control import PositionController

# MOCAPモジュールは必要な場合のみインポート
try:
    import mocap_stream as ms
    MOCAP_AVAILABLE = True
except ImportError:
    print("警告: MOCAPモジュールをインポートできませんでした。MOCAPモードは使用できません。")
    MOCAP_AVAILABLE = False

# グローバル変数
tello_manager = None  # TelloManagerオブジェクト
drones = []  # CustomTelloオブジェクトのリスト
controllers = []  # PositionControllerオブジェクトのリスト
is_flying = False  # 飛行状態
should_stop = False  # プログラム終了フラグ
control_mode = "manual"  # 制御モード ("manual" または "auto")
selected_drone = -1  # 選択中のドローン（-1=全て, 0=ドローン1, 1=ドローン2）
MOCAP_CONNECTED = False  # MOCAPシステムが正常に接続されているかどうか

# ログ用
log_data = {0: [], 1: []}  # 自動制御時の記録

# 実行モード
RUN_MODE = "full"  # デフォルトは統合テストモード
USE_MOCAP = True  # MOCAPを使用するかどうか
USE_DRONES = True  # ドローンを使用するかどうか

# リジッドボディの設定
rigid_body_ids = [1, 2]  # 各ドローンに対応するリジッドボディID

# 初期目標位置（x, y, z）
default_target_positions = [
    [0.5, 1.0, 0.0],  # ドローン1の目標位置
    [0.0, 1.0, 1.5]   # ドローン2の目標位置
]

# 現在の目標位置（実行時に更新される）
target_positions = default_target_positions.copy()  # 初期値はデフォルト値のコピー

# 制御パラメータ（デフォルト値）
SPEED = 50  # 移動速度（0-100）
ROTATION_SPEED = 50  # 回転速度（0-100）
CONTROL_INTERVAL = 0.05  # 制御コマンド送信間隔（秒）
KEEPALIVE_INTERVAL = 5.0  # キープアライブ間隔（秒）- Telloのタイムアウト（15秒）より短くする
DATA_REFRESH_RATE = 0.1  # 位置データ更新間隔（秒）
STATUS_DISPLAY_INTERVAL = 1.0  # ステータス表示間隔（秒）

# デバッグモードフラグ
debug_mode = False  # キーボード入力のレスポンス改善のため無効化

# ロギング設定を完全に無効化
import logging
logging.getLogger().setLevel(logging.CRITICAL)  # rootロガーをCRITICALに設定
for logger_name in ['', 'MAIN', 'natnet', 'mocap', 'tello', 'control']:
    logging.getLogger(logger_name).setLevel(logging.CRITICAL)  # 全ロガーをCRITICALに設定

# CSVファイル出力用の設定
import csv
import os
import time

# CSVログファイルの設定
CSV_LOG_DIR = "results"
# ディレクトリがなければ作成
if not os.path.exists(CSV_LOG_DIR):
    os.makedirs(CSV_LOG_DIR, exist_ok=True)

# データログファイルの生成
TIMESTAMP = time.strftime("%Y%m%d_%H%M%S")
CSV_LOG_FILE = os.path.join(CSV_LOG_DIR, f"drone_data_{TIMESTAMP}_observer.csv")
DEBUG_LOG_FILE = os.path.join(CSV_LOG_DIR, f"debug_{TIMESTAMP}.csv")

# CSVヘッダーの初期化
with open(CSV_LOG_FILE, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'drone_id', 'role', 'mode', 'x', 'y', 'z', 
                    'target_x', 'target_y', 'target_z', 'error_x', 'error_y', 'error_z', 
                    'rc_lr', 'rc_fb', 'rc_ud', 'rc_yaw', 'tilt_x', 'tilt_y',
                    'trust', 'obs_error_x', 'obs_error_y', 'obs_error_z',
                    'obs_state_x', 'obs_state_y', 'obs_state_z',
                    'battery', 'height', 'leader_status', 'exec_time'])

# デバッグ情報をCSVファイルに記録する関数
def csv_debug_log(category, message, data=None):
    with open(DEBUG_LOG_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        writer.writerow([timestamp, category, message, str(data) if data else ""])

# ドローンデータをCSVファイルに記録する関数
def csv_drone_log(drone_id, mode, position, target, error, rc_values, height, battery, start_time=None, quaternion=None):
    with open(CSV_LOG_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        x, y, z = position if position else (0, 0, 0)
        tx, ty, tz = target if target else (0, 0, 0)
        ex, ey, ez = error if error else (0, 0, 0)
        lr, fb, ud, yaw = rc_values if rc_values else (0, 0, 0, 0)
        
        # MOCAPからの傾斜角度を取得（quaternionから変換）
        tilt_x = 0.0
        tilt_y = 0.0
        if quaternion:
            # クォータニオンからオイラー角（ロール・ピッチ）に変換
            # シンプルな変換を使用（実際にはもっと複雑な計算が必要かも）
            q0, q1, q2, q3 = quaternion
            # ロール（x軸周り）の傾き
            tilt_x = math.degrees(math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)))
            # ピッチ（y軸周り）の傾き
            tilt_y = math.degrees(math.asin(2.0 * (q0 * q2 - q3 * q1)))
        
        # Default values for observer-related fields (will be populated in Phase 2)
        trust = 1.0  # Default trust level
        obs_error_x, obs_error_y, obs_error_z = 0.0, 0.0, 0.0
        obs_state_x, obs_state_y, obs_state_z = x, y, z  # Initially just use actual position
        
        # Determine drone role based on ID (0: leader, 1: follower)
        role = "leader" if drone_id == 1 else "follower"
        
        # Default leader status (0: normal)
        leader_status = 0
        
        # Calculate execution time if start_time is provided
        exec_time = 0.0
        if start_time:
            exec_time = (time.time() - start_time) * 1000  # in milliseconds
        
        # Write the full row with all new fields
        writer.writerow([
            timestamp, drone_id, role, mode, x, y, z, 
            tx, ty, tz, ex, ey, ez, 
            lr, fb, ud, yaw, tilt_x, tilt_y,
            trust, obs_error_x, obs_error_y, obs_error_z,
            obs_state_x, obs_state_y, obs_state_z,
            battery, height, leader_status, exec_time
        ])

# ログ記録設定
LOG_ENABLED = True  # ログ記録の有効/無効
LOG_DIRECTORY = 'results'  # ログファイルの保存先ディレクトリ
control_data_loggers = {}  # ドローンIDごとのロガー


def initialize_drones():
    """ドローンの初期化と接続"""
    global tello_manager, drones, controllers
    
    print("ドローンへの接続を開始します...")
    
    # TelloManagerを初期化
    print("TelloManagerを初期化しています...")
    tello_manager = TelloManager()
    
    # 2機のドローンを検索するか既知IPを直接使用する
    num_drones = 2
    print(f"{num_drones}機のTelloドローンを接続しています...")
    
    # 既存の検索メソッドを使用する
    # このメソッドは既知IPも考慮されている
    tello_manager.find_available_tello(num_drones)
    
    # 検出されたドローンを取得
    drones = tello_manager.get_tello_list()
    
    if len(drones) < num_drones:
        print(f"警告: {num_drones}機のドローンが必要ですが、{len(drones)}機しか見つかりませんでした")
        if len(drones) == 0:
            print("ドローンが見つかりません。プログラムを終了します。")
            return False
    
    # 検出されたドローンに接続
    print("ドローンに接続しています...")
    for i, tello in enumerate(drones):
        try:
            battery = tello.get_battery()
            print(f"ドローン {i+1} (IP: {tello.tello_ip}) に接続しました。バッテリー残量: {battery}%")
            
            # バッテリー残量が低い場合は警告
            if battery is not None and battery < 20:
                print(f"警告: ドローン {i+1} のバッテリー残量が低下しています ({battery}%)")
        except Exception as e:
            print(f"ドローン {i+1} (IP: {tello.tello_ip}) への接続に失敗しました: {e}")
    
    # 位置制御クラスを初期化
    for i in range(len(drones)):
        controller = PositionController()
        controller.set_target_position(*default_target_positions[i])
        controllers.append(controller)
    
    print(f"{len(drones)}機のドローンに接続しました。")
    return True


def initialize_mocap():
    """モーションキャプチャシステムの初期化"""
    global MOCAP_CONNECTED
    print("モーションキャプチャシステムに接続しています...")
    
    # MOCAPシステムに接続 (複数のRigidBody IDを指定しない方法で初期化)
    if not ms.initialize(debug_level=1):
        print("モーションキャプチャシステムに接続できませんでした")
        print("警告: MOCAPデータが利用できないため、自動ホバリングモード(Tモード)は使用できません")
        print("Manualモードでの操作は可能です")
        MOCAP_CONNECTED = False
        return True  # プログラムは継続する
    
    # 必要なRigidBody IDを登録
    for rb_id in rigid_body_ids:
        if debug_mode:
            print(f"DEBUG: RigidBody ID {rb_id} を登録します")
        # システムにリジッドボディの登録を行う処理があればここで行う
    
    # 接続状態を確認
    try:
        status = ms.get_connection_status()
        if not status["connected"]:
            print("モーションキャプチャシステムとの接続に失敗しました")
            print("警告: MOCAPデータが利用できないため、自動ホバリングモード(Tモード)は使用できません")
            print("Manualモードでの操作は可能です")
            MOCAP_CONNECTED = False
            return True  # プログラムは継続する
        
        print("モーションキャプチャシステムに接続しました")
        MOCAP_CONNECTED = True
        return True
    except Exception as e:
        print(f"モーションキャプチャシステムとの接続中にエラーが発生しました: {e}")
        print("警告: MOCAPデータが利用できないため、自動ホバリングモード(Tモード)は使用できません")
        print("Manualモードでの操作は可能です")
        MOCAP_CONNECTED = False
        return True  # プログラムは継続する


def get_drone_positions():
    """
    全てのドローンの現在位置を取得する
    
    Returns:
        list: 各ドローンの位置データのリスト。データが取得できないドローンはNone
    """
    positions = []
    
    # MOCAPモードが無効か未接続の場合はNoneのリストを返す
    if not USE_MOCAP or not MOCAP_CONNECTED:
        return [None] * len(drones)
    
    if debug_mode:
        print("DEBUG: get_drone_positionsのrigid_body_ids:", rigid_body_ids)
        print("DEBUG: dronesの長さ:", len(drones))
    
    try:
        for i, rb_id in enumerate(rigid_body_ids):
            if i >= len(drones):
                break
            
            try:
                if debug_mode:
                    print(f"DEBUG: RigidBody ID {rb_id} の位置データ取得試行")
                pos_data = ms.get_rigid_body_position(rb_id)
                if pos_data is not None:
                    # MOCAPからの位置データ形式をリストに変換
                    pos = [pos_data["x"], pos_data["y"], pos_data["z"]]
                    positions.append(pos)
                    if debug_mode:
                        print(f"DEBUG: RigidBody ID {rb_id} の位置データ取得成功: {pos}")
                else:
                    if debug_mode:
                        print(f"DEBUG: RigidBody ID {rb_id} の位置データがNoneです")
                    positions.append(None)
            except Exception as e:
                print(f"RigidBody {rb_id} の位置取得エラー: {e}")
                positions.append(None)
    except Exception as e:
        print(f"MOCAP位置取得中にエラーが発生しました: {e}")
        # エラー時は全ドローンに対してNoneを返す
        return [None] * len(drones)
        
    # 返却前にリスト長を確認し、足りない場合はNoneで補完
    while len(positions) < len(drones):
        positions.append(None)
    
    if debug_mode:
        print("DEBUG: 最終positions:", positions)
        
    return positions


def get_drone_rotations():
    """
    全てのドローンの現在の回転情報を取得する
    
    Returns:
        list: 各ドローンの回転データのリスト。データが取得できないドローンはNone
    """
    rotations = []
    
    # MOCAPモードが無効か未接続の場合はNoneのリストを返す
    if not USE_MOCAP or not MOCAP_CONNECTED:
        return [None] * len(drones)
    
    if debug_mode:
        print("DEBUG: get_drone_rotationsのrigid_body_ids:", rigid_body_ids)
        print("DEBUG: dronesの長さ:", len(drones))
    
    try:
        for i, rb_id in enumerate(rigid_body_ids):
            if i >= len(drones):
                break
            
            try:
                if debug_mode:
                    print(f"DEBUG: RigidBody ID {rb_id} の回転データ取得試行")
                # MOCAPからの回転データを取得
                # ID=1の場合はデフォルト値（rigid_body_id=None）を使用
                if rb_id == 1:
                    rot_data = ms.get_current_rotation()
                    if debug_mode:
                        print(f"DEBUG: ID={rb_id}はデフォルト値を使用")
                else:
                    rot_data = ms.get_current_rotation(rb_id)
                if rot_data is not None:
                    rotations.append(rot_data)
                    if debug_mode:
                        yaw = 0
                        if "controllers" in globals() and len(controllers) > i:
                            yaw = controllers[i].quaternion_to_yaw(rot_data)
                        print(f"DEBUG: RigidBody ID {rb_id} の回転データ取得成功: ヨー角={yaw:.1f}°")
                else:
                    if debug_mode:
                        print(f"DEBUG: RigidBody ID {rb_id} の回転データがNoneです")
                    rotations.append(None)
            except Exception as e:
                print(f"RigidBody {rb_id} の回転データ取得エラー: {e}")
                rotations.append(None)
    except Exception as e:
        print(f"MOCAP回転データ取得中にエラーが発生しました: {e}")
        # エラー時は全ドローンに対してNoneを返す
        return [None] * len(drones)
        
    # 返却前にリスト長を確認し、足りない場合はNoneで補完
    while len(rotations) < len(drones):
        rotations.append(None)
    
    if debug_mode:
        print("DEBUG: 最終rotations長さ:", len(rotations))
        
    return rotations


def display_status():
    """
    ドローンとMOCAPシステムの状態を表示する
    """
    print(f"\n=== ステータス表示 ({RUN_MODE}モード) ===")
    
    # MOCAP接続状態を表示（MOCAPモードかつ接続済みの場合のみ）
    if USE_MOCAP and MOCAP_CONNECTED:
        try:
            mocap_status = ms.get_connection_status()
            print(f"MOCAP接続状態: {mocap_status['connected']}, パケット数: {mocap_status['packets']}")
        except Exception as e:
            print(f"MOCAP状態取得エラー: {e}")
            print("MOCAP接続状態: 未接続")
    elif USE_MOCAP and not MOCAP_CONNECTED:
        print("MOCAP接続状態: 未接続 (手動モードのみ利用可能)")
    
    # ドローンの状態を表示（ドローンモードの場合のみ）
    if USE_DRONES and drones:
        for i, tello in enumerate(drones):
            try:
                battery = tello.get_battery()
                print(f"ドローン {i+1} (IP: {tello.tello_ip}): バッテリー残量 {battery}%")
            except Exception as e:
                print(f"ドローン {i+1} (IP: {tello.tello_ip}): 状態取得失敗 ({e})")
        
        # 制御モードと選択中のドローンを表示
        drone_str = "全て" if selected_drone == -1 else f"ドローン {selected_drone+1}"
        print(f"制御モード: {control_mode}, 選択中のドローン: {drone_str}")
    
    # 現在位置と目標位置を表示（MOCAPモードかつ接続済みの場合のみ）
    if USE_MOCAP and MOCAP_CONNECTED and controllers:
        try:
            positions = get_drone_positions()
            for i, pos in enumerate(positions):
                if i >= len(controllers):
                    break
                    
                target_pos = controllers[i].get_target_position()
                
                if pos is not None:
                    try:
                        error_dist = controllers[i].calculate_error_distance(pos)
                        
                        # 回転データの取得を試みる
                        rot_data = None
                        yaw_deg = "N/A"
                        try:
                            rot_data = ms.get_current_rotation(rigid_body_ids[i])
                            if rot_data is not None:
                                yaw_deg = f"{controllers[i].quaternion_to_yaw(rot_data):.1f}°"
                        except Exception:
                            pass
                            
                        print(f"ドローン {i+1}: Pos [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] | "
                              f"Target [{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}] | "
                              f"Yaw {yaw_deg} | Err {error_dist:.2f} m")
                    except Exception as e:
                        print(f"ドローン {i+1}: 位置計算エラー: {e}")
                else:
                    print(f"ドローン {i+1}: MOCAP DATA UNAVAILABLE (RigidBody ID {rigid_body_ids[i]})")
        except Exception as e:
            print(f"MOCAP位置表示エラー: {e}")
    elif USE_MOCAP and not MOCAP_CONNECTED:
        print("MOCAP未接続のため位置データは利用できません（手動モードのみ利用可能）")
    
    # 実行モードに関する情報を表示
    print(f"実行モード: {RUN_MODE}, MOCAP: {USE_MOCAP}, ドローン: {USE_DRONES}")
    print("終了するにはSPACEキーを押してください")


def initialize_controllers():
    """コントローラーの初期化"""
    global controllers
    for i in range(len(drones)):
        controller = PositionController()
        controller.set_gains(gain_x=0.4, gain_y=0.4, gain_z=0.4, gain_yaw=0.3)
        controllers.append(controller)
    print(f"全{len(controllers)}機のコントローラーを初期化しました")


def initialize_loggers():
    """ログ記録用のロガーを初期化する"""
    global control_data_loggers
    
    if not LOG_ENABLED:
        print("ログ記録は無効化されています")
        return
    
    # ログディレクトリ作成
    os.makedirs(LOG_DIRECTORY, exist_ok=True)
    
    # 現在日時を取得してファイル名に使用
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    for i in range(len(drones)):
        drone_id = i + 1
        log_file_path = os.path.join(LOG_DIRECTORY, f"tmode_control_drone{drone_id}_{timestamp}.csv")
        
        # CSVファイルを作成してヘッダーを書き込む
        with open(log_file_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([
                'timestamp', 'mode', 
                'mocap_x', 'mocap_y', 'mocap_z',
                'target_x', 'target_y', 'target_z',
                'error_x', 'error_y', 'error_z',
                'rc_lr', 'rc_fb', 'rc_ud', 'rc_yaw',
                'height', 'battery'
            ])
        
        control_data_loggers[drone_id] = {
            'path': log_file_path,
            'last_log_time': time.time()
        }
    
    print(f"全{len(drones)}機のログファイルを作成しました")


def log_control_data(drone_index, mode, mocap_position, target_position, error, rc_values, height, battery, start_time=None, quaternion=None):
    """
    制御データをCSVログに記録する
    
    Args:
        drone_index: ドローンのインデックス
        mode: 現在のモード ("手動", "自動", etc.)
        mocap_position: MOCAPから取得した現在位置 [x, y, z]
        target_position: 目標位置 [x, y, z]
        error: 位置誤差 [ex, ey, ez]
        rc_values: RC制御値 [lr, fb, ud, yaw]
        height: 高度(cm)
        battery: バッテリー値(%)
        start_time: 制御ループ開始時間 (パフォーマンス計測用)
        quaternion: MOCAPから取得した回転情報（傾斜角計算用）
    """
    # パフォーマンス向上のため新しいCSVロギングを使用
    drone_id = drone_index + 1
    
    # CSVログ記録を実行
    csv_drone_log(drone_id, mode, mocap_position, target_position, error, rc_values, height, battery, start_time, quaternion)
    if not LOG_ENABLED:
        return
    
    drone_id = drone_index + 1
    if drone_id not in control_data_loggers:
        return
    
    current_time = time.time()
    logger_info = control_data_loggers[drone_id]
    
    # ファイルにデータを追記
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    
    # MOCAP位置が存在しない場合はデフォルト値を設定
    if mocap_position is None:
        mocap_position = [0.0, 0.0, 0.0]
    
    # 目標位置が存在しない場合はデフォルト値を設定
    if target_position is None:
        target_position = [0.0, 0.0, 0.0]
    
    # 誤差が存在しない場合はデフォルト値を設定
    if error is None:
        error = [0.0, 0.0, 0.0]
    
    # RC値が存在しない場合はデフォルト値を設定
    if rc_values is None:
        rc_values = [0, 0, 0, 0]
    
    try:
        with open(logger_info['path'], 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([
                timestamp, mode,
                mocap_position[0], mocap_position[1], mocap_position[2],
                target_position[0], target_position[1], target_position[2],
                error[0], error[1], error[2],
                rc_values[0], rc_values[1], rc_values[2], rc_values[3],
                height, battery
            ])
        
        logger_info['last_log_time'] = current_time
    except Exception as e:
        print(f"ドローン {drone_id} のログ記録中にエラー発生: {e}")


def status_display_thread():
    """
    定期的にステータス情報を表示するスレッド
    """
    global should_stop
    
    last_display_time = 0
    
    while not should_stop:
        # 一定間隔でステータスを表示
        current_time = time.time()
        if current_time - last_display_time >= STATUS_DISPLAY_INTERVAL:
            display_status()
            last_display_time = current_time
        
        time.sleep(0.5)


def takeoff_all():
    """全てのドローンを同時に離陸させる"""
    global is_flying
    
    if is_flying:
        print("既に飛行中です")
        return
    
    print("全てのドローンを同時に離陸させます...")
    print(f"ドローン数: {len(drones)}")
    
    # 離陸操作を試みる
    threads = []
    for i, tello in enumerate(drones):
        thread = threading.Thread(target=lambda t=tello, idx=i: takeoff_drone(t, idx))
        threads.append(thread)
        thread.start()
    
    for thread in threads:
        thread.join()
    
    # 離陸後にRCコマンドをリセット
    for tello in drones:
        tello.send_rc_control(0, 0, 0, 0)
    
    time.sleep(2)  # 安定化のための待機
    is_flying = True
    print("全てのドローンが離陸しました")


def takeoff_drone(tello, index):
    """個別のドローンを離陸させる（スレッド用）"""
    try:
        print(f"ドローン {index+1} (IP: {tello.tello_ip}) を離陸させています...")
        result = tello.takeoff()
        print(f"ドローン {index+1} 離陸コマンド結果: {result}")
    except Exception as e:
        print(f"ドローン {index+1} の離陸に失敗しました: {e}")
        import traceback
        traceback.print_exc()


def land_all():
    """全てのドローンを同時に着陸させる"""
    global is_flying
    
    if not is_flying:
        print("飛行していません")
        return
    
    # 着陸前にRCコマンドをリセット
    for tello in drones:
        tello.send_rc_control(0, 0, 0, 0)
    
    print("全てのドローンを同時に着陸させます...")
    
    # 着陸操作を試みる
    threads = []
    for i, tello in enumerate(drones):
        thread = threading.Thread(target=lambda t=tello, idx=i: land_drone(t, idx))
        threads.append(thread)
        thread.start()
    
    for thread in threads:
        thread.join()
    
    time.sleep(2)  # 安全のための待機
    is_flying = False
    print("全てのドローンが着陸しました")


def land_drone(tello, index):
    """個別のドローンを着陸させる（スレッド用）"""
    try:
        print(f"ドローン {index+1} を着陸させています...")
        tello.land()
    except Exception as e:
        print(f"ドローン {index+1} の着陸に失敗しました: {e}")


def emergency_stop(reason=None):
    """
    全てのドローンを緊急停止させる
    Args:
        reason (str, optional): 緊急停止の理由。Noneの場合は「ユーザー操作」と表示
    """
    global is_flying, control_mode, should_stop

    emergency_reason = reason if reason else "ユーザー操作"
    print(f" 緊急停止します！理由: {emergency_reason} ")

    # 緊急停止コマンド送信を試行（複数回）
    max_attempts = 3

    for i, tello in enumerate(drones):
        success = False

        # 複数回試行
        for attempt in range(max_attempts):
            try:
                # 緊急停止コマンド送信
                tello.emergency()
                print(f"ドローン {i+1} に緊急停止コマンド送信成功 (試行 {attempt+1}/{max_attempts})")
                success = True
                break  # 成功したらループ終了
            except Exception as e:
                print(f"ドローン {i+1} への緊急停止コマンド送信失敗 (試行 {attempt+1}/{max_attempts}): {e}")
                time.sleep(0.1)  # 短い待機時間

        if not success:
            print(f" ドローン {i+1} への緊急停止コマンド送信が全て失敗しました。物理的な対応が必要な可能性があります。 ")

        # 状態をリセット
        is_flying[i] = False

    # 自動制御モードを無効化
    control_mode = "manual"

    # ログに記録
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        # resultsディレクトリがない場合は作成
        os.makedirs(LOG_DIRECTORY, exist_ok=True)
        with open(os.path.join(LOG_DIRECTORY, "emergency_log.txt"), "a") as log_file:
            log_file.write(f"{timestamp}: 緊急停止発生 - 理由: {emergency_reason}\n")
    except Exception as log_error:
        print(f"緊急停止ログの記録に失敗: {log_error}")

    # 必要に応じてプログラムも停止
    if reason and "クリティカル" in reason:
        print("クリティカルエラーのため、プログラムを停止します")
        should_stop = True
    should_stop = True


def get_drone_positions():
    """
    すべてのドローンのMOCAP位置を取得する。
    型安全性強化版: 文字列の自動変換、None処理の改善を行い、常に正しい型の値を返す。
    
    Returns:
        list: 各ドローンの位置データのリスト [[x, y, z], [x, y, z], ...]
              取得失敗時は該当ドローンが None となる
    """
    global rigid_body_ids, debug_mode
    
    # MOCAPが無効な場合は空のリストを返す
    if not USE_MOCAP or not ms:
        return [None] * len(rigid_body_ids)

    positions = []
    try:
        # デバッグ表示
        if debug_mode:
            print("DEBUG: get_drone_positionsのrigid_body_ids:", rigid_body_ids)

        for i, rb_id in enumerate(rigid_body_ids):
            try:
                # 位置データ取得 - ID=1の場合は明示的にNoneを指定してデフォルト値を使用
                if debug_mode:
                    print(f"DEBUG: ID={rb_id}の位置データを取得中...")

                # ID=1の場合はデフォルト値（rigid_body_id=None）を使用
                if rb_id == 1:
                    pos_data = ms.get_current_position()
                    if debug_mode:
                        print(f"DEBUG: ID={rb_id}はデフォルト値を使用")
                else:
                    pos_data = ms.get_current_position(rigid_body_id=rb_id)

                # 取得結果をチェック
                if pos_data is not None:
                    try:
                        if debug_mode:
                            print(f"DEBUG: ID={rb_id}の位置データを取得: {pos_data}")
                        pos = [pos_data["x"], pos_data["y"], pos_data["z"]]
                        positions.append(pos)
                        
                        # 位置データを表示（デバッグ用）
                        if debug_mode:
                            pos_str = ', '.join([f"{coord:.2f}" for coord in pos])
                            print(f"MOCAP位置データ: ID={rb_id}, 位置=[{pos_str}]")
                    except Exception as conv_e:
                        print(f"ID={rb_id}の位置データ変換中にエラー発生: {conv_e}")
                        positions.append(None)
                else:
                    if debug_mode:
                        print(f"DEBUG: ID={rb_id}の位置データがNoneです")
                    positions.append(None)
            except Exception as inner_e:
                print(f"ID={rb_id}の位置データ取得中にエラー発生: {inner_e}")
                positions.append(None)
        
        # 結果をデバッグ出力
        if debug_mode:
            print("DEBUG: get_drone_positionsの結果:", positions)
    except Exception as outer_e:
        print(f"位置データ取得プロセスでエラー発生: {outer_e}")
        return [None] * len(rigid_body_ids)
    
    # 位置データの長さを確認
    if len(positions) != len(rigid_body_ids):
        print(f"警告: 位置データ数({len(positions)})がrigid_body_ids数({len(rigid_body_ids)})と一致しません")
        # 不足分はNoneで補完
        while len(positions) < len(rigid_body_ids):
            positions.append(None)
    
    return positions


def process_keyboard_input():
    """
    キーボード入力を処理する
    
    Returns:
        list: 手動モード時の制御値 [lr, fb, ud, yv] 
    """
    global is_flying, should_stop, control_mode, selected_drone, target_positions, last_key_process_time
    
    # キーボード入力を取得（デバッグ出力なし - 応答性向上のため）
    pressed_keys = kp.get_pressed_keys()
    
    # キー入力のデバウンス処理（連続検出防止）
    current_time = time.time()
    # 前回のキー処理から50ms以内の場合は処理をスキップ
    if hasattr(process_keyboard_input, 'last_time') and current_time - process_keyboard_input.last_time < 0.05:
        if control_mode == "manual":
            # 手動制御値のみ継続して返す（他のキー処理はスキップ）
            return process_keyboard_input.last_values
    
    # 今回の処理時間を記録
    process_keyboard_input.last_time = current_time
    
    # デフォルトの制御値（静止）
    lr, fb, ud, yv = 0, 0, 0, 0
    
    # 手動制御モードの場合の移動値を計算
    if control_mode == "manual":
        # 左右移動 (左右キー)
        if "LEFT" in pressed_keys: lr = -SPEED
        elif "RIGHT" in pressed_keys: lr = SPEED
        
        # 前後移動 (上下キー)
        if "UP" in pressed_keys: fb = SPEED
        elif "DOWN" in pressed_keys: fb = -SPEED
        
        # 上下移動 (w/s)
        if "w" in pressed_keys: ud = SPEED
        elif "s" in pressed_keys: ud = -SPEED
        
        # 左右回転 (a/d)
        if "a" in pressed_keys: yv = -ROTATION_SPEED
        elif "d" in pressed_keys: yv = ROTATION_SPEED
    
    # 特殊機能キー
    if "q" in pressed_keys:
        # time.sleep()は削除 - キー入力のブロッキングを防止
        if not hasattr(process_keyboard_input, 'q_last_time') or \
           current_time - process_keyboard_input.q_last_time > 1.0:
            print("Qキーが押されました - 離陸を開始します")
            process_keyboard_input.q_last_time = current_time
            # スレッドでtakeoff_allを実行
            threading.Thread(target=takeoff_all).start()
    
    if "e" in pressed_keys: 
        land_all()
    
    if "t" in pressed_keys:
        # Tキーによるモード切替え処理
        # 自動モードから手動モードへの切り替えは常に許可
        if control_mode == "auto":
            control_mode = "manual"
            print("手動制御モードに切り替えました")
            # 手動モードに戻る場合は目標位置をリセット
            reset_target_positions()
        else:
            # 手動モードから自動モードへは、MOCAP接続が必要
            if not MOCAP_CONNECTED:
                print("警告: MOCAPデータが利用できないため、自動ホバリングモード(Tモード)は使用できません")
                print("Manualモードでの操作は可能です")
            else:
                control_mode = "auto"
                print("自動ホバリングモードに切り替えました")
                
                # 自動モードでは、現在位置+少し上を目標位置に設定
                # get_drone_positions()のブロッキングを防ぐためスレッド化
                def set_targets_thread():
                    positions = get_drone_positions()
                    for i, pos in enumerate(positions):
                        if pos is not None:
                            # 現在位置から高さ+10cmを目標位置に設定
                            target_positions[i] = [pos[0], pos[1], pos[2] + 10]
                            csv_debug_log("target_set", f"ドローン{i+1}", target_positions[i])
                        else:
                            csv_debug_log("target_error", f"ドローン{i+1}", "位置データ取得不可")
                            # 位置データがない場合、現在の目標位置を維持
                
                # 非同期でターゲット設定を実行
                threading.Thread(target=set_targets_thread).start()
    
    if "m" in pressed_keys:
        control_mode = "manual"
        print("手動制御モードに切り替えました")
    
    # ドローン選択キー
    if "1" in pressed_keys:
        selected_drone = 0
        print("ドローン1を選択しました")
    elif "2" in pressed_keys:
        selected_drone = 1
        print("ドローン2を選択しました")
    elif "0" in pressed_keys:
        selected_drone = -1
        print("全てのドローンを選択しました")
    
    # 目標位置リセット
    if "z" in pressed_keys:
        for i, controller in enumerate(controllers):
            controller.set_target_position(*default_target_positions[i])
        print("目標位置をリセットしました")
    
    # 緊急停止
    if "ESCAPE" in pressed_keys:
        emergency_stop()
    
    # 通常終了
    if "SPACE" in pressed_keys:
        should_stop = True
        print("SPACEキーが押されました。プログラムを正常終了します。")
    
    # キー入力の状態を記録（デバウンス用）
    process_keyboard_input.last_values = [lr, fb, ud, yv]
    if pressed_keys:
        print(f"キー入力検出: {', '.join(pressed_keys)}")
    
    return [lr, fb, ud, yv]


def control_drones_thread():
    """
    ドローン制御のメインループ（スレッド用）
    """
    global is_flying, should_stop, control_mode
    
    print(f"ドローン制御スレッドを開始しました ({RUN_MODE}モード)")
    
    # ドローンが使用されていない場合はキー入力処理のみを行う
    if not USE_DRONES:
        while not should_stop:
            process_keyboard_input()
            time.sleep(0.1)
        return
    
    last_command_time = time.time()
    last_keepalive_time = time.time()
    
    while not should_stop:
        try:
            current_time = time.time()
            
            # コマンド送信間隔を調整
            if current_time - last_command_time < CONTROL_INTERVAL:
                time.sleep(0.01)
                continue
                
            # キープアライブ - Telloのタイムアウト対策（15秒以内）
            if is_flying and current_time - last_keepalive_time > KEEPALIVE_INTERVAL:
                print("キープアライブ信号を送信しています...")
                for i, tello in enumerate(drones):
                    try:
                        # キープアライブとして現在の高度を取得
                        height = tello.get_height()
                        print(f"ドローン {i+1} の高度: {height}cm")
                    except Exception as e:
                        print(f"ドローン {i+1} のキープアライブ失敗: {e}")
                last_keepalive_time = current_time
            
            # キーボード入力を処理
            manual_controls = process_keyboard_input()
            
            # 現在の位置データと回転データを取得（自動モード用）
            positions = [None] * len(drones)
            rotations = [None] * len(drones)
            if control_mode == "auto":
                try:
                    positions = get_drone_positions()
                    rotations = get_drone_rotations()
                except Exception as pos_error:
                    print(f"位置データ取得エラー: {pos_error}")
                    # エラー時は全てNoneとして次の処理へ
                    positions = [None] * len(drones)
                    rotations = [None] * len(drones)
            
            # 飛行中の場合のみコマンドを送信
            if is_flying:
                # 全てのドローンに対して処理
                for i, tello in enumerate(drones):
                    # 選択されたドローンのみ制御（-1は全て選択）
                    if selected_drone != -1 and selected_drone != i:
                        continue
                    
                    # データ異常監視用変数
                    position_data_missing_count = getattr(tello, "_position_data_missing_count", 0)
                    control_anomaly_count = getattr(tello, "_control_anomaly_count", 0)
                    last_position_time = getattr(tello, "_last_position_time", 0)
                    position_timeout = 2.0  # 位置データのタイムアウト秒数
                    
                    # 自動モードの場合は位置制御を使用
                    if control_mode == "auto":
                        # 位置データが存在するか確認
                        if i < len(positions) and positions[i] is not None:
                            # 前回からの経過時間をチェック
                            position_time_diff = time.time() - last_position_time if last_position_time > 0 else 0
                            
                            # 位置データ取得成功、カウンターリセット
                            position_data_missing_count = 0
                            setattr(tello, "_last_position_time", time.time())
                            
                            # 制御ループ開始時間を記録（実行時間計測用）
                            loop_start_time = time.time()
                            
                            # 位置制御アルゴリズムを使用して制御値を計算
                            try:
                                # 回転情報があれば利用、なければNoneを渡す
                                quaternion = rotations[i] if i < len(rotations) else None
                                
                                # 位置制御計算
                                control_values = controllers[i].calculate_control(positions[i], quaternion=quaternion)
                                
                                # 正常な制御計算成功、カウンターリセット
                                control_anomaly_count = 0
                            except Exception as calc_error:
                                print(f"ドローン{i+1} 制御値計算エラー: {calc_error} - ホバリング維持")
                                control_values = [0, 0, 0, 0]  # エラー時はホバリング
                                
                                # 制御計算エラー回数をカウント
                                control_anomaly_count += 1
                            
                            # 位置誤差を計算
                            try:
                                error_vec = [controllers[i].target_position[j] - positions[i][j] for j in range(3)]
                                
                                # 誤差が大きすぎる場合は警告
                                max_error = max([abs(e) for e in error_vec])
                                if max_error > 2.0:  # 2m以上の誤差は異常と判断
                                    print(f"警告: ドローン{i+1} の目標位置誤差が大きいです: {max_error:.2f}m")
                            except Exception as err_calc_error:
                                print(f"誤差計算エラー: {err_calc_error}")
                                error_vec = [0, 0, 0]  # エラー時はゼロベクトル
                            
                            # 高度とバッテリー情報を取得
                            height = 0
                            battery = 0
                            try:
                                height = tello.get_height()
                                battery = tello.get_battery()
                                
                                # バッテリー残量警告
                                if 0 < battery < 15:  # 15%以下の場合は警告
                                    print(f"警告: ドローン{i+1} のバッテリー残量が低下しています: {battery}%")
                                    
                                # バッテリーが極端に低い場合は緊急着陸を検討
                                if 0 < battery < 10:
                                    print(f"警告: ドローン{i+1} のバッテリーが危険水準です: {battery}%")
                            except Exception as e:
                                print(f"ドローン{i+1}の状態取得エラー: {e}")
                                # heightとbatteryは既に0で初期化済み
                            
                            # CSVログに記録
                            log_control_data(
                                drone_index=i,
                                mode="自動",
                                mocap_position=positions[i],
                                target_position=controllers[i].target_position,
                                error=error_vec,
                                rc_values=control_values,
                                height=height,
                                battery=battery,
                                start_time=loop_start_time,
                                quaternion=rotations[i] if i < len(rotations) else None
                            )
                            
                            # デバッグログ用にデータを生成
                            if debug_mode:
                                log_entry = {
                                    "timestamp": time.time(),
                                    "drone_index": i,
                                    "position": positions[i],
                                    "target": controllers[i].target_position,
                                    "error": error_vec,
                                    "control": control_values,
                                    "height": height,
                                    "battery": battery
                                }
                                print(json.dumps(log_entry, indent=2))
                            
                            # 制御値の安全性チェックと制限
                            safe_controls = []
                            has_invalid_control = False
                            
                            for val in control_values:
                                # 無効な値のチェック (NaN, 無限大, 非数値)
                                if not isinstance(val, (int, float)) or val != val or abs(val) == float('inf') or abs(val) > 100:
                                    safe_val = 0
                                    has_invalid_control = True
                                    print(f"警告: ドローン{i+1} の無効な制御値 {val} を検出。0に置き換えました。")
                                else:
                                    # 絶対値が大きい場合は制限する
                                    safe_val = max(min(int(val), 80), -80)  # 安全のために制限値を設定
                                
                                safe_controls.append(safe_val)
                            
                            # 無効な制御値が検出された場合はカウントを増やす
                            if has_invalid_control:
                                control_anomaly_count += 1
                                
                            # 異常検出による緊急停止条件
                            if control_anomaly_count >= 5:
                                print(f"警告: ドローン{i+1} の制御異常が連続しています。緊急停止を検討してください。")
                                
                                # 5回以上異常が続く場合は安全対策を実施
                                if control_anomaly_count >= 10:
                                    print(f"ドローン{i+1} の制御異常が長時間続いています。自動モードを無効化します。")
                                    control_mode = "manual"  # 異常が続く場合は自動制御を停止
                            
                            # 制御値を送信
                            try:
                                tello.send_rc_control(safe_controls[0], safe_controls[1], 
                                                    safe_controls[2], safe_controls[3])
                                if debug_mode:
                                    print(f"ドローン {i+1} にRCコマンド送信: {safe_controls}")
                            except Exception as rc_error:
                                print(f"ドローン {i+1} のRCコマンド送信エラー: {rc_error}")
                                control_anomaly_count += 1  # RC送信エラーもカウント
                        else:
                            # 位置データが取得できない場合の処理
                            position_data_missing_count += 1
                            print(f"ドローン{i+1} の位置データが利用できません (連続{position_data_missing_count}回)")
                            
                            # ホバリング制御値
                            control_values = [0, 0, 0, 0]
                            
                            # 長時間位置データがない場合は安全対策実施
                            if position_data_missing_count >= 10:
                                # 自動制御の安全な中断
                                print(f"警告: ドローン{i+1} の位置データが長時間取得できません。自動制御を停止します。")
                                
                                # ホバリングを送信
                                try:
                                    tello.send_rc_control(0, 0, 0, 0)  # ホバリング
                                    print(f"ドローン {i+1} にホバリングコマンド送信")
                                except Exception as e:
                                    print(f"ドローン {i+1} のホバリングコマンド送信失敗: {e}")
                                
                                # 15回以上連続して位置データがない場合は自動モードを停止
                                if position_data_missing_count >= 15:
                                    print("モーションキャプチャデータの長期間の欠損により自動モードを停止します")
                                    control_mode = "manual"
                            
                        # 安全対策: 制御異常を検出し、緊急時は手動モードに切り替え
                        if control_mode == "auto" and control_anomaly_count >= 5:
                            print(f"警告: ドローン{i+1} の制御異常が連続しています。緊急停止を検討してください。")
                            
                            # 5回以上異常が続く場合は安全対策を実施
                            if control_anomaly_count >= 10:
                                print(f"ドローン{i+1} の制御異常が長時間続いています。自動モードを無効化します。")
                                control_mode = "manual"  # 異常が続く場合は自動制御を停止
                                
                        # 各カウンターを保存
                        setattr(tello, "_position_data_missing_count", position_data_missing_count)
                        setattr(tello, "_control_anomaly_count", control_anomaly_count)
                    
                    # 手動モードの場合はキーボード入力による制御を使用
                    else:
                        # 手動制御値を送信
                        try:
                            tello.send_rc_control(manual_controls[0], manual_controls[1], 
                                                manual_controls[2], manual_controls[3])
                            if debug_mode:
                                print(f"ドローン {i+1} に手動コマンド送信: {manual_controls}")
                                
                            # 手動モードでもログ記録を行う
                            try:
                                # 位置データ取得
                                mocap_position = None
                                if USE_MOCAP and i < len(positions) and positions[i] is not None:
                                    mocap_position = positions[i]
                                
                                # 回転データ取得
                                quaternion = None
                                if USE_MOCAP and i < len(rotations) and rotations[i] is not None:
                                    quaternion = rotations[i]
                                
                                # 高度とバッテリー情報を取得
                                height = 0
                                battery = 0
                                try:
                                    height = tello.get_height()
                                    battery = tello.get_battery()
                                except Exception as state_error:
                                    print(f"ドローン{i+1}の状態取得エラー: {state_error}")
                                
                                # 目標位置とエラー計算
                                target_position = controllers[i].target_position if controllers else [0, 0, 0]
                                error_vec = [0, 0, 0]
                                if mocap_position and target_position:
                                    try:
                                        error_vec = [target_position[j] - mocap_position[j] for j in range(3)]
                                    except Exception as err_calc_error:
                                        print(f"手動モード時の誤差計算エラー: {err_calc_error}")
                                
                                # ログ記録
                                log_control_data(
                                    drone_index=i,
                                    mode="手動",
                                    mocap_position=mocap_position,
                                    target_position=target_position,
                                    error=error_vec,
                                    rc_values=manual_controls,
                                    height=height,
                                    battery=battery,
                                    start_time=None,
                                    quaternion=quaternion
                                )
                            except Exception as log_error:
                                print(f"手動モード時のログ記録エラー: {log_error}")
                        except Exception as e:
                            print(f"ドローン {i+1} へのコマンド送信に失敗しました: {e}")
            
            # コマンド送信時間を更新
            last_command_time = current_time
            
        except Exception as e:
            print(f"制御スレッドでエラーが発生しました: {e}")
            import traceback
            traceback.print_exc()  # スタックトレースを表示してデバッグを容易に
            # エラーが発生しても継続するために短い待機を入れる
            time.sleep(0.1)
            
            # エラーの場合でも、長時間通信が途絶えるとドローンが緊急着陸するので
            # キープアライブ時間をリセット
            if time.time() - last_keepalive_time > KEEPALIVE_INTERVAL * 1.5:
                try:
                    print("エラー発生後のキープアライブを試行...")
                    for i, tello in enumerate(drones):
                        if is_flying:
                            # エラー後の安全措置としてホバリング命令
                            try:
                                tello.send_rc_control(0, 0, 0, 0)
                            except Exception as rc_error:
                                print(f"ドローン {i+1} ホバリング送信エラー: {rc_error}")
                    last_keepalive_time = time.time()
                except Exception as keep_alive_error:
                    print(f"キープアライブ試行中にエラー: {keep_alive_error}")
    
    # 終了時の処理
    print("ドローン制御スレッドを終了します")


def cleanup():
    """
    リソースをクリーンアップする
    """
    global drones
    
    print("クリーンアップを実行しています...")
    # 自動制御ログを保存
    if any(len(v) for v in log_data.values()):
        results_dir = LOG_DIRECTORY
        os.makedirs(results_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        for idx, data in log_data.items():
            if not data:
                continue
            out_path = os.path.join(results_dir, f"auto_log_d{idx+1}_{timestamp_str}.jsonl")
            with open(out_path, "w", encoding="utf-8") as f:
                for item in data:
                    f.write(json.dumps(item) + "\n")
            print(f"自動制御ログを {out_path} に保存しました")
    
    # 飛行中の場合は着陸させる
    if is_flying:
        land_all()
    
    # ドローンとの接続を閉じる
    if tello_manager:
        tello_manager.shutdown()
    
    # MOCAPシステムとの接続を閉じる
    if USE_MOCAP:
        ms.shutdown()
    
    # Pygameを終了
    kp.quit()
    
    print("クリーンアップ完了")


def main():
    """
    メイン関数
    """
    global RUN_MODE, USE_MOCAP, USE_DRONES, should_stop
    
    # コマンドライン引数の処理
    parser = argparse.ArgumentParser(description="MOCAP for 2 TELLOs - 2機のTelloドローンをMOCAPでホバリング制御")
    parser.add_argument("--mode", choices=["comm", "mocap", "full"], default="full",
                        help="実行モード: comm=通信のみ, mocap=MOCAPのみ, full=統合テスト")
    args = parser.parse_args()
    
    # 実行モードの設定
    RUN_MODE = args.mode
    
    # モードに応じて機能を有効化/無効化
    if RUN_MODE == "comm":
        USE_MOCAP = False
        USE_DRONES = True
        print("[通信のみモード] ドローンとの通信テストを行います。MOCAPは使用しません。")
    elif RUN_MODE == "mocap":
        if not MOCAP_AVAILABLE:
            print("エラー: MOCAPモジュールが使用できないため、MOCAPモードを開始できません。")
            return
        USE_MOCAP = True
        USE_DRONES = False
        print("[MOCAPのみモード] MOCAPデータの取得テストを行います。ドローンは使用しません。")
    else:  # fullモード
        if not MOCAP_AVAILABLE:
            print("エラー: MOCAPモジュールが使用できないため、統合テストモードを開始できません。")
            return
        USE_MOCAP = True
        USE_DRONES = True
        print("[統合テストモード] MOCAPとドローンの完全統合テストを行います。")
    
    print("=== MOCAP for 2 TELLOs ===")
    print("2機のTelloドローンをMOCAPでホバリング制御")
    print("")
    print("キー操作:")
    print("- Q: 離陸")
    print("- E: 着陸")
    print("- M: 手動制御モード")
    print("- W/S: 上下移動")
    print("- A/D: 左右回転")
    print("- 矢印キー: 前後左右移動")
    if USE_MOCAP:
        print("- T: 目標位置に移動（自動ホバリングモード）")
        print("- M: 手動制御モード")
        print("- Z: 目標位置をリセット")
    if len(drones) > 1:
        print("- 1/2: ドローン選択 (1=ドローン1, 2=ドローン2, 0=全て)")
    print("- ESC: 緊急停止")
    print("- SPACE: 正常終了")
    print()
    
    # 初期化処理
    kp.init()
    
    # ドローンを初期化（ドローンモードの場合のみ）
    if USE_DRONES:
        initialize_drones()
    
    # MOCAPシステムを初期化（MOCAPモードの場合のみ）
    if USE_MOCAP:
        initialize_mocap()
        # 位置制御コントローラを初期化
        initialize_controllers()
        
    # ログ記録の初期化（ドローンモードの場合のみ）
    if USE_DRONES:
        initialize_loggers()
    
    # ステータス表示スレッドを開始
    status_thread = threading.Thread(target=status_display_thread)
    status_thread.daemon = True
    status_thread.start()
    
    # ドローン制御スレッドを開始
    control_thread = threading.Thread(target=control_drones_thread)
    control_thread.daemon = True
    control_thread.start()
    
    try:
        # メインスレッドはキー入力を監視
        while not should_stop:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nキーボード割り込みを検出しました。プログラムを終了します。")
    finally:
        # 終了処理
        should_stop = True
        cleanup()


if __name__ == "__main__":
    main()
