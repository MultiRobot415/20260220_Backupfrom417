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
import math
import numpy as np
from pathlib import Path
from datetime import datetime
import keyboard_control as kp
from custom_tello import CustomTello, TelloManager

# グローバル変数の初期化
from position_control import PositionController

# CSVロガーモジュール
from csv_logger import init_csv_logger, close_csv_logger, log_control_data, csv_debug_log

# MOCAPモジュール（使用時のみインポート）
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

# フォーメーション制御関連
formation_control_enabled = False  # フォーメーション制御の有効/無効（Tモード時に自動有効化）

# ターゲット位置移動制御関連
target_moving = False  # ターゲット位置移動中フラグ
target_move_velocity = [0.02, 0.0, -0.1]  # [X, Y, Z]の移動速度 (m/s)
last_target_update_time = None  # 前回のターゲット位置更新時刻

# ログ用
log_data = {0: [], 1: []}  # 自動制御時の記録

# 実行モード
RUN_MODE = "full"  # デフォルトは統合テストモード
USE_MOCAP = True  # MOCAPを使用するかどうか
USE_DRONES = True  # ドローンを使用するかどうか

# リジッドボディの設定
rigid_body_ids = [1, 2]  # 各ドローンに対応するリジッドボディID

# 初期目標位置（x, y, z） - 修正後の値
# ユーザー指定の目標位置に修正
# proj座標系: ドローン1(z,x,y)=(1,0.9,1), ドローン2(z,x)=(1,0.1)
# proj座標系[X,Y,Z]: X=前後, Y=高度, Z=左右
default_target_positions = [
    [0.9, 1, 1],  # ドローン1の目標位置: proj(X,Y,Z) = (0.9, 1, 1) = (z,x,y)=(1,0.9,1)
    [0.1, 1, 1]   # ドローン2の目標位置: proj(X,Y,Z) = (0.1, 1, 1)
]

# 現在の目標位置（実行時に更新される）
target_positions = default_target_positions.copy()  # 初期値はデフォルト値のコピー

# 制御パラメータ（デフォルト値）
SPEED = 50  # 移動速度（0-100）
ROTATION_SPEED = 50  # 回転速度（0-100）
CONTROL_INTERVAL = 0.1  # 制御コマンド送信間隔（秒）
KEEPALIVE_INTERVAL = 10.0  # キープアライブ間隔（秒）- Telloのタイムアウト（15秒）より短くする
DATA_REFRESH_RATE = 0.1  # 位置データ更新間隔（秒）
STATUS_DISPLAY_INTERVAL = 1.0  # ステータス表示間隔（秒）

# デバッグモードフラグ
debug_mode = False  # キーボード入力のレスポンス改善のため無効化

# ロギング設定を完全に無効化
import logging
logging.getLogger().setLevel(logging.CRITICAL)  # rootロガーをCRITICALに設定
for logger_name in ['', 'MAIN', 'natnet', 'mocap', 'tello', 'control']:
    logging.getLogger(logger_name).setLevel(logging.CRITICAL)  # 全ロガーをCRITICALに設定

# ログ記録設定

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
    print(f"初期化前の状態: MOCAP_CONNECTED={MOCAP_CONNECTED}, USE_MOCAP={USE_MOCAP}")
    
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
        tracked_ids = ms.get_tracked_rigid_body_ids()
        print(f"追跡中のリジッドボディID: {tracked_ids}")
        if len(tracked_ids) == 0:
            print("警告: 追跡中のリジッドボディIDが見つかりません。MOCAPシステムからデータが配信されていない可能性があります。")
        MOCAP_CONNECTED = True
        print(f"初期化後の状態: MOCAP_CONNECTED={MOCAP_CONNECTED}, USE_MOCAP={USE_MOCAP}")
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
        list: 各ドローンの位置データとダミーフラグのタプルのリスト。データが取得できないドローンはNoneとダミーフラグのタプル
    """
    positions = []
    
    # MOCAPの状態を診断出力
    current_time = time.time()
    print(f"MOCAP診断 [{current_time:.3f}]: USE_MOCAP={USE_MOCAP}, MOCAP_CONNECTED={MOCAP_CONNECTED}")
    
    # 接続状態と品質の詳細をモニタリング
    if MOCAP_CONNECTED:
        try:
            status = ms.get_connection_status()
            tracked_ids = ms.get_tracked_rigid_body_ids()
            print(f"MOCAP接続詳細: 接続状態={status['connected']}, パケット数={status['packets']}, 追跡ID数={len(tracked_ids)}")
            print(f"追跡中のID一覧: {tracked_ids}")
            
            # 品質データをCSVログに記録
            try:
                csv_debug_log(
                    "mocap_quality", 
                    "MOCAP接続診断", 
                    {
                        "timestamp": current_time,
                        "connected": status['connected'], 
                        "packets": status['packets'],
                        "tracked_ids": len(tracked_ids),
                        "ids": str(tracked_ids)
                    }
                )
            except Exception as log_err:
                print(f"MOCAP品質ログ記録エラー: {log_err}")
                
        except Exception as e:
            print(f"MOCAP接続状態取得エラー: {e}")
            
            # 接続エラーをCSVログに記録
            try:
                csv_debug_log(
                    "mocap_error", 
                    "MOCAP接続エラー", 
                    {"timestamp": current_time, "error": str(e)}
                )
            except Exception as log_err:
                print(f"MOCAPエラーログ記録エラー: {log_err}")
    
    # MOCAPモードが無効か未接続の場合はNoneのリストを返す
    if not USE_MOCAP or not MOCAP_CONNECTED:
        dummy_positions = [(None, True)] * len(drones)  # ダミーフラグTrue
        print(f"MOCAPが無効または未接続のためダミーデータを返します: {dummy_positions}")
        return dummy_positions
    
    if debug_mode or True:  # 常に出力するために強制的にTrue
        print("DEBUG: get_drone_positionsのrigid_body_ids:", rigid_body_ids)
        print("DEBUG: dronesの長さ:", len(drones))
    
    try:
        for i, rb_id in enumerate(rigid_body_ids):
            if i >= len(drones):
                break
            
            try:
                print(f"DEBUG: RigidBody ID {rb_id} の位置データ取得試行")
                pos_data = ms.get_rigid_body_position(rb_id)
                print(f"DEBUG: RigidBody ID {rb_id} の位置データ取得結果: {pos_data}")
                
                if pos_data is not None:
                    # MOCAPからの位置データ形式をリストに変換
                    pos = [pos_data["x"], pos_data["y"], pos_data["z"]]
                    # 実データとダミーデータを区別するためにタプルで保存(データ, ダミーフラグ)
                    positions.append((pos, False))  # 実データのためダミーフラグFalse
                    if debug_mode:
                        print(f"DEBUG: RigidBody ID {rb_id} の位置データ取得成功: {pos}")
                else:
                    print(f"DEBUG: RigidBody ID {rb_id} の位置データがNoneです")
                    # ダミーデータとして識別できるようタプルで保存(データ, ダミーフラグ)
                    positions.append((None, True))  # ダミーフラグTrue
            except Exception as e:
                print(f"RigidBody {rb_id} の位置取得エラー: {e}")
                # エラー時もダミーデータとして識別できるようタプルで保存(データ, ダミーフラグ)
                positions.append((None, True))  # ダミーフラグTrue
    except Exception as e:
        print(f"MOCAP位置取得中にエラーが発生しました: {e}")
        # エラー時は全ドローンに対してダミーデータを返す
        return [(None, True)] * len(drones)  # ダミーフラグTrue
        
    # 返却前にリスト長を確認し、足りない場合はダミーデータで補完
    while len(positions) < len(drones):
        positions.append((None, True))  # ダミーフラグTrue
    
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


def status_display_thread():
    """
    ステータス表示を定期的に行うスレッド関数
    """
    global should_stop
    
    while not should_stop:
        display_status()
        time.sleep(STATUS_DISPLAY_INTERVAL)  # 設定された間隔で表示


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
                if battery is None:
                    print(f"ドローン {i+1} (IP: {tello.tello_ip}): 未接続 (電源OFFまたは通信不能)")
                else:
                    print(f"ドローン {i+1} (IP: {tello.tello_ip}): 接続済み - バッテリー残量 {battery}%")
            except Exception as e:
                print(f"ドローン {i+1} (IP: {tello.tello_ip}): 状態取得失敗 ({e})")
                print(f"ドローン {i+1} (IP: {tello.tello_ip}): 未接続 (電源OFFまたは通信不能)")
        
        # 制御モードと選択中のドローンを表示
        drone_str = "全て" if selected_drone == -1 else f"ドローン {selected_drone+1}"
        print(f"制御モード: {control_mode}, 選択中のドローン: {drone_str}")
    
    # 現在位置と目標位置を表示（MOCAPモードかつ接続済みの場合のみ）
    if USE_MOCAP and MOCAP_CONNECTED and controllers:
        try:
            positions = get_drone_positions()
            for i, pos_tuple in enumerate(positions):
                if i >= len(controllers):
                    break
                
                # タプル形式から位置データとダミーフラグを取得
                pos_data, is_dummy = pos_tuple if isinstance(pos_tuple, tuple) and len(pos_tuple) >= 2 else (None, True)
                
                target_pos = controllers[i].get_target_position()
                
                if pos_data is not None and not is_dummy:  # 実データの場合のみ処理
                    try:
                        error_dist = controllers[i].calculate_error_distance(pos_data)
                        
                        # 回転データの取得を試みる
                        rot_data = None
                        yaw_deg = "N/A"
                        try:
                            rot_data = ms.get_current_rotation(rigid_body_ids[i])
                            if rot_data is not None:
                                yaw_deg = f"{controllers[i].quaternion_to_yaw(rot_data):.1f}°"
                        except Exception:
                            pass
                            
                        print(f"ドローン {i+1}: Pos [{pos_data[0]:.2f}, {pos_data[1]:.2f}, {pos_data[2]:.2f}] | "
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


def initialize_controllers():
    """コントローラーの初期化"""
    global controllers
    
    # 各ドローンのポジションコントローラーを初期化
    controllers = []
    for i in range(len(drones)):
        controller = PositionController()
        controller.set_target_position(*default_target_positions[i])
        
        # フォーメーション制御を有効化（Phase 1: 1号機がリーダー、2号機がフォロワー）
        is_leader = (i == 0)  # 1号機（インデックス0）がリーダー
        controller.enable_formation_control(drone_id=i, is_leader=is_leader)
        
        # 機体間距離CBF (test.md 2式目) を両機で有効化
        controller.cbf_params.enable_velocity_constraints = True
        print(f"ドローン{i+1}: 機体間距離CBF=ON (test.md 2式目有効)")
        
        controllers.append(controller)
    
    print(f"{len(controllers)}機のポジションコントローラーを初期化しました（フォーメーション制御有効）")


def process_keyboard_input():
    """
    キーボード入力を処理する
    
    Returns:
        list: 手動モード時の制御値 [lr, fb, ud, yv] 
    """
    global is_flying, should_stop, control_mode, selected_drone, target_positions, formation_control_enabled
    
    # キーボード入力を取得し、デバッグ出力を追加
    pressed_keys = kp.get_pressed_keys()
    
    # デバッグ: キー情報の伝達状況を確認
    print(f"[DEBUG] process_keyboard_input: pressed_keys = {pressed_keys}")
    
    if pressed_keys:
        print(f"=========== 押されたキー: {pressed_keys} ===========")
    
    # キー入力のデバウンス処理（連続検出防止）
    current_time = time.time()
    
    # 目標位置操作キー（G/B/V/N）の場合はより頻繁な更新を許可（100ms間隔）
    position_control_keys = {"g", "b", "v", "n"}
    has_position_keys = any(key in pressed_keys for key in position_control_keys)
    
    # デバウンス間隔を動的に調整
    if has_position_keys and control_mode == "auto":
        debounce_interval = 0.1  # 目標位置操作は100ms間隔で許可（より連続的）
    else:
        debounce_interval = 0.05  # その他のキーは50ms間隔
    
    # 特殊キー（Q, E, T, SPACE, ESCAPE）は常に処理する
    special_keys = {"q", "e", "t", "SPACE", "ESCAPE"}
    has_special_keys = any(key in pressed_keys for key in special_keys)
    
    # 前回のキー処理から指定間隔以内の場合は処理をスキップ（ただし特殊キーは除く）
    if hasattr(process_keyboard_input, 'last_time') and current_time - process_keyboard_input.last_time < debounce_interval and not has_special_keys:
        if control_mode == "manual":
            # 手動制御値のみ継続して返す（他のキー処理はスキップ）
            return process_keyboard_input.last_values
        elif control_mode == "auto" and not has_position_keys:
            # 自動モードで目標位置操作以外のキーの場合はスキップ
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
        # デバウンス間隔を短縮（0.5秒）
        if not hasattr(process_keyboard_input, 'q_last_time') or \
           current_time - process_keyboard_input.q_last_time > 0.5:
            print("=========== Qキーが押されました - 離陸を開始します ===========")
            process_keyboard_input.q_last_time = current_time
            # スレッドでtakeoff_allを実行
            threading.Thread(target=takeoff_all).start()
        else:
            print(f"Q key debounce: {current_time - process_keyboard_input.q_last_time:.3f}s < 0.5s")
    
    # eキー処理の改善（デバウンス処理と非同期化）
    if "e" in pressed_keys:
        # eキーのデバウンス処理（前回押されてから1.0秒以上経過していれば処理する）
        if not hasattr(process_keyboard_input, 'e_last_time') or \
           current_time - process_keyboard_input.e_last_time > 1.0:
            process_keyboard_input.e_last_time = current_time
            print("Eキーが押されました - 着陸処理を開始します")
            
            # 非同期で着陸処理を実行
            threading.Thread(target=land_all).start()
    
    # hキー: ターゲット位置移動開始
    if "h" in pressed_keys:
        global target_moving, last_target_update_time
        if not target_moving:
            target_moving = True
            last_target_update_time = current_time
            print("=========== hキーが押されました - ターゲット位置移動を開始します ===========")
            print(f"移動速度: X={target_move_velocity[0]}m/s, Z={target_move_velocity[2]}m/s")
    
    # jキー: ターゲット位置移動停止
    if "j" in pressed_keys:
        if target_moving:
            target_moving = False
            print("=========== jキーが押されました - ターゲット位置移動を停止します ===========")
    
    # Tキー処理の改善（デバウンス処理を完全削除し、確実に反応させる）
    if "t" in pressed_keys:
        t_key_timestamp = time.time()
        print("======================================================")
        print(f"[{t_key_timestamp:.6f}] **** Tキーが押されました - モード切替処理を開始します ****")
        print(f"[{t_key_timestamp:.6f}] *** 現在のモード: {control_mode}, MOCAP接続状態: {MOCAP_CONNECTED} ***")
        csv_debug_log("t_key_pressed", f"current_mode={control_mode}", f"mocap_connected={MOCAP_CONNECTED}")
        
        # 手動モードからフォーメーション制御モードへの切替、または既存フォーメーションモードの再開始
        if control_mode == "manual" or control_mode == "auto":
            # 手動モードから自動モードへは、MOCAP接続が必要
            if not MOCAP_CONNECTED:
                print(f"[{time.time():.6f}] 警告: MOCAPデータが利用できないため、自動ホバリングモード(Tモード)は使用できません")
                print(f"[{time.time():.6f}] Manualモードでの操作は可能です")
                csv_debug_log("t_mode_denied", "reason=no_mocap", "")
            else:
                mode_change_timestamp = time.time()
                control_mode = "auto"  # Phase 2: 自動制御モードとして明確に記録
                formation_control_enabled = True  # Tモード時にフォーメーション制御を自動有効化
                # CBFフィルタをTモード時に有効化
                print(f"🚀 CBF有効化開始: Tモード切替時")
                for i, ctrl in enumerate(controllers):
                    try:
                        ctrl.cbf_enabled = True
                        print(f"✅ ドローン{i+1}: CBF有効化完了 (cbf_enabled={ctrl.cbf_enabled})")
                    except Exception as e:
                        print(f"⚠️ ドローン{i+1}: CBF有効化エラー: {e}")
                        pass
                print(f"[{mode_change_timestamp:.6f}] フォーメーション制御モードに切り替えました (手動→フォーメーションへの切替時間: {mode_change_timestamp - t_key_timestamp:.6f}秒)")
                print(f"[{mode_change_timestamp:.6f}] Phase 2: リーダー交代判定が有効化されました")
                csv_debug_log("mode_changed", "manual_to_auto", f"delay={mode_change_timestamp - t_key_timestamp:.6f}")
                csv_debug_log("formation_control", "enabled", "T_mode_activated")
                csv_debug_log("leader_switching", "enabled", "formation_mode_active")
                
                # 目標位置をデフォルト値（初期設定値）に固定
                print(f"[{time.time():.6f}] === ドローンの目標位置設定処理 ===")
                print(f"[{time.time():.6f}] 固定目標位置を使用します: {default_target_positions}")
                
                # 固定のデフォルト目標位置を使用
                target_positions = default_target_positions.copy()
                
                # 各ドローンのデフォルト目標位置をログに記録
                for i, target_pos in enumerate(target_positions):
                    update_start = time.time()
                    controller = controllers[i] if i < len(controllers) else None
                    if controller:
                        controller.set_target_position(*target_pos)
                    update_end = time.time()
                    print(f"[{update_end:.6f}] ドローン{i+1}の目標位置を設定: {target_pos} (処理時間: {update_end - update_start:.6f}秒)")
                    csv_debug_log("target_set_fixed", f"ドローン{i+1}", f"{target_pos}, time={update_end - update_start:.6f}")
                
                # 重要: コントローラーオブジェクトの目標位置も更新
                print("コントローラーの目標位置を更新します...")
                for i, target_pos in enumerate(target_positions):
                    if i < len(controllers):
                        # コントローラーの目標位置を更新
                        x, y, z = target_pos
                        controllers[i].set_target_position(x, y, z)
                        print(f"ドローン{i+1}のコントローラー目標位置を更新: {controllers[i].get_target_position()}")
                
                # 修正後の目標位置を表示
                print(f"最終的な目標位置: {target_positions}")
        elif control_mode == "auto":
            # 既にautoモードの場合はフォーメーション制御を再有効化
            formation_control_enabled = True
            print(f"[{time.time():.6f}] autoモードでフォーメーション制御を再有効化しました")
            csv_debug_log("formation_control", "re_enabled", "auto_mode_active")
        else:
            # その他のモードの場合は警告
            print(f"[{time.time():.6f}] 不明なモード: {control_mode}")
            csv_debug_log("t_key_ignored", "reason=unknown_mode", f"mode={control_mode}")
    
    if "m" in pressed_keys:
        if control_mode == "auto":
            control_mode = "manual"
            formation_control_enabled = False  # 手動モード時にフォーメーション制御を無効化
            # CBFフィルタをMモード時に無効化
            print(f"🚫 CBF無効化開始: Mモード切替時")
            for i, ctrl in enumerate(controllers):
                try:
                    ctrl.cbf_enabled = False
                    print(f"❌ ドローン{i+1}: CBF無効化完了 (cbf_enabled={ctrl.cbf_enabled})")
                except Exception:
                    pass
            print(f"[{time.time():.6f}] 手動操縦モードに切り替えました")
            print(f"[{time.time():.6f}] フォーメーション制御を無効化しました")
            csv_debug_log("mode_changed", "auto_to_manual", "")
            csv_debug_log("formation_control", "disabled", "manual_mode_activated")
        else:
            # 手動モードの場合は何もせず維持
            print(f"[{time.time():.6f}] 既に手動モードです。状態を維持します。")
            csv_debug_log("m_key_ignored", "reason=already_in_manual", "")
    
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
    
    # Tモード（フォーメーション制御モード）での目標位置操作
    if control_mode == "auto" and MOCAP_CONNECTED:
        # より滑らかな移動のため、ステップサイズを小さく設定
        position_step = 0.05  # 1回のキー入力で0.05m移動（従来の0.2mから変更）
        
        # 目標位置操作が実行されたかどうかのフラグ
        position_updated = False
        
        # 前後移動 (G/B)
        if "g" in pressed_keys:  # 前方移動
            for i, controller in enumerate(controllers):
                new_pos = controller.increment_target_position(dx=position_step)
                # グローバル変数target_positionsも更新
                if i < len(target_positions):
                    target_positions[i] = new_pos
                position_updated = True
            if position_updated:
                print(f"目標位置を前方に更新: ステップ={position_step}m")
                csv_debug_log("target_position_update", "all_drones", f"forward_{position_step}m")
                
        elif "b" in pressed_keys:  # 後方移動
            for i, controller in enumerate(controllers):
                new_pos = controller.increment_target_position(dx=-position_step)
                # グローバル変数target_positionsも更新
                if i < len(target_positions):
                    target_positions[i] = new_pos
                position_updated = True
            if position_updated:
                print(f"目標位置を後方に更新: ステップ={position_step}m")
                csv_debug_log("target_position_update", "all_drones", f"backward_{position_step}m")
        
        # 左右移動 (V/N)
        if "v" in pressed_keys:  # 左移動
            for i, controller in enumerate(controllers):
                new_pos = controller.increment_target_position(dz=-position_step)
                # グローバル変数target_positionsも更新
                if i < len(target_positions):
                    target_positions[i] = new_pos
                position_updated = True
            if position_updated:
                print(f"目標位置を左に更新: ステップ={position_step}m")
                csv_debug_log("target_position_update", "all_drones", f"left_{position_step}m")
                
        elif "n" in pressed_keys:  # 右移動
            for i, controller in enumerate(controllers):
                new_pos = controller.increment_target_position(dz=position_step)
                # グローバル変数target_positionsも更新
                if i < len(target_positions):
                    target_positions[i] = new_pos
                position_updated = True
            if position_updated:
                print(f"目標位置を右に更新: ステップ={position_step}m")
                csv_debug_log("target_position_update", "all_drones", f"right_{position_step}m")
        
        # 目標位置が更新された場合、現在の目標位置を表示
        if position_updated:
            for i, pos in enumerate(target_positions):
                print(f"  ドローン{i+1}の現在目標位置: {pos}")
    
    # F/Rモード処理は上部で統合済み
    # ここでは重複処理を行わない
    
    # 緊急停止
    if "ESCAPE" in pressed_keys:
        emergency_stop()
    
    # 通常終了
    if "SPACE" in pressed_keys:
        # デバウンス処理を追加
        if not hasattr(process_keyboard_input, 'space_last_time') or \
           current_time - process_keyboard_input.space_last_time > 0.5:
            should_stop = True
            process_keyboard_input.space_last_time = current_time
            print("=========== SPACEキーが押されました。プログラムを正常終了します。 ===========")
        else:
            print(f"SPACE key debounce: {current_time - process_keyboard_input.space_last_time:.3f}s < 0.5s")
    
    # キー入力の状態を記録（デバウンス用）
    process_keyboard_input.last_values = [lr, fb, ud, yv]
    if pressed_keys:
        print(f"キー入力検出: {', '.join(pressed_keys)}")
    
    return [lr, fb, ud, yv]


def control_drones_thread():
    """
    ドローン制御用スレッド関数
    """
    print("[DEBUG] control_drones_thread: 関数開始")
    
    try:
        print("[DEBUG] control_drones_thread: グローバル変数宣言開始")
        global should_stop, is_flying, drones, controllers, selected_drone_index, control_mode
        global rigid_body_ids, formation_control_enabled
        print("[DEBUG] control_drones_thread: グローバル変数宣言完了")
    except Exception as e:
        print(f"[ERROR] control_drones_thread: グローバル変数宣言でエラー: {e}")
        return
    
    # 異常カウンターの初期化
    control_anomaly_count = 0  # 制御異常カウンター
    position_data_missing_count = 0  # 位置データ欠損カウンター
    
    print(f"ドローン制御スレッドを開始しました ({RUN_MODE}モード)")
    
    # ドローンが使用されていない場合はキー入力処理のみを行う
    if not USE_DRONES:
        while not should_stop:
            process_keyboard_input()
            time.sleep(0.1)
        return
    
    last_command_time = time.time()
    last_keepalive_time = time.time()
    last_status_time = time.time()  # Tello状態データ取得の前回時間
    
    # Tello状態データ取得間隔（秒）
    STATUS_INTERVAL = 0.1  # 0.1秒間隔（高頻度ログ記録）
    
    print("[DEBUG] メインループ開始: while not should_stop ループに入ります")
    
    while not should_stop:
        print("[DEBUG] メインループ: ループの開始")
        try:
            current_time = time.time()
            print(f"[DEBUG] メインループ: current_time = {current_time:.1f}")
            
            # メインループの実行状況を確認
            if current_time % 5 < 0.1:  # 5秒ごとに表示
                print(f"[DEBUG] メインループ実行中: {current_time:.1f}s, should_stop={should_stop}")
            
            # コマンド送信間隔を調整
            if current_time - last_command_time < CONTROL_INTERVAL:
                time.sleep(0.01)
                continue
        except Exception as e:
            print(f"コントロールループの時間計算エラー: {e}")
            time.sleep(0.1)
            continue
            
        try:
                
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
                
            # Tello状態データの定期取得とログ記録 - 飛行状態や制御モードに関わらず実行
            if current_time - last_status_time > STATUS_INTERVAL:
                try:
                    print(f"Tello状態データ記録中 ({control_mode}モード)...")
                    # すべてのドローンの状態を取得（選択中のドローンに関わらず）
                    for i, tello in enumerate(drones):
                        try:
                            # Tello状態データを取得
                            status_data = tello.get_state()
                            if status_data:
                                # 取得した状態データをCSVに記録
                                from csv_logger import log_tello_status
                                log_tello_status(i+1, status_data)  # ドローンIDを渡す(1-indexed)
                                
                                # デバッグ出力（遠隔監視用）
                                if debug_mode:
                                    truncated = status_data[:50] + "..." if len(status_data) > 50 else status_data
                                    print(f"Tello {i+1} 状態: {truncated}")
                        except Exception as e:
                            print(f"ドローン{i+1}の状態データ取得エラー: {e}")
                    
                    # Phase 2: リーダー交代判定処理（ドローンループの前で一度だけ実行）
                    # current_leader_idxを初期化（デフォルトは1号機）
                    current_leader_idx = 0
                    
                    # 制御データも同時に記録（全ドローン・全時刻で必ず記録する）
                    print(f"制御データ記録中 ({control_mode}モード)...")
                    # positions配列が存在しない場合に備えて空のリストを作成
                    if 'positions' not in locals() or positions is None:
                        positions = [(None, True)] * len(drones)  # ダミーデータのリストを作成
                    
                    for i, tello in enumerate(drones):
                        try:
                            # 位置データ取得
                            mocap_position = None
                            is_dummy = True  # デフォルトはダミーデータと見なす
                            try:
                                if USE_MOCAP and i < len(positions) and positions[i] is not None:
                                    # タプル構造のチェック
                                    if isinstance(positions[i], tuple) and len(positions[i]) >= 2:
                                        mocap_position, is_dummy = positions[i]  # データ部分とダミーフラグを分離
                                        if is_dummy:
                                            print(f"警告: ドローン{i+1}のMOCAP位置データが取得できません")
                                    else:
                                        print(f"警告: ドローン{i+1}の位置データが不正な形式です: {positions[i]}")
                            except Exception as pos_err:
                                print(f"位置データ取得エラー: {pos_err}")
                                mocap_position = None
                                is_dummy = True
                            
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
                                # エラー発生時は0のままにする
                                if debug_mode:
                                    print(f"ドローン{i+1}の状態取得エラー: {state_error}")
                            
                            # 目標位置とエラー計算
                            target_position = controllers[i].target_position if controllers and i < len(controllers) else [0, 0, 0]
                            error_vec = [0, 0, 0]
                            if mocap_position and target_position:
                                try:
                                    error_vec = [target_position[j] - mocap_position[j] for j in range(3)]
                                except Exception:
                                    # エラー発生時は0のままにする
                                    pass
                            
                            # 現在の制御値を取得
                            current_controls = manual_controls.copy() if 'manual_controls' in locals() else [0, 0, 0, 0]
                            
                            # 役割情報（1号機=リーダー、2号機=フォロワー）
                            role = 'leader' if i == 0 else 'follower'
                            
                            # ログ記録 - 重要なパラメータのデフォルト値を設定
                            # MOCAPデータが取得できない場合は、ダミーフラグ付きで記録する
                            is_mocap_available = USE_MOCAP and MOCAP_CONNECTED
                            if is_dummy and is_mocap_available:
                                print(f"警告: ドローン{i+1}のMOCAP位置データが取得できません")
                                
                            # MOCAPが利用できない場合はNoneのままにして、CSVロガーで処理
                            # 異常カウンター値を取得（属性が存在しない場合は0にする）
                            control_anomaly_count = getattr(tello, "_control_anomaly_count", 0)
                            position_data_missing_count = getattr(tello, "_position_data_missing_count", 0)
                            
                            # 信頼度とオブザーバーデータ（デフォルト値）
                            trust_metric = 1.0
                            obs_error = [0, 0, 0]
                            obs_state = [0, 0, 0]
                            
                            # フォーメーション制御一本化: role-basedロジックを廃止し、フォーメーション制御のPD制御で自然収束
                            # CSV記録用の目標位置を取得
                            updated_target_position = controllers[i].get_target_position()
                            print(f"📊 ドローン{i+1} CSV記録用目標位置: {updated_target_position}")
                            
                            log_control_data(
                                drone_index=i,
                                mode=control_mode or "manual",
                                mocap_position=(mocap_position, is_dummy),  # タプルで(位置データ, ダミーフラグ)を渡す
                                target_position=updated_target_position,
                                error=error_vec or [0, 0, 0],
                                rc_values=current_controls or [0, 0, 0, 0],
                                height=height or 0,
                                battery=battery or 0,
                                start_time=None,
                                quaternion=quaternion,
                                role=role,
                                trust_metric=trust_metric,  # Phase 2: 信頼度を追加
                                obs_error=obs_error,  # Phase 2: オブザーバー残差を追加
                                obs_state=obs_state,  # Phase 2: オブザーバー推定状態を追加
                                control_anomaly_count=control_anomaly_count,  # 制御異常カウント
                                position_missing_count=position_data_missing_count  # 位置データ欠損カウント
                            )
                            
                            if debug_mode:
                                print(f"ドローン{i+1}の制御データを記録しました")
                        except Exception as log_error:
                            print(f"ドローン{i+1}の制御データ記録エラー: {log_error}")
                    
                    last_status_time = current_time
                except Exception as status_cycle_error:
                    print(f"状態データサイクル全体でエラー発生: {status_cycle_error}")
                    last_status_time = current_time  # エラー時も時間を更新して次のサイクルへ
            
            # キーボード入力を処理
            print("[DEBUG] メインループ: process_keyboard_input()を呼び出し中...")
            try:
                manual_controls = process_keyboard_input()
                print(f"[DEBUG] メインループ: process_keyboard_input()完了, manual_controls = {manual_controls}")
            except Exception as kb_error:
                print(f"[ERROR] process_keyboard_input()でエラー発生: {kb_error}")
                manual_controls = [0, 0, 0, 0]  # デフォルト値
            
            # ターゲット位置の自動更新処理
            global target_moving, last_target_update_time, target_positions
            if target_moving:
                if last_target_update_time is not None:
                    dt = current_time - last_target_update_time
                    # 全てのドローンのターゲット位置を更新
                    for i in range(len(target_positions)):
                        old_pos = target_positions[i].copy()
                        target_positions[i][0] += target_move_velocity[0] * dt  # X座標
                        target_positions[i][1] += target_move_velocity[1] * dt  # Y座標
                        target_positions[i][2] += target_move_velocity[2] * dt  # Z座標
                        print(f"🔄 ドローン{i+1} ターゲット更新: {old_pos} → {target_positions[i]}")
                    # コントローラーの目標位置も更新
                    for i, ctrl in enumerate(controllers):
                        ctrl.set_target_position(*target_positions[i])
                        print(f"✅ ドローン{i+1} コントローラー更新完了")
                last_target_update_time = current_time
            
            # 現在の位置データと回転データを取得（モードに関わらず取得してログ用に保存）
            positions = [None] * len(drones)
            rotations = [None] * len(drones)
            try:
                positions = get_drone_positions()
                rotations = get_drone_rotations()
                if control_mode == "manual":
                    print(f"manualモード: MOCAPデータは制御に使用しないがログに記録")
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
                    # 自動モードの場合は常に全てのドローンを制御
                    if control_mode == "manual" and selected_drone != -1 and selected_drone != i:
                        continue
                    
                    # データ異常監視用変数
                    position_data_missing_count = getattr(tello, "_position_data_missing_count", 0)
                    control_anomaly_count = getattr(tello, "_control_anomaly_count", 0)
                    last_position_time = getattr(tello, "_last_position_time", 0)
                    position_timeout = 2.0  # 位置データのタイムアウト秒数
                    
                    # 自動モードの場合は位置制御を使用
                    if control_mode == "auto":
                        # 位置データが存在するか確認
                        if i < len(positions) and positions[i][0] is not None:  # タプル対応修正
                            # データ部分とダミーフラグを分離
                            position_data, is_dummy = positions[i]
                            if is_dummy:
                                print(f"警告: ドローン{i+1}のMOCAP位置データが取得できません")
                            # 前回からの経過時間をチェック
                            position_time_diff = time.time() - last_position_time if last_position_time > 0 else 0
                            
                            # 位置データがダミーかどうかに基づいてカウンター処理
                            if not is_dummy:
                                # 実データの場合のみカウンターリセット
                                position_data_missing_count = 0
                                setattr(tello, "_last_position_time", time.time())
                                
                                # カウンターリセットをログ出力
                                csv_debug_log(
                                    "mocap_counter_reset", 
                                    f"ドローン{i+1}", 
                                    {"position_missing_count": position_data_missing_count}
                                )
                            else:
                                # ダミーデータの場合はカウントアップ
                                position_data_missing_count += 1
                                print(f"ドローン{i+1} の位置データが利用できません (連続{position_data_missing_count}回)")
                                
                                # カウントアップをログ出力
                                csv_debug_log(
                                    "mocap_counter_increment", 
                                    f"ドローン{i+1}", 
                                    {"position_missing_count": position_data_missing_count}
                                )
                            
                            # 制御ループ開始時間を記録（実行時間計測用）
                            loop_start_time = time.time()
                            
                            # 位置制御アルゴリズムを使用して制御値を計算
                            try:
                                # 回転情報があれば利用、なければNoneを渡す
                                quaternion = rotations[i] if i < len(rotations) else None
                                
                                # ===== STEP 1: 制御値計算 =====
                                # 制御計算開始時間を記録
                                control_calc_start = time.time()
                                
                                # フォーメーション制御の準備（Tモード時のみ）
                                if formation_control_enabled:
                                    # 他のドローンの情報を各コントローラーに更新
                                    for j, controller in enumerate(controllers):
                                        # 他のドローンの位置情報を更新
                                        for k in range(len(positions)):
                                            if k != j and k < len(positions) and positions[k][0] is not None:
                                                other_pos_data, other_is_dummy = positions[k]
                                                if not other_is_dummy:
                                                    # 速度推定のため位置データを更新
                                                    controller.update_other_drone_info(
                                                        drone_id=k, 
                                                        position=other_pos_data,
                                                        target=controllers[k].get_target_position() if k < len(controllers) else [0, 0, 0]
                                                    )
                                    
                                    # Phase 2: 動的リーダーの目標位置に基づいてフォロワーの目標位置を更新
                                    # role-basedロジック廃止により、フォーメーション制御一本化
                                    
                                    # リーダーは1号機に固定
                                    leader_controller = controllers[current_leader_idx]  # 動的リーダー
                                    leader_target = leader_controller.get_target_position()
                                    
                                    for j, controller in enumerate(controllers):
                                        if j != current_leader_idx:  # Phase 2: 動的リーダー以外はフォロワー
                                            controller.update_target_with_formation(leader_target)
                                    print(f"🔄 フォーメーション制御: リーダー{current_leader_idx+1}号機の目標位置 {leader_target} でフォロワー更新")
                                
                                # 位置制御計算
                                # タプル対応修正
                                position_data, is_dummy = positions[i]
                                if is_dummy:
                                    print(f"警告: ドローン{i+1}のMOCAP位置データが取得できません")
                                    # ダミーデータの場合は安全な値を使用
                                    control_values = [0, 0, 0, 0]
                                else:
                                    # 基本的な位置制御を計算
                                    control_values = controllers[i].calculate_control(position_data, quaternion=quaternion)
                                    
                                    # フォーメーション制御が有効な場合は追加の制御入力を適用
                                    if formation_control_enabled:
                                        try:
                                            formation_input = controllers[i].calculate_formation_control(position_data, CONTROL_INTERVAL)
                                            
                                            # フォーメーション制御入力を基本制御に加算（適度にスケーリング）
                                            formation_gain = 0.3  # フォーメーション制御のゲイン（調整可能）
                                            control_values[0] += formation_gain * formation_input[1]  # Y軸→左右
                                            control_values[1] += formation_gain * formation_input[0]  # X軸→前後
                                            control_values[2] += formation_gain * formation_input[2]  # Z軸→上下
                                            # Yaw制御（control_values[3]）はそのまま
                                            
                                            # 制御値を安全範囲に制限（基本制御と同じ±20に統一）
                                            for idx in range(3):
                                                control_values[idx] = max(-20, min(20, control_values[idx]))
                                            
                                            if i == 0:  # リーダーの場合のみログ出力（冗長性回避）
                                                print(f"フォーメーション制御適用: 入力={formation_input}, ゲイン={formation_gain}, 最終制御値={control_values[:3]}")
                                        except Exception as formation_error:
                                            print(f"ドローン{i+1}のフォーメーション制御エラー: {formation_error}")
                                            # エラー時はフォーメーション制御をスキップし、基本制御のみ使用（±20制限済み）
                                
                                # オブザーバー更新はドローンループの外で一括処理するため、ここでは制御値のみを保存
                                # 全ドローンの制御値を保存するためのリストを初期化（初回のみ）
                                if not hasattr(tello, '_all_control_values'):
                                    setattr(tello, '_all_control_values', [None] * len(drones))
                                
                                # 現在のドローンの制御値を保存
                                all_control_values = getattr(drones[0], '_all_control_values', [None] * len(drones))
                                all_control_values[i] = control_values.copy()
                                
                                # 全ドローンに同じリストを共有
                                for drone in drones:
                                    setattr(drone, '_all_control_values', all_control_values)
                                
                                control_calc_time = time.time() - control_calc_start
                                print(f"ドローン{i+1} 制御値計算時間: {control_calc_time:.4f}秒")
                                
                                # Phase 2: 正常な制御計算成功、カウンターリセット
                                control_anomaly_count = 0
                            except Exception as calc_error:
                                print(f"⚠️  ドローン{i+1} 制御値計算エラー: {calc_error} - ホバリング維持")
                                control_values = [0, 0, 0, 0]  # エラー時はホバリング
                                
                                # Phase 2: 制御計算エラーは深刻な異常のみカウント（一時的エラーを除外）
                                calc_error_count = getattr(locals(), 'calc_error_count', 0) + 1
                                if calc_error_count >= 3:  # 3回連続計算エラーのみ制御異常としてカウント
                                    control_anomaly_count += 1
                                    calc_error_count = 0  # リセット
                                    print(f"🚨 Phase 2: ドローン{i+1} の連続制御計算エラーを異常として記録")
                            
                            # 位置誤差を計算
                            try:
                                # タプル対応修正
                                position_data, is_dummy = positions[i]
                                if is_dummy:
                                    print(f"警告: ドローン{i+1}のMOCAP位置データが取得できません")
                                    # ダミーデータの場合は安全な値を使用
                                    error_vec = [0, 0, 0]
                                else:
                                    error_vec = [controllers[i].target_position[j] - position_data[j] for j in range(3)]
                                
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
                                if 0 < battery < 10:  # 15%→10%に調整
                                    print(f"警告: ドローン{i+1} のバッテリー残量が低下しています: {battery}%")
                                    
                                # バッテリーが極端に低い場合は緊急着陸を検討
                                if 0 < battery < 5:  # 10%→5%に調整
                                    print(f"警告: ドローン{i+1} のバッテリーが危険水準です: {battery}%")
                            except Exception as e:
                                print(f"ドローン{i+1}の状態取得エラー: {e}")
                                # heightとbatteryは既に0で初期化済み
                            
                            # CBF情報を取得
                            cbf_info = controllers[i].get_cbf_info()
                            
                            # csv_logger.pyのlog_control_data関数を使用
                            log_control_data(
                                drone_index=i,
                                mode="auto",
                                mocap_position=(positions[i][0], positions[i][1]),  # タプル形式でデータとダミーフラグを渡す
                                target_position=controllers[i].target_position,
                                error=error_vec,
                                rc_values=control_values,
                                height=height,
                                battery=battery,
                                start_time=loop_start_time,
                                quaternion=rotations[i] if i < len(rotations) else None,
                                trust_metric=None,  # オブザーバーデータはループ外で処理するため、ここではNone
                                role='follower',  # デフォルトはフォロワー、オブザーバー更新後に正しい役割が記録される
                                control_anomaly_count=control_anomaly_count,  # 制御異常カウント
                                position_missing_count=position_data_missing_count,  # 位置データ欠損カウント
                                # CBF関連パラメータ（仕様書準拠）
                                cbf_fire_flag=cbf_info['fire_flag'],
                                rc_safe_x=cbf_info['rc_safe_x'],
                                rc_safe_z=cbf_info['rc_safe_z'],
                                rc_diff_norm=cbf_info['rc_diff_norm'],
                                qp_status=cbf_info['qp_status'],
                                active_constraint_id=cbf_info['active_constraint_id'],
                                # CBF数学的値
                                rc_nom_x=cbf_info.get('rc_nom_x'),
                                rc_nom_z=cbf_info.get('rc_nom_z'),
                                h_x=cbf_info.get('h_x'),
                                HOh_x=cbf_info.get('HOh_x'),
                                # CBF速度差制約（h2, h3）
                                h2_value=cbf_info.get('h2_value'),
                                h2_satisfied=cbf_info.get('h2_satisfied'),
                                h3_value=cbf_info.get('h3_value'),
                                h3_satisfied=cbf_info.get('h3_satisfied'),
                                velocity_diff_x=cbf_info.get('velocity_diff_x'),
                                velocity_diff_y=cbf_info.get('velocity_diff_y')
                            )
                            
                            # デバッグログ用にデータを生成
                            if debug_mode:
                                log_entry = {
                                    "timestamp": time.time(),
                                    "drone_index": i,
                                    "position": positions[i][0] if not positions[i][1] else None,  # タプル対応修正
                                    "target": controllers[i].target_position,
                                    "error": error_vec,
                                    "control": control_values,
                                    "height": height,
                                    "battery": battery,
                                    "trust_metric": obs_data['trust'] if obs_data else None,
                                    "control_anomaly": control_anomaly_count,  # 制御異常カウント
                                    "position_missing": position_data_missing_count  # 位置データ欠損カウント
                                }
                                print(json.dumps(log_entry, indent=2))
                            
                            # 制御値の安全性チェックと制限
                            safe_controls = []
                            has_invalid_control = False
                            
                            for val in control_values:
                                # 無効な値のチェック (NaN, 無限大, 非数値)
                                if not isinstance(val, (int, float)) or val != val or abs(val) == float('inf'):
                                    # 真の異常値（NaN、無限大、非数値）の場合のみ異常カウント
                                    safe_val = 0
                                    has_invalid_control = True
                                    print(f"警告: ドローン{i+1} の無効な制御値 {val} を検出。0に置き換えました。")
                                elif abs(val) > 100:
                                    # 100を超える値は異常ではなく、単に大きな制御値として処理
                                    safe_val = max(min(int(val), 80), -80)  # 安全のために制限値を設定
                                    print(f"情報: ドローン{i+1} の大きな制御値 {val} を {safe_val} に制限しました。")
                                else:
                                    # 絶対値が大きい場合は制限する
                                    safe_val = max(min(int(val), 80), -80)  # 安全のために制限値を設定
                                
                                safe_controls.append(safe_val)
                            
                            # Phase 2: 真の異常のみカウント（NaN、無限大、非数値のみ）
                            if has_invalid_control:
                                control_anomaly_count += 1
                                print(f"⚠️  Phase 2: ドローン{i+1} の真の制御異常を検出 (NaN/Inf): {control_anomaly_count}/30")
                            else:
                                # 正常な制御値の場合はカウンターをリセット
                                if control_anomaly_count > 0:
                                    control_anomaly_count = max(0, control_anomaly_count - 1)  # 緊急減少
                                
                            # Phase 2: 異常検出による緊急停止条件を緩和（30回に増加）
                            if control_anomaly_count >= 15:
                                print(f"⚠️  警告: ドローン{i+1} の制御異常が連続しています。({control_anomaly_count}/30)")
                                
                                # 30回以上異常が続く場合は安全対策を実施
                                if control_anomaly_count >= 30:
                                    print(f"🛑 ドローン{i+1} の制御異常が長時間続いています。自動モードを無効化します。")
                                    control_mode = "manual"  # 異常が続く場合は自動制御を停止
                            
                            # Phase 2: 制御値を送信（エラーハンドリング改善）
                            try:
                                tello.send_rc_control(safe_controls[0], safe_controls[1], 
                                                      safe_controls[2], safe_controls[3])
                                if debug_mode:
                                    print(f"ドローン {i+1} にRCコマンド送信: {safe_controls}")
                                # RC送信成功時はエラーカウンターをリセット
                                rc_error_count = 0
                            except Exception as rc_error:
                                print(f"⚠️  ドローン {i+1} のRCコマンド送信エラー: {rc_error}")
                                # RC送信エラーは別カウンターで管理（一時的ネットワークエラーを考慮）
                                rc_error_count = getattr(locals(), 'rc_error_count', 0) + 1
                                if rc_error_count >= 5:  # 5回連続RCエラーのみ制御異常としてカウント
                                    control_anomaly_count += 1
                                    rc_error_count = 0  # リセット
                        else:
                            # 位置データが取得できない場合の処理
                            position_data_missing_count += 1
                            print(f"ドローン{i+1} の位置データが利用できません (連続{position_data_missing_count}回)")
                            
                            # カウントアップをログ出力（データなし）
                            csv_debug_log(
                                "mocap_data_missing",
                                f"ドローン{i+1}",
                                {
                                    "position_missing_count": position_data_missing_count,
                                    "reason": "no_position_data"
                                }
                            )
                            
                            # ホバリング制御値
                            control_values = [0, 0, 0, 0]
                            
                            # 長時間位置データがない場合は安全対策実施
                            if position_data_missing_count >= 20:  # 10→20に増加
                                # 自動制御の安全な中断
                                print(f"警告: ドローン{i+1} の位置データが長時間取得できません。自動制御を停止します。")
                                
                                # ホバリングを送信
                                try:
                                    tello.send_rc_control(0, 0, 0, 0)  # ホバリング
                                    print(f"ドローン {i+1} にホバリングコマンド送信")
                                except Exception as e:
                                    print(f"ドローン {i+1} のホバリングコマンド送信失敗: {e}")
                                
                                # 30回以上連続して位置データがない場合は自動モードを停止
                                if position_data_missing_count >= 30:  # 15→30に増加
                                    print("モーションキャプチャデータの長期間の欠損により自動モードを停止します")
                                    control_mode = "manual"
                            
                        # 安全対策: 制御異常を検出し、緊急時は手動モードに切り替え
                        if control_mode == "auto" and control_anomaly_count >= 10:  # 5→10に増加
                            print(f"警告: ドローン{i+1} の制御異常が連続しています。緊急停止を検討してください。")
                            
                            # 15回以上異常が続く場合は安全対策を実施
                            if control_anomaly_count >= 15:  # 10→15に増加
                                print(f"ドローン{i+1} の制御異常が長時間続いています。自動モードを無効化します。")
                                control_mode = "manual"  # 異常が続く場合は自動制御を停止
                                
                        # 各カウンターを保存
                        setattr(tello, "_position_data_missing_count", position_data_missing_count)
                        setattr(tello, "_control_anomaly_count", control_anomaly_count)
                    
                    # 手動モードの場合はキーボード入力による制御を使用
                    else:
                        # 手動制御値を送信
                        try:
                            # 制御値をコピー
                            controls_to_send = manual_controls.copy()
                            # 修正された制御値を送信
                            tello.send_rc_control(controls_to_send[0], controls_to_send[1], 
                                                  controls_to_send[2], controls_to_send[3])
                            if debug_mode:
                                print(f"ドローン {i+1} に手動コマンド送信: {controls_to_send}")
                                if fault_mode and controls_to_send != manual_controls:
                                    print(f"(故障注入モード有効中)")
                                
                            # 手動モードでもログ記録を行う
                            try:
                                # 位置データ取得
                                mocap_position = None
                                if USE_MOCAP and i < len(positions) and positions[i][0] is not None:  # タプル対応修正
                                    position_data, is_dummy = positions[i]  # データ部分とダミーフラグを分離
                                    if is_dummy:
                                        print(f"警告: ドローン{i+1}のMOCAP位置データが取得できません")
                                        mocap_position = None
                                    else:
                                        mocap_position = position_data
                                
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
                                # ✨ 重要: 既に取得済みのrole変数を使用（重複取得を防止）
                                # role変数は上部の統一された役割情報取得ロジックで既に設定済み
                                print(f"📊 Phase 2: 重複防止 - ドローン{i+1}の統一role変数を使用: {role}")
                                # ログ記録
                                log_control_data(
                                    drone_index=i,
                                    mode=control_mode or "manual",
                                    mocap_position=mocap_position,
                                    target_position=target_position,
                                    error=error_vec,
                                    rc_values=controls_to_send if 'controls_to_send' in locals() else manual_controls,
                                    height=height,
                                    battery=battery,
                                    start_time=None,
                                    quaternion=quaternion,
                                    role=role,
                                    control_anomaly_count=control_anomaly_count,  # 制御異常カウント
                                    position_missing_count=position_data_missing_count  # 位置データ欠損カウント
                                )
                            except Exception as log_error:
                                print(f"手動モード時のログ記録エラー: {log_error}")
                        except Exception as e:
                            print(f"ドローン {i+1} へのコマンド送信に失敗しました: {e}")
            
            # コマンド送信時間を更新
            last_command_time = current_time
            
            # 異常カウンターのデバッグ出力
            if control_anomaly_count > 0 or position_data_missing_count > 0:
                print(f"異常カウンター状態: 制御異常={control_anomaly_count}, 位置欠損={position_data_missing_count}")
                
                # 異常カウンターが警告レベルに近づいている場合は警告
                if control_anomaly_count > 5:
                    print(f"警告: 制御異常カウンターが高い値です({control_anomaly_count}/15)")
                if position_data_missing_count > 10:
                    print(f"警告: 位置データ欠損カウンターが高い値です({position_data_missing_count}/30)")
                
                # CSVにデバッグ情報記録
                try:
                    csv_debug_log(
                        "anomaly_counter", 
                        "T-mode diagnostics", 
                        {"control_anomaly": control_anomaly_count, "position_missing": position_data_missing_count}
                    )
                except Exception as e:
                    print(f"デバッグ情報記録エラー: {e}")
            

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
    """
    全てのドローンを着陸させる
    """
    global drones, is_flying
    
    print("全てのドローンを着陸させます...")
    
    # 各ドローンを着陸させる
    for i, drone in enumerate(drones):
        try:
            print(f"ドローン {i+1} を着陸させています...")
            drone.land()
            print(f"ドローン {i+1} の着陸成功")
        except Exception as e:
            print(f"ドローン {i+1} の着陸に失敗しました: {e}")
    
    # 着陸完了を記録
    is_flying = False


def perform_cleanup():
    """
    プログラム終了時のクリーンアップ処理
    """
    global drones, tello_manager
    
    print("クリーンアップを実行しています...")
    
    # 自動制御ログを保存
    try:
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
    except Exception as e:
        print(f"ログ保存中にエラーが発生しました: {e}")
    
    # CSVロガーを閉じる
    try:
        print("CSVロガーを閉じています...")
        close_csv_logger()
    except Exception as e:
        print(f"CSVロガーのクローズ中にエラーが発生しました: {e}")
    
    # 飛行中の場合は着陸させる
    if is_flying:
        try:
            land_all()  # ここで絶対に無限再帰が起きないように注意
        except Exception as e:
            print(f"着陸中にエラーが発生しました: {e}")
    
    # ドローンとの接続を閉じる
    try:
        if tello_manager:
            tello_manager.shutdown()
    except Exception as e:
        print(f"Tello Managerのシャットダウン中にエラーが発生しました: {e}")
    
    # MOCAPシステムとの接続を閉じる
    try:
        if USE_MOCAP and 'ms' in globals() and ms is not None:
            ms.shutdown()
    except Exception as e:
        print(f"MOCAP接続のシャットダウン中にエラーが発生しました: {e}")
    
    print("クリーンアップ完了")

    
    # Pygameを終了
    kp.quit()
    
    print("クリーンアップ完了")


def main():
    """
    メイン関数
    """
    global RUN_MODE, USE_MOCAP, USE_DRONES, should_stop, previous_leader_idx
    
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
    print("[DEBUG] メイン: 初期化処理開始")
    kp.init()
    print("[DEBUG] メイン: kp.init()完了")
    
    # ドローンを初期化（ドローンモードの場合のみ）
    if USE_DRONES:
        print("[DEBUG] メイン: ドローン初期化開始")
        initialize_drones()
        print("[DEBUG] メイン: ドローン初期化完了")
    
    # MOCAPシステムを初期化（MOCAPモードの場合のみ）
    if USE_MOCAP:
        print("[DEBUG] メイン: MOCAP初期化開始")
        initialize_mocap()
        print("[DEBUG] メイン: MOCAP初期化完了")
        # 位置制御コントローラを初期化
        print("[DEBUG] メイン: コントローラ初期化開始")
        initialize_controllers()
        print("[DEBUG] メイン: コントローラ初期化完了")
        
    # CSVロガーの初期化
    print("[DEBUG] メイン: CSVロガー初期化開始")
    print("ログ記録を初期化します...")
    log_file_path = init_csv_logger()
    print(f"CSVログファイルを作成しました: {log_file_path}")
    csv_debug_log("system", "program_start", {"mode": RUN_MODE, "mocap": USE_MOCAP, "drones": USE_DRONES})
    print("[DEBUG] メイン: CSVロガー初期化完了")
    
    # ステータス表示スレッドを開始
    print("[DEBUG] メイン: ステータススレッド開始")
    status_thread = threading.Thread(target=status_display_thread)
    status_thread.daemon = True
    status_thread.start()
    print("[DEBUG] メイン: ステータススレッド開始完了")
    
    # ドローン制御スレッドを開始
    print("[DEBUG] メイン: ドローン制御スレッドを作成中...")
    try:
        control_thread = threading.Thread(target=control_drones_thread)
        print("[DEBUG] メイン: スレッド作成完了")
        control_thread.daemon = True
        print("[DEBUG] メイン: デーモンフラグ設定完了")
        control_thread.start()
        print("[DEBUG] メイン: スレッド開始完了")
        print(f"[DEBUG] メイン: スレッド状態 - alive: {control_thread.is_alive()}, name: {control_thread.name}")
    except Exception as e:
        print(f"[ERROR] メイン: スレッド起動でエラー: {e}")
        print(f"[ERROR] メイン: エラー詳細: {type(e).__name__}: {str(e)}")
        import traceback
        traceback.print_exc()
    
    try:
        # メインスレッドはキー入力を監視
        while not should_stop:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nキーボード割り込みを検出しました。プログラムを終了します。")
    finally:
        # 終了処理
        should_stop = True
        print("プログラムを終了します...")
        perform_cleanup()
        print("全ての処理を完了しました。プログラムを強制終了します。")
        # 確実に終了するためにプロセスを強制終了
        import os
        os._exit(0)


if __name__ == "__main__":
    main()
