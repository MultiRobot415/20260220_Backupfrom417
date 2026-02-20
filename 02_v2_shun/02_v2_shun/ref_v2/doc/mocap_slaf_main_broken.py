#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mocap_slaf_main.py - PID階層型SLAF実機実装メインプログラム

ref/sim_PID_v1 のPID階層型SLAFアルゴリズムを実機で再現
4エージェント構成: 仮想リーダー2機 + 実機フォロワー2機（Tello + MOCAP）

グラフ構造（Plan準拠）:
- V = {1, 2, 3, 4}
- V_l = {1, 2} (仮想リーダー)
- V_f = {3, 4} (実機フォロワー: Tello ID 0, 1)
- N_3 = {1, 2} (フォロワー3の隣人: リーダー1, 2)
- N_4 = {1, 3} (フォロワー4の隣人: リーダー1, フォロワー3)

キーボード操作:
- Q: 全ドローン離陸
- E: 全ドローン着陸
- T: SLAFモード開始（自動ホバリング + フォーメーション）
- M: 手動モードへ復帰
- G/B: 仮想リーダーの前進/後退（X軸、ステップ: 0.05m）
- V/N: 仮想リーダーの左/右移動（Z軸、ステップ: 0.05m）
- Z: 目標位置リセット
- ESC: 緊急停止
- SPACE: 正常終了

作成日: 2025-11-21
"""

import os
import sys
import time
import threading
import numpy as np
import logging
from datetime import datetime
from pathlib import Path

# 自作モジュール
import keyboard_control as kp
from custom_tello import CustomTello, TelloManager
from slaf_pid_controller import SLAFSystemManager
from virtual_leader import VirtualLeaderManager
from csv_logger import init_csv_logger, close_csv_logger, log_slaf_control_data as log_control_data, csv_debug_log

# MOCAPモジュール
try:
    import mocap_stream as ms
    MOCAP_AVAILABLE = True
except ImportError:
    print("警告: MOCAPモジュールをインポートできませんでした。")
    MOCAP_AVAILABLE = False
    sys.exit(1)

# ログ設定
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ========== グローバル変数 ==========
# ドローン関連
tello_manager = None
drones = []  # CustomTelloオブジェクト [drone3, drone4] (実際のTelloは0,1)
drone_to_follower_map = {0: 3, 1: 4}  # Tello ID -> Follower ID
follower_to_drone_map = {3: 0, 4: 1}  # Follower ID -> Tello ID

# SLAF制御システム
slaf_manager = None  # SLAFSystemManager
virtual_leaders = None  # VirtualLeaderManager

# 制御状態
is_flying = False
should_stop = False
control_mode = "manual"  # "manual" or "slaf"
slaf_mode_enabled = False
MOCAP_CONNECTED = False

# 制御パラメータ
CONTROL_INTERVAL = 0.1  # 制御周期（秒）= dt
SPEED = 50
ROTATION_SPEED = 50
KEEPALIVE_INTERVAL = 10.0

# 目標位置ステップサイズ
TARGET_STEP_SIZE = 0.05  # m

# MOCAP RigidBody IDs（実機フォロワー）
RIGID_BODY_IDS = [1, 2]  # Tello 0, 1 に対応

# デフォルト位置
DEFAULT_ALTITUDE = 1.0  # m
DEFAULT_LEADER_POSITIONS = [
    [0.0, DEFAULT_ALTITUDE, -0.5],  # リーダー1（左側）
    [0.0, DEFAULT_ALTITUDE, 0.5]    # リーダー2（右側）
]
DEFAULT_FOLLOWER_FORMATION_OFFSET = [
    [0.5, 0.0, -0.5],  # フォロワー3: リーダー1から前方0.5m
    [0.5, 0.0, 0.5]    # フォロワー4: リーダー2から前方0.5m
]

# CSVログ
LOG_DIRECTORY = 'slaf_results'


# ========== ドローン初期化 ==========
def initialize_drones():
    """Telloドローンの初期化"""
    global tello_manager, drones
    
    print("ドローンへの接続を開始します...")
    logger.info("ドローンへの接続を開始...")
    
    tello_manager = TelloManager()
    num_drones = 2
    print(f"{num_drones}機のTelloドローンを接続しています...")
    
    tello_manager.find_available_tello(num_drones)
    drones = tello_manager.get_tello_list()
    
    if len(drones) < num_drones:
        print(f"警告: {num_drones}機のドローンが必要ですが、{len(drones)}機しか見つかりませんでした")
        logger.warning(f"{num_drones}機のドローンが必要ですが、{len(drones)}機しか見つかりませんでした")
        if len(drones) == 0:
            print("ドローンが見つかりません。プログラムを終了します。")
            logger.error("ドローンが見つかりません")
            return False
    
    print("ドローンに接続しています...")
    for i, tello in enumerate(drones):
        try:
            battery = tello.get_battery()
            print(f"ドローン {i+1} (IP: {tello.tello_ip}) に接続しました。バッテリー残量: {battery}%")
            logger.info(f"ドローン{i} (IP: {tello.tello_ip}), バッテリー: {battery}%")
            if battery is not None and battery < 20:
                print(f"警告: ドローン {i+1} のバッテリー残量が低下しています ({battery}%)")
                logger.warning(f"ドローン{i}のバッテリー残量が低下({battery}%)")
        except Exception as e:
            print(f"ドローン {i+1} (IP: {tello.tello_ip}) への接続に失敗しました: {e}")
            logger.error(f"ドローン{i}への接続失敗: {e}")
    
    print(f"{len(drones)}機のドローンに接続しました。")
    logger.info(f"{len(drones)}機のドローン接続完了")
    return True


def initialize_mocap():
    """MOCAP初期化"""
    global MOCAP_CONNECTED
    
    print("モーションキャプチャシステムに接続しています...")
    logger.info("MOCAPシステムに接続中...")
    
    try:
        if not ms.initialize(debug_level=1):
            print("モーションキャプチャシステムに接続できませんでした")
            print("警告: MOCAPデータが利用できないため、SLAFモード(Tモード)は使用できません")
            print("手動モードでの操作は可能です")
            logger.error("MOCAP接続失敗")
            MOCAP_CONNECTED = False
            return True  # プログラムは継続する
    except Exception as e:
        print(f"MOCAPの初期化中にエラーが発生しました: {e}")
        print("警告: MOCAPデータが利用できないため、SLAFモード(Tモード)は使用できません")
        print("手動モードでの操作は可能です")
        logger.error(f"MOCAP初期化エラー: {e}")
        MOCAP_CONNECTED = False
        return True  # プログラムは継続する
    
    # 接続確認（数秒待機）
    time.sleep(2.0)
    
    # 接続状態を確認
    try:
        status = ms.get_connection_status()
        if not status.get("connected", False):
            print("モーションキャプチャシステムとの接続に失敗しました")
            print("警告: MOCAPデータが利用できないため、SLAFモード(Tモード)は使用できません")
            print("手動モードでの操作は可能です")
            logger.warning("MOCAP接続状態確認失敗")
            MOCAP_CONNECTED = False
            return True  # プログラムは継続する
    except Exception as e:
        logger.warning(f"MOCAP状態確認エラー: {e}")
        print("MOCAP状態確認でエラーが発生しました（プログラムは継続します）")
        # エラーでも継続
    
    # RigidBody確認
    for rigid_id in RIGID_BODY_IDS:
        try:
            position = ms.get_position(rigid_id)
            if position is None or position.get('is_dummy', True):
                print(f"RigidBody {rigid_id}のデータ取得に問題があります")
                logger.warning(f"RigidBody {rigid_id}のデータ取得失敗")
            else:
                print(f"RigidBody {rigid_id}: 位置確認OK")
                logger.info(f"RigidBody {rigid_id}: 位置={position}")
        except Exception as e:
            print(f"RigidBody {rigid_id}の確認中にエラー: {e}")
            logger.warning(f"RigidBody {rigid_id}確認エラー: {e}")
    
    MOCAP_CONNECTED = True
    print("モーションキャプチャシステムに接続しました")
    logger.info("MOCAP接続成功")
    return True


def initialize_slaf_system():
    """SLAFシステムの初期化"""
    global slaf_manager, virtual_leaders
    
    print("SLAFシステムを初期化しています...")
    logger.info("SLAFシステムを初期化中...")
    
    # 仮想リーダー管理を初期化
    virtual_leaders = VirtualLeaderManager(
        num_leaders=2,
        initial_positions=DEFAULT_LEADER_POSITIONS,
        formation_offset=[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        dt=CONTROL_INTERVAL
    )
    print("仮想リーダー管理を初期化しました")
    
    # SLAFシステム管理を初期化
    slaf_manager = SLAFSystemManager(dt=CONTROL_INTERVAL)
    print("SLAF制御システムを初期化しました")
    
    # フォロワー初期位置（MOCAP から取得）
    follower_positions = {}
    for tello_id in [0, 1]:
        follower_id = drone_to_follower_map[tello_id]
        rigid_id = RIGID_BODY_IDS[tello_id]
        
        if MOCAP_CONNECTED:
            mocap_pos = ms.get_position(rigid_id)
            if mocap_pos and not mocap_pos.get('is_dummy', True):
                # MOCAPから取得（x-z平面）
                x = mocap_pos.get('x', 0.0)
                z = mocap_pos.get('z', 0.0)
                follower_positions[follower_id] = [x, z]
                print(f"フォロワー{follower_id}初期位置（MOCAP）: [{x:.3f}, {z:.3f}]")
                logger.info(f"フォロワー{follower_id}初期位置（MOCAP）: [{x:.3f}, {z:.3f}]")
            else:
                # デフォルト位置
                default_pos = DEFAULT_LEADER_POSITIONS[tello_id]
                default_offset = DEFAULT_FOLLOWER_FORMATION_OFFSET[tello_id]
                x = default_pos[0] + default_offset[0]
                z = default_pos[2] + default_offset[2]
                follower_positions[follower_id] = [x, z]
                print(f"フォロワー{follower_id}初期位置（デフォルト）: [{x:.3f}, {z:.3f}]")
                logger.warning(f"フォロワー{follower_id}初期位置（デフォルト）: [{x:.3f}, {z:.3f}]")
        else:
            # MOCAP未接続時はデフォルト位置
            default_pos = DEFAULT_LEADER_POSITIONS[tello_id]
            default_offset = DEFAULT_FOLLOWER_FORMATION_OFFSET[tello_id]
            x = default_pos[0] + default_offset[0]
            z = default_pos[2] + default_offset[2]
            follower_positions[follower_id] = [x, z]
            print(f"フォロワー{follower_id}初期位置（デフォルト）: [{x:.3f}, {z:.3f}]")
            logger.info(f"フォロワー{follower_id}初期位置（デフォルト）: [{x:.3f}, {z:.3f}]")
    
    # SLAFフォロワーを初期化
    slaf_manager.initialize_followers(follower_positions)
    
    print("SLAFシステム初期化完了")
    logger.info("SLAFシステム初期化完了")
    return True


def shutdown_systems():
    """システムのシャットダウン"""
    global should_stop, drones, MOCAP_CONNECTED
    
    logger.info("システムをシャットダウン中...")
    should_stop = True
    
    # ドローン着陸
    if drones and is_flying:
        logger.info("全ドローン着陸中...")
        for i, drone in enumerate(drones):
            try:
                drone.land()
                logger.info(f"ドローン{i}着陸完了")
            except Exception as e:
                logger.error(f"ドローン{i}着陸失敗: {e}")
    
    # MOCAP切断
    if MOCAP_CONNECTED:
        try:
            ms.shutdown()
            logger.info("MOCAP切断完了")
        except:
            pass
    
    # CSVログクローズ
    try:
        close_csv_logger()
        logger.info("CSVログクローズ完了")
    except:
        pass
    
    logger.info("シャットダウン完了")


# ========== キーボード入力処理 ==========
def handle_keyboard_input():
    """キーボード入力を処理するスレッド"""
    global is_flying, should_stop, control_mode, slaf_mode_enabled
    
    logger.info("キーボード入力スレッド開始")
    
    # キーバインド初期化はメインスレッドで既に実行済み
    
    last_key_time = {}  # キーごとの最終押下時刻
    key_debounce_interval = 0.1  # デバウンス間隔（秒）
    
    try:
        while not should_stop:
            # キー入力取得（各キーを個別にチェック）
            current_time = time.time()
            
            # 各キーをチェック
            keys_to_check = ['q', 'e', 't', 'm', 'z', 'g', 'b', 'v', 'n', 'escape', 'space']
            key = None
            for k in keys_to_check:
                if kp.getKey(k):
                    key = k
                    break
            
            # デバウンス処理
            if key:
                if key in last_key_time:
                    if current_time - last_key_time[key] < key_debounce_interval:
                        continue
                last_key_time[key] = current_time
                
                # キー処理
                if key == 'q':  # 離陸
                    if not is_flying:
                        logger.info("離陸コマンド")
                        for i, drone in enumerate(drones):
                            try:
                                drone.takeoff()
                                logger.info(f"ドローン{i}離陸")
                            except Exception as e:
                                logger.error(f"ドローン{i}離陸失敗: {e}")
                        is_flying = True
                        csv_debug_log("command", "takeoff", "all_drones")
                
                elif key == 'e':  # 着陸
                    if is_flying:
                        logger.info("着陸コマンド")
                        for i, drone in enumerate(drones):
                            try:
                                drone.land()
                                logger.info(f"ドローン{i}着陸")
                            except Exception as e:
                                logger.error(f"ドローン{i}着陸失敗: {e}")
                        is_flying = False
                        csv_debug_log("command", "land", "all_drones")
                
                elif key == 't':  # SLAFモード開始
                    if control_mode != "slaf":
                        logger.info("SLAFモード開始")
                        control_mode = "slaf"
                        slaf_mode_enabled = True
                        csv_debug_log("mode_switch", "slaf_mode", "enabled")
                
                elif key == 'm':  # 手動モード
                    if control_mode != "manual":
                        logger.info("手動モード")
                        control_mode = "manual"
                        slaf_mode_enabled = False
                        csv_debug_log("mode_switch", "manual_mode", "enabled")
                
                elif key == 'z':  # 目標位置リセット
                    if slaf_mode_enabled:
                        logger.info("目標位置リセット")
                        virtual_leaders.set_reference_leader_target(
                            x=0.0, y=DEFAULT_ALTITUDE, z=0.0
                        )
                        csv_debug_log("target_reset", "leader_target", "reset_to_origin")
                
                # 仮想リーダーの目標位置移動（SLAFモード時のみ）
                elif slaf_mode_enabled:
                    if key == 'g':  # 前進
                        virtual_leaders.update_reference_leader_target(dx=TARGET_STEP_SIZE, dy=0.0, dz=0.0)
                        logger.debug(f"前進: dx={TARGET_STEP_SIZE}")
                    elif key == 'b':  # 後退
                        virtual_leaders.update_reference_leader_target(dx=-TARGET_STEP_SIZE, dy=0.0, dz=0.0)
                        logger.debug(f"後退: dx={-TARGET_STEP_SIZE}")
                    elif key == 'v':  # 左移動
                        virtual_leaders.update_reference_leader_target(dx=0.0, dy=0.0, dz=-TARGET_STEP_SIZE)
                        logger.debug(f"左移動: dz={-TARGET_STEP_SIZE}")
                    elif key == 'n':  # 右移動
                        virtual_leaders.update_reference_leader_target(dx=0.0, dy=0.0, dz=TARGET_STEP_SIZE)
                        logger.debug(f"右移動: dz={TARGET_STEP_SIZE}")
                
                # 緊急停止・終了
                elif key == 'escape':
                    logger.warning("緊急停止")
                    should_stop = True
                    break
                
                elif key == 'space':
                    logger.info("正常終了")
                    should_stop = True
                    break
            
            time.sleep(0.01)
    
    except Exception as e:
        logger.error(f"キーボード入力スレッドエラー: {e}")
    finally:
        logger.info("キーボード入力スレッド終了")


# ========== メイン制御ループ ==========
def slaf_control_loop():
    """SLAF制御ループ"""
    global slaf_mode_enabled, should_stop
    
    logger.info("SLAF制御ループ開始")
    
    loop_count = 0
    
    try:
        while not should_stop:
            loop_start_time = time.time()
            
            # SLAFモードが有効な場合のみ制御
            if slaf_mode_enabled and is_flying:
                # 1. 仮想リーダーを更新
                virtual_leaders.update_all()
                leader_states_3d = virtual_leaders.get_all_states()
                
                # 2. リーダーの水平状態を取得（x-z平面）
                leader_states_2d = []
                for state_3d in leader_states_3d:
                    leader_states_2d.append({
                        'position': np.array([state_3d['position'][0], state_3d['position'][2]]),
                        'target_position': np.array([state_3d['target_position'][0], state_3d['target_position'][2]])
                    })
                
                # 3. フォロワーの目標軌道を設定（フォーメーション）
                follower_targets = {}
                for follower_id in [3, 4]:
                    if follower_id == 3:
                        # フォロワー3の目標：リーダー1の位置 + オフセット
                        leader1_pos_2d = leader_states_2d[0]['target_position']
                        target_pos = leader1_pos_2d + np.array(DEFAULT_FOLLOWER_FORMATION_OFFSET[0][::2])  # [x, z]
                    else:  # follower_id == 4
                        # フォロワー4の目標：リーダー2の位置 + オフセット
                        leader2_pos_2d = leader_states_2d[1]['target_position']
                        target_pos = leader2_pos_2d + np.array(DEFAULT_FOLLOWER_FORMATION_OFFSET[1][::2])  # [x, z]
                    
                    follower_targets[follower_id] = {
                        'position': target_pos,
                        'velocity': np.zeros(2),
                        'acceleration': np.zeros(2)
                    }
                
                slaf_manager.set_follower_targets(follower_targets)
                
                # 4. MOCAPからフォロワー位置を取得
                mocap_positions = {}
                for tello_id in [0, 1]:
                    follower_id = drone_to_follower_map[tello_id]
                    rigid_id = RIGID_BODY_IDS[tello_id]
                    
                    mocap_pos = ms.get_position(rigid_id)
                    if mocap_pos and not mocap_pos.get('is_dummy', True):
                        x = mocap_pos.get('x', 0.0)
                        z = mocap_pos.get('z', 0.0)
                        mocap_positions[follower_id] = np.array([x, z])
                    else:
                        # ダミーデータの場合は前回値を使用
                        controller = slaf_manager.follower_controllers[follower_id]
                        mocap_positions[follower_id] = controller.p_actual
                
                # 5. SLAF制御を更新
                control_inputs = slaf_manager.update_followers(mocap_positions, leader_states_2d)
                
                # 6. 制御入力をドローンに送信（加速度→速度指令に変換）
                for follower_id, u_2d in control_inputs.items():
                    tello_id = follower_to_drone_map[follower_id]
                    drone = drones[tello_id]
                    
                    # 加速度指令を速度指令に変換（簡易的にゲインをかける）
                    # u_2d = [ax, az]  -> velocity = [vx, vz]
                    velocity_gain = 50.0  # 調整パラメータ
                    vx = int(np.clip(u_2d[0] * velocity_gain, -100, 100))
                    vz = int(np.clip(u_2d[1] * velocity_gain, -100, 100))
                    
                    # 高度は一定に保つ（簡易実装）
                    vy = 0
                    
                    # 回転なし
                    vyaw = 0
                    
                    # SDK指令送信
                    try:
                        drone.send_rc_control(vx, vz, vy, vyaw)
                    except Exception as e:
                        logger.error(f"ドローン{tello_id}制御入力送信失敗: {e}")
                    
                    # CSVログ記録
                    controller = slaf_manager.follower_controllers[follower_id]
                    state = controller.get_state()
                    errors = controller.get_errors()
                    
                    log_data = {
                        'timestamp': time.time(),
                        'drone_id': tello_id,
                        'follower_id': follower_id,
                        'mode': 'slaf',
                        'position': state['p_actual'],
                        'position_hat': state['p_hat'],
                        'target_position': state['p_star'],
                        'velocity': state['v_actual'],
                        'velocity_hat': state['v_hat'],
                        'control_input': u_2d,
                        'rc_command': [vx, vy, vz, vyaw],
                        'xi': state['xi'],
                        'psi': state['psi'],
                        'is_collinear': state['is_collinear'],
                        'tracking_error': errors['tracking_position_error_norm'],
                        'estimation_error': errors['estimation_position_error_norm']
                    }
                    
                    log_control_data(log_data)
            
            else:
                # 手動モードまたは着陸状態：何もしない
                time.sleep(0.05)
            
            # 制御周期を維持
            loop_count += 1
            elapsed = time.time() - loop_start_time
            sleep_time = CONTROL_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif loop_count % 10 == 0:
                logger.warning(f"制御ループ遅延: {elapsed:.4f}s（目標: {CONTROL_INTERVAL}s）")
    
    except Exception as e:
        logger.error(f"SLAF制御ループエラー: {e}", exc_info=True)
    finally:
        logger.info("SLAF制御ループ終了")


# ========== メイン関数 ==========
def main():
    """メイン関数"""
    global should_stop
    
    print("=" * 60)
    print("PID階層型SLAF実機実装プログラム")
    print("4エージェント構成: 仮想リーダー2 + 実機フォロワー2")
    print("=" * 60)
    
    # 初期化
    try:
        # キーボード初期化（最初に実行）
        print("[DEBUG] キーボード初期化開始")
        kp.init()
        print("[DEBUG] キーボード初期化完了")
        
        # CSVログ初期化
        print("[DEBUG] CSVロガー初期化開始")
        log_dir = Path(LOG_DIRECTORY)
        log_dir.mkdir(exist_ok=True)
        init_csv_logger(LOG_DIRECTORY)
        print("CSVロガーを初期化しました")
        logger.info("CSVロガー初期化完了")
        print("[DEBUG] CSVロガー初期化完了")
        
        # ドローン初期化
        print("[DEBUG] ドローン初期化開始")
        if not initialize_drones():
            print("エラー: ドローンの初期化に失敗しました")
            logger.error("ドローン初期化失敗")
            return
        print("[DEBUG] ドローン初期化完了")
        
        # MOCAP初期化
        print("[DEBUG] MOCAP初期化開始")
        if not initialize_mocap():
            # MOCAP失敗でもプログラムは継続（警告のみ）
            print("注意: MOCAP初期化に問題がありました（プログラムは継続します）")
            logger.warning("MOCAP初期化に問題")
        print("[DEBUG] MOCAP初期化完了")
        
        # SLAFシステム初期化
        print("[DEBUG] SLAFシステム初期化開始")
        if not initialize_slaf_system():
            print("エラー: SLAFシステムの初期化に失敗しました")
            logger.error("SLAFシステム初期化失敗")
            return
        print("[DEBUG] SLAFシステム初期化完了")
        
        print("\n=== 全システム初期化完了 ===")
        logger.info("全システム初期化完了")
        
        # スレッド開始
        print("[DEBUG] スレッド作成開始")
        keyboard_thread = threading.Thread(target=handle_keyboard_input, daemon=True)
        control_thread = threading.Thread(target=slaf_control_loop, daemon=True)
        print("[DEBUG] スレッド作成完了")
        
        print("[DEBUG] スレッド開始中")
        keyboard_thread.start()
        control_thread.start()
        print("[DEBUG] スレッド開始完了")
        
        logger.info("全スレッド開始")
        print("\n=== 準備完了 ===")
        print("キーボードウィンドウをクリックしてフォーカスしてください")
        print("")
        print("【操作方法】")
        print("  Q: 離陸")
        print("  E: 着陸")
        print("  T: SLAFモード開始（自動制御）")
        print("  M: 手動モード")
        print("  G/B: 前進/後退（仮想リーダー）")
        print("  V/N: 左/右移動（仮想リーダー）")
        print("  Z: 目標位置リセット")
        print("  ESC: 緊急停止")
        print("  SPACE: 正常終了")
        print("")
        print("注意: キーボードウィンドウに注目してください！")
        print("=" * 60 + "\n")
        
        # メインループ（状態表示）
        print("[DEBUG] メインループ開始")
        last_status_time = time.time()
        status_interval = 2.0  # 秒
        
        print("[DEBUG] whileループに入ります")
        while not should_stop:
            current_time = time.time()
            
            # 定期的に状態表示
            if current_time - last_status_time >= status_interval:
                print(f"\r[状態] モード: {control_mode:6s} | 飛行: {'Yes' if is_flying else 'No ':3s} | "
                      f"SLAF: {'ON ' if slaf_mode_enabled else 'OFF'}", end='', flush=True)
                last_status_time = current_time
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        logger.info("KeyboardInterruptを受信")
        should_stop = True
    
    except Exception as e:
        logger.error(f"メイン関数エラー: {e}", exc_info=True)
    
    finally:
        # シャットダウン
        print("[DEBUG] finally節に入りました")
        shutdown_systems()
        
        # Pygameを終了
        try:
            kp.quit()
        except:
            pass
        
        print("\n\nプログラム終了")


if __name__ == "__main__":
    main()
