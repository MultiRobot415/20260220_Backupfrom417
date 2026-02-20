#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
オブザーバ型SLAF実機実装プログラム (v2)
元のMOCAP_for2TELLOsをベースに、SLAF制御を統合

理論ベース: v4_observer.tex, ref_v2/sim_v2/
"""

import os
import time
import threading
import numpy as np
from pathlib import Path

# 元のモジュール
import keyboard_control as kp
from custom_tello import CustomTello, TelloManager
from csv_logger import init_csv_logger, log_slaf_control_data as log_control_data, close_csv_logger

# SLAFモジュール (v2: オブザーバ型)
from slaf_observer_controller import SLAFSystemManager
from virtual_leader import VirtualLeaderManager

# MOCAPモジュール
try:
    import mocap_stream as ms
    MOCAP_AVAILABLE = True
except ImportError:
    print("警告: MOCAPモジュールをインポートできませんでした")
    MOCAP_AVAILABLE = False

# ========== グローバル変数 ==========
tello_manager = None
drones = []
is_flying = False
should_stop = False
control_mode = "manual"
slaf_mode_enabled = False
MOCAP_CONNECTED = False

# SLAF関連
slaf_manager = None
virtual_leaders = None
drone_to_follower_map = {0: 3, 1: 4}
follower_to_drone_map = {3: 0, 4: 1}

# Hモード軌道生成（src/CBF_for2TELLOs準拠）
h_mode_active = False  # Hモードアクティブフラグ
target_acceleration_2d = np.array([0.02, -0.02])  # 目標加速度 [X, Z] m/s^2
target_velocity_2d = np.array([0.0, 0.0])  # 目標速度 [X, Z] m/s
last_h_mode_update_time = None  # 前回のHモード更新時刻

# 各フォロワーの現在の目標位置（Tキー押下時に現在のMOCAP位置で初期化、Hモードで更新される）
# ★v1準拠: 固定値は使わず、Tキー押下時に現在のMOCAP位置を取得★
follower_target_positions = {}

# === H押下時の初期設定（v2オブザーバ性能検証用） ===
# H押下時に設定する初期目標位置と初期推定位置のオフセット [X, Z] (m)
# T押下: 制御開始、初期位置=初期推定位置=初期目標位置=現在位置（推定誤差ゼロ）
# H押下: 軌道追従開始、目標位置と推定位置を以下の値に設定

# 初期目標位置（H押下時に設定、絶対座標、MOCAP座標系）
H_MODE_INITIAL_TARGET_POSITION = {
    3: np.array([0, 0.8]),  # フォロワー3の初期目標位置 [X, Z] (m)
    4: np.array([0, 0.2])   # フォロワー4の初期目標位置 [X, Z] (m)
}

# 初期推定位置（H押下時に設定、絶対座標、MOCAP座標系）
# Noneの場合は現在の実位置（MOCAP）を使用 → 推定誤差ゼロ
# 座標を指定した場合はその位置を初期推定位置として使用
H_MODE_INITIAL_ESTIMATION_POSITION = {
    3: np.array([-0.1, 0.7]),  # None → 現在のMOCAP位置を使用 → 推定誤差ゼロ 
    4: np.array([-0.1, 0.2])    # None → 現在のMOCAP位置を使用 → 推定誤差ゼロ
}
# 推定誤差 = p_hat_H - p_actual（H押下時点の実位置）

# 設定例:
# - 推定誤差なし: None（実位置を使用）
# - 推定誤差あり（検証用）: np.array([-1.0, 1.0]) など絶対座標を指定

# 制御パラメータ
CONTROL_INTERVAL = 0.1
RIGID_BODY_IDS = [1, 2]
LOG_DIRECTORY = 'slaf_results'
DEFAULT_ALTITUDE = 1.0

# 仮想リーダー（固定位置）- 相対測定の基準点として配置
VIRTUAL_LEADER_POSITIONS = [
    [3.0, 1, -0.5],  # リーダー1（固定）(通常)
    [3.0, 1, 0.5]    # リーダー2（固定）(通常)
    #[0.0, 1, -0.39],  # リーダー1（固定）(共線状態)
    #[3.0, 1, -2.0] # リーダー（固定）(共線状態)
]

# 注意：初期位置・初期目標位置・初期推定位置は、すべてTキー押下時にMOCAPから取得します（v1準拠）
# 固定値は使用しません。

# 位置制御ゲイン - src/MOCAP_for2TELLOsと同じ
POSITION_GAIN = 10.0  # k1
VELOCITY_GAIN = 2.0  # k2
TARGET_STEP_SIZE = 0.05

# 制御パラメータ - src/MOCAP_for2TELLOsと同じ
MAX_SPEED = 50  # 最大速度
# 注意：不感帯はslaf_observer_controller.pyのdeadband_x, deadband_zで設定（デフォルト0.0）


# ========== 初期化関数 ==========
def initialize_drones():
    """ドローン初期化"""
    global tello_manager, drones
    
    print("ドローンへの接続を開始します...")
    tello_manager = TelloManager()
    tello_manager.find_available_tello(2)
    drones = tello_manager.get_tello_list()
    
    if len(drones) == 0:
        print("ドローンが見つかりません")
        return False
    
    print("ドローンに接続しています...")
    for i, tello in enumerate(drones):
        try:
            battery = tello.get_battery()
            print(f"ドローン {i+1} (IP: {tello.tello_ip}) バッテリー: {battery}%")
        except Exception as e:
            print(f"ドローン {i+1} への接続エラー: {e}")
    
    print(f"{len(drones)}機のドローンに接続しました")
    return True


def initialize_mocap():
    """MOCAP初期化"""
    global MOCAP_CONNECTED
    
    print("モーションキャプチャシステムに接続しています...")
    
    try:
        if not ms.initialize(debug_level=1):
            print("MOCAPに接続できませんでした（プログラムは継続します）")
            MOCAP_CONNECTED = False
            return True
    except Exception as e:
        print(f"MOCAP初期化エラー: {e}（プログラムは継続します）")
        MOCAP_CONNECTED = False
        return True
    
    time.sleep(2.0)
    
    try:
        status = ms.get_connection_status()
        if not status.get("connected", False):
            print("MOCAP接続に失敗しました（プログラムは継続します）")
            MOCAP_CONNECTED = False
            return True
    except:
        pass
    
    MOCAP_CONNECTED = True
    print("モーションキャプチャシステムに接続しました")
    return True


def initialize_slaf_system():
    """SLAFシステム初期化"""
    global slaf_manager, virtual_leaders
    
    print("SLAFシステムを初期化しています...")
    
    try:
        # 仮想リーダー管理（固定位置）
        virtual_leaders = VirtualLeaderManager(
            num_leaders=2,
            initial_positions=VIRTUAL_LEADER_POSITIONS,
            formation_offset=[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dt=CONTROL_INTERVAL
        )
        print("仮想リーダー管理を初期化しました（固定位置）")
        
        # SLAFシステム管理（v2: follower_configsを渡す）
        # グラフ構造（Plan仕様）
        # V = {1, 2, 3, 4}
        # V_l = {1, 2} (仮想リーダー)
        # V_f = {3, 4} (実機フォロワー)
        # N_3 = {1, 2}, N_4 = {1, 3}
        follower_configs = [
            {'id': 3, 'neighbors': [1, 2]},  # フォロワー3の隣人：リーダー1, 2
            {'id': 4, 'neighbors': [1, 3]}   # フォロワー4の隣人：リーダー1, フォロワー3
        ]
        slaf_manager = SLAFSystemManager(
            follower_configs=follower_configs,
            dt=CONTROL_INTERVAL
        )
        print("SLAF制御システムを初期化しました（v2オブザーバ型）")
        
        # 推定器の初期化はTキー押下時に行うため、ここでは初期化しない
        # （理由：Tキー押下時にMOCAPから現在位置を取得して初期化する方が正確）
        print("推定器の初期化はTキー押下時に実施されます")
        print("SLAFシステム初期化完了")
        return True
        
    except Exception as e:
        print(f"SLAFシステム初期化エラー: {e}")
        import traceback
        traceback.print_exc()
        return False


# ========== キーボード処理 ==========
def takeoff_drone(tello, index):
    """個別のドローンを離陸させる（スレッド用）- src/MOCAP_for2TELLOsと同じ"""
    try:
        print(f"ドローン {index+1} (IP: {tello.tello_ip}) を離陸させています...")
        result = tello.takeoff()
        print(f"ドローン {index+1} 離陸コマンド結果: {result}")
        
        # エラーでも処理を継続（既に飛行中の場合など）
        if result and "error" in str(result).lower():
            print(f"ドローン {index+1}: 離陸コマンドエラー（既に飛行中の可能性）")
    except Exception as e:
        print(f"ドローン {index+1} の離陸に失敗しました: {e}")
        import traceback
        traceback.print_exc()


def takeoff_all():
    """全ドローン離陸（別スレッドで実行）- src/MOCAP_for2TELLOsと同じ構造"""
    global is_flying
    
    if is_flying:
        print("既に飛行中です")
        return
    
    print("全てのドローンを同時に離陸させます...")
    print(f"ドローン数: {len(drones)}")
    
    # 離陸操作を並列スレッドで実行（重要：これによりフリーズしない）
    threads = []
    for i, tello in enumerate(drones):
        thread = threading.Thread(target=lambda t=tello, idx=i: takeoff_drone(t, idx))
        threads.append(thread)
        thread.start()
    
    # 全スレッドの完了を待つ
    for thread in threads:
        thread.join()
    
    # 離陸後にRCコマンドをリセット
    for tello in drones:
        tello.send_rc_control(0, 0, 0, 0)
    
    time.sleep(2)  # 安定化のための待機
    is_flying = True
    print("全てのドローンが離陸しました")


def land_drone(tello, index):
    """個別のドローンを着陸させる（スレッド用）"""
    try:
        print(f"ドローン {index+1} (IP: {tello.tello_ip}) を着陸させています...")
        result = tello.land()
        print(f"ドローン {index+1} 着陸コマンド結果: {result}")
    except Exception as e:
        print(f"ドローン {index+1} の着陸に失敗しました: {e}")
        import traceback
        traceback.print_exc()


def land_all():
    """全ドローン着陸（別スレッドで実行）- src/MOCAP_for2TELLOsと同じ構造"""
    global is_flying
    
    if not is_flying:
        print("既に着陸しています")
        return
    
    print("全てのドローンを着陸させます...")
    
    # 着陸操作を並列スレッドで実行
    threads = []
    for i, tello in enumerate(drones):
        thread = threading.Thread(target=lambda t=tello, idx=i: land_drone(t, idx))
        threads.append(thread)
        thread.start()
    
    # 全スレッドの完了を待つ
    for thread in threads:
        thread.join()
    
    is_flying = False
    print("全てのドローンが着陸しました")


def process_keyboard_input():
    """キーボード入力処理"""
    global is_flying, should_stop, control_mode, slaf_mode_enabled
    global h_mode_active, target_velocity_2d, last_h_mode_update_time
    
    # キーボード入力を取得
    pressed_keys = kp.get_pressed_keys()
    
    if pressed_keys:
        print(f"押されたキー: {pressed_keys}")
    
    # デバウンス処理
    current_time = time.time()
    
    # Q: 離陸
    if "q" in pressed_keys:
        if not hasattr(process_keyboard_input, 'q_last_time') or \
           current_time - process_keyboard_input.q_last_time > 0.5:
            process_keyboard_input.q_last_time = current_time
            print("Qキー検出 - 離陸")
            threading.Thread(target=takeoff_all, daemon=True).start()
    
    # E: 着陸
    if "e" in pressed_keys:
        if not hasattr(process_keyboard_input, 'e_last_time') or \
           current_time - process_keyboard_input.e_last_time > 1.0:
            process_keyboard_input.e_last_time = current_time
            print("Eキー検出 - 着陸")
            threading.Thread(target=land_all, daemon=True).start()
    
    # T: SLAF制御モード開始（モード切替のみ、制御は常に実行）
    if "t" in pressed_keys:
        if not hasattr(process_keyboard_input, 't_last_time') or \
           current_time - process_keyboard_input.t_last_time > 0.5:
            process_keyboard_input.t_last_time = current_time
            if control_mode != "slaf":
                print("=" * 60)
                print("Tキー検出 - SLAF制御モード開始")
                print("=" * 60)
                
                # 仮想リーダー位置を表示（固定）
                print("\n仮想リーダー（固定位置 - 相対測定基準点）:")
                for i, pos in enumerate(VIRTUAL_LEADER_POSITIONS):
                    print(f"  リーダー{i+1}: {pos}")
                
                # フォロワー目標位置を現在位置に設定（Tキー押下時の真の値を取得）
                print("\nフォロワー（実機ドローン）目標位置を現在位置に設定:")
                global follower_target_positions, target_velocity_2d, h_mode_active, last_h_mode_update_time
                
                # Hモードをリセット
                h_mode_active = False
                target_velocity_2d = np.array([0.0, 0.0])
                last_h_mode_update_time = None
                
                # 現在のMOCAP位置・速度を目標として設定（一回のみ取得）
                follower_target_positions = {}
                follower_target_velocities = {}
                
                if MOCAP_CONNECTED:
                    print("  MOCAP接続 - 現在の真の位置・速度を目標として設定")
                    for follower_id in [3, 4]:
                        tello_id = follower_to_drone_map[follower_id]
                        rigid_id = RIGID_BODY_IDS[tello_id]
                        
                        try:
                            mocap_pos = ms.get_rigid_body_position(rigid_id)
                            if mocap_pos:
                                x = mocap_pos.get('x', 0.0)
                                z = mocap_pos.get('z', 0.0)
                                follower_target_positions[follower_id] = np.array([x, z])
                                
                                # 速度は制御ループで推定された値を使用（ここでは初期値ゼロ）
                                follower_target_velocities[follower_id] = np.array([0.0, 0.0])
                                
                                print(f"  フォロワー{follower_id}（ドローン{tello_id+1}）: 目標位置=[{x:.3f}, {z:.3f}], 目標速度=[0.000, 0.000]")
                            else:
                                print(f"  ⚠️ フォロワー{follower_id}: MOCAP位置取得失敗 - デフォルト位置使用")
                                follower_target_positions[follower_id] = np.array([0.0, 0.0])
                                follower_target_velocities[follower_id] = np.array([0.0, 0.0])
                        except Exception as e:
                            print(f"  ⚠️ フォロワー{follower_id}: MOCAP位置取得エラー - {e}")
                            follower_target_positions[follower_id] = np.array([0.0, 0.0])
                            follower_target_velocities[follower_id] = np.array([0.0, 0.0])
                else:
                    print("  ⚠️ MOCAP未接続 - デフォルト位置を使用")
                    for follower_id in [3, 4]:
                        follower_target_positions[follower_id] = np.array([0.0, 0.0])
                        follower_target_velocities[follower_id] = np.array([0.0, 0.0])
                
                print("  Hモードをリセット")
                print("目標位置・速度設定完了（現在位置ベース）")
                
                # 推定器を初期化（T押下時: 推定誤差ゼロ）
                print("\n推定器初期化中（Assumption: 初期推定誤差ゼロ）...")
                if slaf_manager:
                    for follower_id in [3, 4]:
                        # 現在位置（MOCAP測定値）
                        p_actual = follower_target_positions[follower_id].copy()
                        v_actual = follower_target_velocities[follower_id].copy()
                        
                        # T押下時: 推定位置 = 実位置（推定誤差ゼロ）
                        p_hat_0 = p_actual.copy()
                        v_hat_0 = v_actual.copy()
                        
                        # 推定器を初期化
                        controller = slaf_manager.get_controller(follower_id)
                        if controller:
                            controller.initialize_state(p_hat_0, v_hat_0)
                        
                        # 表示
                        print(f"  フォロワー{follower_id}（ドローン{follower_to_drone_map[follower_id]+1}）:")
                        print(f"    p_actual(0) = [{p_actual[0]:.3f}, {p_actual[1]:.3f}]")
                        print(f"    p_star(0)   = [{p_actual[0]:.3f}, {p_actual[1]:.3f}]")
                        print(f"    p_hat(0)    = [{p_hat_0[0]:.3f}, {p_hat_0[1]:.3f}]")
                        print(f"    推定誤差    = [0.000, 0.000]")
                    
                    print("  ✅ 推定器を初期化しました（初期推定誤差ゼロ）")
                else:
                    print("⚠️ SLAF管理クラスが初期化されていません")
                
                print("\nSLAF制御モード")
                print("  H: Hモード開始（目標軌道生成）")
                print("  J: Hモード停止")
                print("=" * 60)
                
                # モード切替
                control_mode = "slaf"
                slaf_mode_enabled = True
    
    # M: 手動モード
    if "m" in pressed_keys:
        if not hasattr(process_keyboard_input, 'm_last_time') or \
           current_time - process_keyboard_input.m_last_time > 0.5:
            process_keyboard_input.m_last_time = current_time
            if control_mode != "manual":
                print("Mキー検出 - 手動モード")
                control_mode = "manual"
                slaf_mode_enabled = False
    
    # H: Hモード開始（一度押すと目標軌道生成開始）
    if "h" in pressed_keys:
        print(f"[DEBUG] Hキー検出: control_mode={control_mode}, virtual_leaders={virtual_leaders is not None}, h_mode_active={h_mode_active}")
        if control_mode == "slaf" and virtual_leaders:
            if not h_mode_active:
                print("=" * 60)
                print("Hキー検出 - Hモード開始（軌道追従開始）")
                print(f"目標加速度: [{target_acceleration_2d[0]:.3f}, {target_acceleration_2d[1]:.3f}] m/s^2")
                
                # H押下時: 初期目標位置と初期推定位置を設定
                print("\n初期目標位置と初期推定位置を設定中...")
                for follower_id in [3, 4]:
                    # 初期目標位置を設定（絶対座標、MOCAP座標系）
                    p_star_H = H_MODE_INITIAL_TARGET_POSITION.get(follower_id, np.array([0.0, 0.0]))
                    follower_target_positions[follower_id] = p_star_H.copy()
                    
                    # 初期推定位置を設定（絶対座標、MOCAP座標系）
                    controller = slaf_manager.get_controller(follower_id)
                    if controller:
                        # 現在の実位置を取得（推定誤差計算用）
                        p_actual_current = controller.p_actual.copy()
                        
                        # 初期推定位置を取得
                        p_hat_H_config = H_MODE_INITIAL_ESTIMATION_POSITION.get(follower_id, None)
                        if p_hat_H_config is None:
                            # None → 現在の実位置を使用（推定誤差ゼロ）
                            p_hat_H = p_actual_current.copy()
                        else:
                            # 絶対座標を指定（推定誤差あり）
                            p_hat_H = np.array(p_hat_H_config, dtype=float)
                        
                        v_hat_H = np.array([0.0, 0.0])  # 初期速度はゼロ
                        
                        # 推定位置のみを設定（実位置は変更しない）
                        controller.set_estimated_state(p_hat_H, v_hat_H)
                        
                        # 推定誤差を計算
                        estimation_error = p_hat_H - p_actual_current
                        error_norm = np.linalg.norm(estimation_error)
                        
                        # 表示
                        print(f"  フォロワー{follower_id}（ドローン{follower_to_drone_map[follower_id]+1}）:")
                        print(f"    p_actual(H) = [{p_actual_current[0]:.3f}, {p_actual_current[1]:.3f}]  (MOCAP実測値)")
                        print(f"    p_star(H)   = [{p_star_H[0]:.3f}, {p_star_H[1]:.3f}]")
                        print(f"    p_hat(H)    = [{p_hat_H[0]:.3f}, {p_hat_H[1]:.3f}]")
                        if error_norm > 1e-6:
                            print(f"    推定誤差    = [{estimation_error[0]:.3f}, {estimation_error[1]:.3f}] (ノルム: {error_norm:.3f}m)")
                        else:
                            print(f"    推定誤差    = [0.000, 0.000]  (ノルム: 0.000m)")
                
                # Hモードをアクティブ化
                h_mode_active = True
                target_velocity_2d = np.array([0.0, 0.0])  # 初期速度ゼロ
                last_h_mode_update_time = current_time
                
                print("✅ 初期設定完了 - 軌道追従を開始します")
                print(f"[DEBUG] h_mode_active={h_mode_active}, last_h_mode_update_time={last_h_mode_update_time}")
                print("=" * 60)
            else:
                print("[DEBUG] Hモードは既にアクティブです")
        else:
            print(f"[DEBUG] Hモード開始条件を満たしていません")
    
    # J: Hモード停止
    if "j" in pressed_keys:
        if h_mode_active:
            print("=" * 60)
            print("Jキー検出 - Hモード停止")
            h_mode_active = False
            target_velocity_2d = np.array([0.0, 0.0])
            last_h_mode_update_time = None
            print("=" * 60)
    
    # O: オクルージョンモードON（フォロワー4 = ドローン2）
    if "o" in pressed_keys:
        if not hasattr(process_keyboard_input, 'o_last_time') or \
           current_time - process_keyboard_input.o_last_time > 0.5:
            process_keyboard_input.o_last_time = current_time
            if control_mode == "slaf" and slaf_manager:
                print("=" * 60)
                print("Oキー検出 - オクルージョンモードON")
                print("対象：フォロワー4（ドローン2、TelloID 1）")
                print("  - 重み行列 H = 0（隣接情報なし）")
                print("  - ξ = 0（幾何学的補正なし）")
                print("  - ψ, τが有効（共線回避動作）")
                print("=" * 60)
                slaf_manager.set_follower_occlusion(4, True)
    
    # P: オクルージョンモードOFF
    if "p" in pressed_keys:
        if not hasattr(process_keyboard_input, 'p_last_time') or \
           current_time - process_keyboard_input.p_last_time > 0.5:
            process_keyboard_input.p_last_time = current_time
            if control_mode == "slaf" and slaf_manager:
                print("=" * 60)
                print("Pキー検出 - オクルージョンモードOFF")
                print("フォロワー4のセンサ復旧")
                print("=" * 60)
                slaf_manager.set_follower_occlusion(4, False)
    
    # ESC: 緊急停止
    if "ESCAPE" in pressed_keys:
        print("緊急停止")
        should_stop = True
    
    # SPACE: 正常終了
    if "SPACE" in pressed_keys:
        print("正常終了")
        should_stop = True
    
    return [0, 0, 0, 0]


# ========== 制御スレッド ==========
def control_drones_thread():
    """ドローン制御スレッド（元のMOCAP_for2TELLOsと同じ構造）"""
    global should_stop, slaf_mode_enabled, is_flying
    global h_mode_active, target_velocity_2d, target_acceleration_2d, last_h_mode_update_time, follower_target_positions
    
    print("制御スレッド開始")
    
    last_command_time = time.time()
    last_csv_log_time = time.time()
    last_status_print_time = time.time()
    CSV_LOG_INTERVAL = 0.5  # CSVログ記録間隔（秒）- 0.5秒ごと（以前は1.0秒）
    STATUS_PRINT_INTERVAL = 2.0  # ステータス表示間隔（秒）
    
    while not should_stop:
        try:
            current_time = time.time()
            
            # 制御間隔チェック
            if current_time - last_command_time < CONTROL_INTERVAL:
                time.sleep(0.01)
                continue
            
            last_command_time = current_time
            
            # キーボード入力処理
            process_keyboard_input()
            
            # SLAF制御モード：常に観測・推定・制御を実行（MOCAP未接続でも実行）
            if control_mode == "slaf" and is_flying:
                try:
                    # 定期的なステータス表示
                    if current_time - last_status_print_time >= STATUS_PRINT_INTERVAL:
                        h_status = "ON" if h_mode_active else "OFF"
                        print(f"[SLAF制御] 飛行: {is_flying} | Hモード: {h_status} | 時刻: {current_time:.2f}")
                        if h_mode_active:
                            print(f"  目標速度: [{target_velocity_2d[0]:.3f}, {target_velocity_2d[1]:.3f}] m/s")
                        last_status_print_time = current_time
                    
                    # Hモード：目標軌道更新（src/CBF_for2TELLOs準拠）
                    if h_mode_active and last_h_mode_update_time is not None:
                        dt = current_time - last_h_mode_update_time
                        # 目標速度を更新：v_star(t+dt) = v_star(t) + a_star * dt
                        old_velocity = target_velocity_2d.copy()
                        target_velocity_2d += target_acceleration_2d * dt
                        # 目標位置を更新：p_star(t+dt) = p_star(t) + v_star * dt
                        dx = target_velocity_2d[0] * dt
                        dz = target_velocity_2d[1] * dt
                        
                        # 各フォロワーの目標位置を更新（同じ速度で移動）
                        for follower_id in [3, 4]:
                            follower_target_positions[follower_id][0] += dx
                            follower_target_positions[follower_id][1] += dz
                        
                        # デバッグ出力（最初の数回のみ）
                        if not hasattr(control_drones_thread, 'h_mode_debug_count'):
                            control_drones_thread.h_mode_debug_count = 0
                        if control_drones_thread.h_mode_debug_count < 5:
                            print(f"[DEBUG] Hモード更新: dt={dt:.3f}, v_old=[{old_velocity[0]:.4f}, {old_velocity[1]:.4f}], v_new=[{target_velocity_2d[0]:.4f}, {target_velocity_2d[1]:.4f}], dx={dx:.4f}, dz={dz:.4f}")
                            print(f"[DEBUG]   フォロワー3目標: [{follower_target_positions[3][0]:.4f}, {follower_target_positions[3][1]:.4f}]")
                            print(f"[DEBUG]   フォロワー4目標: [{follower_target_positions[4][0]:.4f}, {follower_target_positions[4][1]:.4f}]")
                            control_drones_thread.h_mode_debug_count += 1
                    if h_mode_active:
                        last_h_mode_update_time = current_time
                    
                    # 仮想リーダー更新（固定位置だが、目標位置は更新可能）
                    virtual_leaders.update_all()
                    leader_states_3d = virtual_leaders.get_all_states()
                    
                    # 2D状態に変換（x, z平面）
                    leader_states_2d = []
                    for state_3d in leader_states_3d:
                        leader_states_2d.append({
                            'position': np.array([state_3d['position'][0], state_3d['position'][2]]),
                            'target_position': np.array([state_3d['target_position'][0], state_3d['target_position'][2]])
                        })
                    
                    # フォロワー目標設定（各フォロワー独立、オフセットなし）
                    follower_targets = {}
                    
                    for follower_id in [3, 4]:
                        # 各フォロワーの目標位置を使用（初期位置からHモードで更新）
                        target_pos = follower_target_positions[follower_id]
                        
                        # Hモード時は目標速度・加速度を設定
                        if h_mode_active:
                            follower_targets[follower_id] = {
                                'position': target_pos,
                                'velocity': target_velocity_2d,      # Hモードから取得
                                'acceleration': target_acceleration_2d  # Hモードから取得
                            }
                        else:
                            follower_targets[follower_id] = {
                                'position': target_pos,
                                'velocity': np.zeros(2),
                                'acceleration': np.zeros(2)
                            }
                    
                    slaf_manager.set_follower_targets(follower_targets)
                    
                    # MOCAP位置取得と制御
                    mocap_positions = {}
                    if MOCAP_CONNECTED:
                        for tello_id in [0, 1]:
                            follower_id = drone_to_follower_map[tello_id]
                            rigid_id = RIGID_BODY_IDS[tello_id]
                            
                            try:
                                mocap_pos = ms.get_rigid_body_position(rigid_id)
                                if mocap_pos:
                                    x = mocap_pos.get('x', 0.0)
                                    z = mocap_pos.get('z', 0.0)
                                    mocap_positions[follower_id] = np.array([x, z])
                                else:
                                    controller = slaf_manager.follower_controllers[follower_id]
                                    mocap_positions[follower_id] = controller.p_actual
                            except:
                                controller = slaf_manager.follower_controllers[follower_id]
                                mocap_positions[follower_id] = controller.p_actual
                    else:
                        # MOCAP未接続時は推定位置を使用
                        for tello_id in [0, 1]:
                            follower_id = drone_to_follower_map[tello_id]
                            controller = slaf_manager.follower_controllers[follower_id]
                            mocap_positions[follower_id] = controller.p_actual
                    
                    # SLAF制御更新（観測・推定・制御）
                    control_inputs = slaf_manager.update_followers(mocap_positions, leader_states_2d)
                    
                    # デバッグ：制御入力確認（最初の5回のみ）
                    if not hasattr(control_drones_thread, 'control_debug_count'):
                        control_drones_thread.control_debug_count = 0
                    if control_drones_thread.control_debug_count < 5:
                        print(f"[DEBUG] 制御入力: {control_inputs}")
                        control_drones_thread.control_debug_count += 1
                    
                    # 制御コマンド送信
                    for follower_id, u_2d in control_inputs.items():
                        tello_id = follower_to_drone_map[follower_id]
                        drone = drones[tello_id]
                        controller = slaf_manager.follower_controllers[follower_id]
                        
                        # 制御値をRC値に変換
                        velocity_gain = 50.0
                        control_lr = int(np.clip(u_2d[1] * velocity_gain, -MAX_SPEED, MAX_SPEED))  # z方向→左右
                        control_fb = int(np.clip(u_2d[0] * velocity_gain, -MAX_SPEED, MAX_SPEED))  # x方向→前後
                        control_ud = 0  # 高度は一定
                        control_yaw = 0
                        
                        # デバッグ出力（2秒ごと）
                        if current_time - last_status_print_time >= STATUS_PRINT_INTERVAL - 0.1:  # ステータス表示と同期
                            state = controller.get_state()
                            print(f"ドローン{tello_id+1}（フォロワー{follower_id}）:")
                            print(f"  実際位置=[{state['p_actual'][0]:.2f}, {state['p_actual'][1]:.2f}]")
                            print(f"  推定位置=[{state['p_hat'][0]:.2f}, {state['p_hat'][1]:.2f}]")
                            print(f"  目標位置=[{state['p_star'][0]:.2f}, {state['p_star'][1]:.2f}]")
                            print(f"  制御入力=[{u_2d[0]:.4f}, {u_2d[1]:.4f}]")
                            print(f"  RC指令=[lr:{control_lr}, fb:{control_fb}]")
                        
                        # 制御コマンド送信
                        try:
                            drone.send_rc_control(control_lr, control_fb, control_ud, control_yaw)
                        except Exception as e:
                            print(f"ドローン{tello_id+1}制御失敗: {e}")
                        
                        # CSVログ記録
                        if current_time - last_csv_log_time >= CSV_LOG_INTERVAL:
                            state = controller.get_state()
                            errors = controller.get_errors()
                            
                            log_data = {
                                'timestamp': time.time(),
                                'drone_id': tello_id,
                                'follower_id': follower_id,
                                'mode': 'slaf',
                                'position': state['p_actual'],
                                'position_hat': state['p_hat'],  # 推定位置
                                'target_position': state['p_star'],
                                'target_velocity': state['v_star'],  # 目標速度
                                'velocity': state['v_actual'],
                                'velocity_hat': state['v_hat'],  # 推定速度
                                'control_input': u_2d,
                                'rc_command': [control_lr, control_fb, control_ud, control_yaw],
                                'xi': state['xi'],
                                'xi_ijk': state['xi_ijk'],  # ξ_ijk（直接隣接）
                                'xi_sig': state['xi_sig'],  # ξ_sig（拡張エッジ）
                                'psi': state['psi'],
                                'tau': state['tau'],  # bearing誤差
                                'is_collinear': state['is_collinear'],
                                'is_occluded': state['is_occluded'],  # オクルージョン状態
                                'tracking_error': errors['tracking_position_error_norm'],
                                'estimation_error': errors['estimation_position_error_norm'],
                                'control_weight_norm': state['control_weight_norm']  # 制御則の重み行列H_iiのノルム
                            }
                            log_control_data(log_data)
                    
                    # CSV時刻更新
                    if current_time - last_csv_log_time >= CSV_LOG_INTERVAL:
                        last_csv_log_time = current_time
                
                except Exception as e:
                    print(f"SLAF制御エラー: {e}")
                    import traceback
                    traceback.print_exc()
            
        except Exception as e:
            print(f"制御ループエラー: {e}")
            time.sleep(0.1)
    
    print("制御スレッド終了")


# ========== メイン関数 ==========
def main():
    """メイン関数"""
    global should_stop
    
    print("=" * 60)
    print("オブザーバ型SLAF実機実装プログラム (v2)")
    print("理論: v4_observer.tex, sim_v2/system_dynamics.m")
    print("=" * 60)
    
    try:
        # キーボード初期化（最初に実行）
        print("[DEBUG] キーボード初期化開始")
        kp.init()
        print("[DEBUG] キーボード初期化完了")
        
        # CSVログ初期化
        log_dir = Path(LOG_DIRECTORY)
        log_dir.mkdir(exist_ok=True)
        init_csv_logger(LOG_DIRECTORY)
        print("CSVロガーを初期化しました")
        
        # ドローン初期化
        if not initialize_drones():
            print("ドローン初期化失敗")
            return
        
        # MOCAP初期化
        initialize_mocap()
        
        # システム初期化
        if not initialize_slaf_system():
            print("システム初期化失敗")
            return
        
        print("\n=== 全システム初期化完了 ===")
        print("\n【操作方法】")
        print("  Q: 離陸")
        print("  E: 着陸")
        print("  T: SLAF制御モード開始（観測・推定・制御を常に実行）")
        print("  H: Hモード開始（一度押すと目標軌道生成開始、加速度→速度→位置）")
        print("  J: Hモード停止")
        print("  O: オクルージョンON（フォロワー4 = ドローン2、ξ→0, ψ・τ有効）")
        print("  P: オクルージョンOFF（フォロワー4センサ復旧）")
        print("  M: 手動モード")
        print("  ESC: 緊急停止")
        print("  SPACE: 正常終了")
        print("\n" + "=" * 60)
        
        # 制御スレッド開始
        print("[DEBUG] 制御スレッド開始中")
        control_thread = threading.Thread(target=control_drones_thread, daemon=True)
        control_thread.start()
        print("[DEBUG] 制御スレッド開始完了")
        
        # メインループ（元のMOCAP_for2TELLOsと同じ）
        print("[DEBUG] メインループ開始")
        while not should_stop:
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nキーボード割り込み")
        should_stop = True
    
    except Exception as e:
        print(f"メイン関数エラー: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("[DEBUG] 終了処理開始")
        should_stop = True
        
        # ドローン着陸
        if drones and is_flying:
            for i, drone in enumerate(drones):
                try:
                    drone.land()
                except:
                    pass
        
        # MOCAP切断
        if MOCAP_CONNECTED:
            try:
                ms.shutdown()
            except:
                pass
        
        # CSVログクローズ
        try:
            close_csv_logger()
        except:
            pass
        
        # Pygame終了
        try:
            kp.quit()
        except:
            pass
        
        print("\n\nプログラム終了")


if __name__ == "__main__":
    main()
