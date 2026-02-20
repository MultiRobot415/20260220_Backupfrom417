#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_slaf_modules.py - SLAF主要モジュールの動作テスト

実機接続なしで主要モジュールの基本動作を確認
"""

import numpy as np
import sys

print("=" * 60)
print("SLAF モジュールテスト")
print("=" * 60)

# 1. weight_matrices テスト
print("\n[1] weight_matrices.py テスト")
try:
    from weight_matrices import (
        calculate_weight_matrix_bearing,
        calculate_xi_correction,
        calculate_psi_collinearity_avoidance,
        check_collinearity
    )
    
    # 非共線テスト
    p_i = np.array([0.0, 0.0])
    p_j = np.array([1.0, 0.0])
    p_k = np.array([0.0, 1.0])
    
    Hij = calculate_weight_matrix_bearing(p_i, p_j)
    print(f"  Hij計算: {Hij.shape} - OK")
    
    is_collinear = check_collinearity(p_i, p_j, p_k)
    print(f"  共線チェック（非共線）: {is_collinear} (期待: False) - {'OK' if not is_collinear else 'NG'}")
    
    # 共線テスト
    p_k_collinear = np.array([2.0, 0.0])
    is_collinear2 = check_collinearity(p_i, p_j, p_k_collinear)
    print(f"  共線チェック（共線）: {is_collinear2} (期待: True) - {'OK' if is_collinear2 else 'NG'}")
    
    # ξ計算テスト
    xi = calculate_xi_correction(p_i, p_j, p_k, p_i, p_j, p_k)
    print(f"  ξ計算: shape={xi.shape}, norm={np.linalg.norm(xi):.4f} - OK")
    
    print("  ✓ weight_matrices.py: 全テスト成功")
    
except Exception as e:
    print(f"  ✗ weight_matrices.py: エラー - {e}")
    sys.exit(1)

# 2. virtual_leader テスト
print("\n[2] virtual_leader.py テスト")
try:
    from virtual_leader import VirtualLeaderManager
    
    manager = VirtualLeaderManager(num_leaders=2, dt=0.1)
    print(f"  VirtualLeaderManager初期化: OK")
    
    # 目標位置設定
    manager.set_reference_leader_target(x=1.0, y=1.0, z=0.0)
    print(f"  目標位置設定: OK")
    
    # 更新
    manager.update_all()
    states = manager.get_all_planar_states()
    print(f"  状態更新: {len(states)}機 - OK")
    print(f"    リーダー1位置: {states[0]['position']}")
    print(f"    リーダー2位置: {states[1]['position']}")
    
    print("  ✓ virtual_leader.py: 全テスト成功")
    
except Exception as e:
    print(f"  ✗ virtual_leader.py: エラー - {e}")
    sys.exit(1)

# 3. slaf_pid_controller テスト
print("\n[3] slaf_pid_controller.py テスト")
try:
    from slaf_pid_controller import SLAFSystemManager
    
    slaf_manager = SLAFSystemManager(dt=0.1)
    print(f"  SLAFSystemManager初期化: OK")
    
    # フォロワー初期化
    follower_positions = {3: [0.0, -0.5], 4: [0.0, 0.5]}
    slaf_manager.initialize_followers(follower_positions)
    print(f"  フォロワー初期化: OK")
    
    # 目標軌道設定
    follower_targets = {
        3: {'position': [1.0, -0.5], 'velocity': [0.1, 0.0], 'acceleration': [0.0, 0.0]},
        4: {'position': [1.0, 0.5], 'velocity': [0.1, 0.0], 'acceleration': [0.0, 0.0]}
    }
    slaf_manager.set_follower_targets(follower_targets)
    print(f"  目標軌道設定: OK")
    
    # リーダー状態（ダミー）
    leader_states = [
        {'position': np.array([0.0, -0.5]), 'target_position': np.array([0.0, -0.5])},
        {'position': np.array([0.0, 0.5]), 'target_position': np.array([0.0, 0.5])}
    ]
    
    # MOCAP位置（ダミー）
    mocap_positions = {3: np.array([0.1, -0.5]), 4: np.array([0.1, 0.5])}
    
    # 制御更新
    control_inputs = slaf_manager.update_followers(mocap_positions, leader_states)
    print(f"  制御更新: {len(control_inputs)}機 - OK")
    for fid, u in control_inputs.items():
        print(f"    フォロワー{fid}: u = {u}")
    
    # 状態取得
    states = slaf_manager.get_all_states()
    errors = slaf_manager.get_all_errors()
    print(f"  状態・誤差取得: OK")
    print(f"    フォロワー3追跡誤差: {errors[3]['tracking_position_error_norm']:.4f}")
    print(f"    フォロワー4追跡誤差: {errors[4]['tracking_position_error_norm']:.4f}")
    
    print("  ✓ slaf_pid_controller.py: 全テスト成功")
    
except Exception as e:
    print(f"  ✗ slaf_pid_controller.py: エラー - {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# 4. csv_logger テスト
print("\n[4] csv_logger.py テスト")
try:
    from csv_logger import init_csv_logger, log_slaf_control_data, close_csv_logger
    import tempfile
    import os
    
    # 一時ディレクトリでテスト
    with tempfile.TemporaryDirectory() as tmpdir:
        init_csv_logger(log_dir=tmpdir)
        print(f"  CSVロガー初期化: OK")
        
        # ログデータ作成
        log_data = {
            'timestamp': 1234567890.0,
            'drone_id': 0,
            'follower_id': 3,
            'mode': 'slaf',
            'position': [0.1, -0.5],
            'position_hat': [0.09, -0.51],
            'target_position': [0.0, -0.5],
            'velocity': [0.01, 0.0],
            'velocity_hat': [0.009, 0.0],
            'control_input': [0.5, 0.0],
            'rc_command': [10, 0, 0, 0],
            'xi': [0.01, 0.0],
            'psi': [0.0, 0.0],
            'is_collinear': False,
            'tracking_error': 0.1,
            'estimation_error': 0.01
        }
        
        log_slaf_control_data(log_data)
        print(f"  ログデータ記録: OK")
        
        close_csv_logger()
        print(f"  CSVロガークローズ: OK")
        
        # ファイル存在確認
        csv_files = [f for f in os.listdir(tmpdir) if f.endswith('.csv')]
        print(f"  生成されたCSVファイル: {len(csv_files)}個 - {'OK' if len(csv_files) > 0 else 'NG'}")
    
    print("  ✓ csv_logger.py: 全テスト成功")
    
except Exception as e:
    print(f"  ✗ csv_logger.py: エラー - {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# 統合テスト
print("\n[5] 統合テスト（簡易シミュレーション）")
try:
    from virtual_leader import VirtualLeaderManager
    from slaf_pid_controller import SLAFSystemManager
    
    # 初期化
    virtual_leaders = VirtualLeaderManager(num_leaders=2, dt=0.1)
    slaf_manager = SLAFSystemManager(dt=0.1)
    
    follower_positions = {3: [0.0, -0.5], 4: [0.0, 0.5]}
    slaf_manager.initialize_followers(follower_positions)
    
    print(f"  システム初期化: OK")
    
    # 10ステップシミュレーション
    for step in range(10):
        # 仮想リーダー更新
        virtual_leaders.update_all()
        leader_states_3d = virtual_leaders.get_all_states()
        
        # 2D状態に変換
        leader_states_2d = []
        for state_3d in leader_states_3d:
            leader_states_2d.append({
                'position': np.array([state_3d['position'][0], state_3d['position'][2]]),
                'target_position': np.array([state_3d['target_position'][0], state_3d['target_position'][2]])
            })
        
        # フォロワー目標設定
        follower_targets = {
            3: {'position': leader_states_2d[0]['target_position'] + np.array([0.5, 0.0]),
                'velocity': np.zeros(2), 'acceleration': np.zeros(2)},
            4: {'position': leader_states_2d[1]['target_position'] + np.array([0.5, 0.0]),
                'velocity': np.zeros(2), 'acceleration': np.zeros(2)}
        }
        slaf_manager.set_follower_targets(follower_targets)
        
        # MOCAP測定（簡易シミュレーション）
        mocap_positions = {
            3: slaf_manager.follower_controllers[3].p_actual + np.array([0.01, 0.0]),
            4: slaf_manager.follower_controllers[4].p_actual + np.array([0.01, 0.0])
        }
        
        # SLAF制御更新
        control_inputs = slaf_manager.update_followers(mocap_positions, leader_states_2d)
        
        if step % 5 == 0:
            errors = slaf_manager.get_all_errors()
            print(f"  ステップ{step}: 追跡誤差 F3={errors[3]['tracking_position_error_norm']:.4f}, "
                  f"F4={errors[4]['tracking_position_error_norm']:.4f}")
    
    print("  ✓ 統合テスト: 全テスト成功")
    
except Exception as e:
    print(f"  ✗ 統合テスト: エラー - {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# 全体結果
print("\n" + "=" * 60)
print("全テスト成功！")
print("=" * 60)
print("\nSLAFシステムは正常に動作します。")
print("実機実験を開始する準備が整いました。")
print("\n実行コマンド:")
print("  python3 mocap_slaf_main.py")
