#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
オブザーバーシミュレーション実行スクリプト
ドローンモデルとオブザーバーの動作をシミュレーションし、結果を可視化します。
"""

import os
import numpy as np
import time
import datetime
import argparse
from typing import Dict, List, Tuple, Any, Optional

from sim.models.drone import MultiDroneSimulator
from sim.models.observer import DroneObserver
from sim.utils.visualization import SimulationVisualizer, CSVLogger
from sim.utils.metrics import analyze_simulation_results, print_evaluation_summary


def run_simulation(config: Dict[str, Any]) -> Dict[str, Any]:
    """
    シミュレーション実行の主関数
    
    Args:
        config: シミュレーション設定
        
    Returns:
        シミュレーション結果
    """
    # パラメータ取得
    sim_time = config.get('sim_time', 30.0)  # シミュレーション時間（秒）
    dt = config.get('dt', 0.1)  # 時間ステップ
    num_drones = config.get('num_drones', 2)  # ドローン数
    fault_drone = config.get('fault_drone', None)  # 故障ドローン（0または1、Noneなら故障なし）
    fault_time = config.get('fault_time', 15.0)  # 故障発生時間（秒）
    fault_type = config.get('fault_type', 'position_drift')  # 故障の種類
    
    # 結果記録用
    results = {
        'positions': {i+1: [] for i in range(num_drones)},  # 実際の位置
        'attitudes': {i+1: [] for i in range(num_drones)},  # 実際の姿勢
        'quaternions': {i+1: [] for i in range(num_drones)},  # クォータニオン
        'observer_positions': {i+1: [] for i in range(num_drones)},  # オブザーバー推定位置
        'observer_velocities': {i+1: [] for i in range(num_drones)},  # オブザーバー推定速度
        'residuals': {i+1: [] for i in range(num_drones)},  # 残差
        'trust_values': {i+1: [] for i in range(num_drones)},  # 信頼度
        'fault_detected': {i+1: [] for i in range(num_drones)},  # 障害検出フラグ
        'leader_history': [],  # リーダーの履歴
        'rc_commands': {i+1: [] for i in range(num_drones)},  # RC制御コマンド
        'time_points': [],  # 時間点
        'fault_times': {},  # 故障時間
        'drone_ids': list(range(1, num_drones+1))  # ドローンID
    }
    
    # モデル・オブザーバーのインスタンス化
    simulator = MultiDroneSimulator(num_drones=num_drones, dt=dt)
    observer = DroneObserver(num_drones=num_drones, dt=dt)
    
    # ログフォルダの作成
    log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'sim_results')
    os.makedirs(log_dir, exist_ok=True)
    
    # ファイル名の生成
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"sim_log_{timestamp}_observer.csv"
    log_filepath = os.path.join(log_dir, filename)
    
    # CSVロガー初期化
    logger = CSVLogger(log_filepath)
    
    # 可視化の準備（オプション）
    visualizer = None
    if config.get('visualize', False):
        visualizer = SimulationVisualizer()
        visualizer.create_3d_plot()
        visualizer.create_time_series_plots()
    
    # 時間ステップの計算
    num_steps = int(sim_time / dt)
    
    # シミュレーションループ
    for step in range(num_steps):
        current_time = step * dt
        results['time_points'].append(current_time)
        
        # 故障の記録
        if fault_drone is not None and current_time >= fault_time:
            if step == int(fault_time / dt):  # 最初の故障発生時のみ記録
                print(f"Fault injected on drone {fault_drone+1} at time {current_time:.1f}s")
                results['fault_times'][fault_drone+1] = step
        
        # 飛行パターンの生成
        rc_commands = []
        for i in range(num_drones):
            # デバッグ用に全機体を静止状態に設定
            rc = [0, 0, 0, 0]
            
            rc_commands.append(rc)
            results['rc_commands'][i+1].append(rc)
            
        # 故障時のRCコマンド上書き処理
        if fault_drone is not None and current_time >= fault_time:
            if fault_type == 'input_hardover':
                # 故障機のRCコマンドを[100, 0, 0, 0]に強制固定
                hardover_rc = [100, 0, 0, 0]
                rc_commands[fault_drone] = hardover_rc
                results['rc_commands'][fault_drone+1][-1] = hardover_rc  # 結果記録も更新
                
                # デバッグ出力の追加（毎ステップ）
                print(f"[Runner] Timestep {step}: Input Hardover active for drone {fault_drone+1}, RC command set to {hardover_rc}")
        
        # シミュレーションに故障状態を設定（ただしinput_hardoverタイプ以外）
        if fault_drone is not None and current_time >= fault_time:
            if fault_type != 'input_hardover':  # input_hardoverはRCコマンドで処理済み
                simulator.set_drone_fault(fault_drone, fault_type, magnitude=1.0)
                
        # シミュレーション1ステップ実行
        sim_results = simulator.step(rc_commands)
        
        # 測定データの抽出
        positions = []
        for drone_id, data in sim_results.items():
            results['positions'][drone_id].append(data['position'].copy())
            results['attitudes'][drone_id].append(data['attitude'].copy())
            results['quaternions'][drone_id].append(data['quaternion'].copy())
            positions.append(data['position'])
        
        # オブザーバーの更新（RCコマンドも渡す）
        observer_results = observer.update(positions, rc_commands)
        
        # オブザーバー結果の記録
        for drone_id, data in observer_results.items():
            results['observer_positions'][drone_id].append(data['position'].copy())
            results['observer_velocities'][drone_id].append(data['velocity'].copy())
            results['residuals'][drone_id].append(np.linalg.norm(data['residual']))
            results['trust_values'][drone_id].append(data['trust'])
            results['fault_detected'][drone_id].append(data['fault_detected'])
        
        # リーダー情報の更新
        results['leader_history'].append(observer.get_leader_index())
        
        # CSVログの記録
        logger.log_simulation_step(
            current_time,
            {drone_id: {'position': data['position'], 'attitude': data['attitude'], 'quaternion': data['quaternion']}
             for drone_id, data in sim_results.items()},
            observer_results,
            {drone_id: rc for drone_id, rc in zip(range(1, num_drones+1), rc_commands)}
        )
        
        # 可視化（オプション）
        if visualizer and step % 5 == 0:  # 5ステップごとに更新
            drone_positions = [sim_results[i+1]['position'] for i in range(num_drones)]
            drone_ids = list(range(1, num_drones+1))
            
            visualizer.plot_drone_positions(drone_positions, drone_ids)
            
            # 進捗表示
            if step % 50 == 0:
                print(f"Simulation progress: {step}/{num_steps} steps ({current_time:.1f}s/{sim_time:.1f}s)")
    
    # シミュレーション完了メッセージ
    print(f"\nSimulation completed. Results saved to {log_filepath}")
    
    # 結果の評価
    evaluation_data = {
        'drone_ids': results['drone_ids'],
        'positions': results['positions'],
        'fault_times': results['fault_times']
    }
    
    observer_data = {
        'positions': results['observer_positions'],
        'trust_values': results['trust_values'],
        'fault_times': {drone_id: [i for i, flag in enumerate(results['fault_detected'][drone_id]) if flag]
                        for drone_id in results['drone_ids']}
    }
    
    evaluation_results = analyze_simulation_results(evaluation_data, observer_data)
    print_evaluation_summary(evaluation_results)
    
    # 最終結果の可視化
    if visualizer:
        # 軌跡プロット
        trajectory_data = {
            drone_id: {
                'positions': results['positions'][drone_id]
            } for drone_id in results['drone_ids']
        }
        
        observer_plot_data = {
            drone_id: {
                'residuals': results['residuals'][drone_id],
                'trust_values': results['trust_values'][drone_id]
            } for drone_id in results['drone_ids']
        }
        observer_plot_data[1]['leader_history'] = results['leader_history']
        
        visualizer.update_trajectory_plot(
            results['time_points'],
            trajectory_data,
            observer_plot_data
        )
        
        # プロット保存
        plot_filepath = os.path.join(log_dir, f"sim_plot_{timestamp}.png")
        visualizer.save_plots(plot_filepath)
        
        # プロット表示
        visualizer.show()
    
    return results


def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='ドローンオブザーバーシミュレーション')
    parser.add_argument('--time', type=float, default=30.0, help='シミュレーション時間（秒）')
    parser.add_argument('--dt', type=float, default=0.1, help='時間ステップ（秒）')
    parser.add_argument('--fault', type=int, default=None, help='故障ドローン（1または2、指定なしは故障なし）')
    parser.add_argument('--fault-time', type=float, default=15.0, help='故障発生時間（秒）')
    parser.add_argument('--fault-type', type=str, default='position_drift',
                        choices=['position_drift', 'attitude_bias', 'sensor_noise', 
                                'hardover_position', 'hardover_attitude', 'input_hardover'],
                        help='故障の種類（position_drift=位置ドリフト, attitude_bias=姿勢バイアス, \
                              sensor_noise=センサノイズ増加, hardover_position=位置強制置換, \
                              hardover_attitude=姿勢強制置換, input_hardover=入力値強制固定）')
    parser.add_argument('--no-viz', action='store_true', help='可視化を無効にする')
    
    args = parser.parse_args()
    
    # 設定の準備
    config = {
        'sim_time': args.time,
        'dt': args.dt,
        'num_drones': 2,
        'visualize': not args.no_viz,
        'fault_type': args.fault_type
    }
    
    # 故障ドローンの設定（1→0インデックス変換）
    if args.fault is not None:
        config['fault_drone'] = args.fault - 1
        config['fault_time'] = args.fault_time
    
    # シミュレーション実行
    print("\n===== ドローンオブザーバーシミュレーション開始 =====")
    print(f"シミュレーション時間: {config['sim_time']}秒")
    print(f"時間ステップ: {config['dt']}秒")
    print(f"ドローン数: {config['num_drones']}")
    if 'fault_drone' in config:
        print(f"故障ドローン: {args.fault}号機")
        print(f"故障発生時間: {config['fault_time']}秒")
        print(f"故障タイプ: {config['fault_type']}")
    print("===============================================\n")
    
    run_simulation(config)


if __name__ == "__main__":
    main()
