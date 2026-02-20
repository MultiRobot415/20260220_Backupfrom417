#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
シミュレーション結果の可視化ユーティリティ
シミュレーションデータの可視化機能を提供します。
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Dict, Any, Optional, Tuple


class SimulationVisualizer:
    """
    シミュレーション結果の可視化クラス
    """
    
    def __init__(self):
        """可視化クラスの初期化"""
        self.fig = None
        self.axes = {}
        self.lines = {}
        self.scatter_points = {}
        
        # プロットカラー
        self.drone_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    
    def create_3d_plot(self):
        """3Dプロット環境の作成"""
        self.fig = plt.figure(figsize=(10, 8))
        ax = self.fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title('Drone Trajectory Simulation')
        
        # 軸の範囲を設定
        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])
        ax.set_zlim([0, 2.0])
        
        self.axes['3d'] = ax
        return self.fig
    
    def create_time_series_plots(self):
        """時系列データ用のプロット作成"""
        self.fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        
        # 位置プロット
        self.axes['position_x'] = axes[0, 0]
        self.axes['position_x'].set_title('X Position')
        self.axes['position_x'].set_xlabel('Time [s]')
        self.axes['position_x'].set_ylabel('X [m]')
        
        self.axes['position_y'] = axes[1, 0]
        self.axes['position_y'].set_title('Y Position')
        self.axes['position_y'].set_xlabel('Time [s]')
        self.axes['position_y'].set_ylabel('Y [m]')
        
        self.axes['position_z'] = axes[2, 0]
        self.axes['position_z'].set_title('Z Position')
        self.axes['position_z'].set_xlabel('Time [s]')
        self.axes['position_z'].set_ylabel('Z [m]')
        
        # 観測値と推定値の残差
        self.axes['residual'] = axes[0, 1]
        self.axes['residual'].set_title('Observer Residual')
        self.axes['residual'].set_xlabel('Time [s]')
        self.axes['residual'].set_ylabel('Residual Norm')
        
        # 信頼度メトリクス
        self.axes['trust'] = axes[1, 1]
        self.axes['trust'].set_title('Trust Metric')
        self.axes['trust'].set_xlabel('Time [s]')
        self.axes['trust'].set_ylabel('Trust Value')
        
        # リーダー指標
        self.axes['leader'] = axes[2, 1]
        self.axes['leader'].set_title('Leader Index')
        self.axes['leader'].set_xlabel('Time [s]')
        self.axes['leader'].set_ylabel('Drone ID')
        self.axes['leader'].set_yticks([1, 2])
        
        plt.tight_layout()
        return self.fig
    
    def plot_drone_positions(self, positions: List[np.ndarray], drone_ids: List[int]):
        """
        ドローン位置の3Dプロット更新
        
        Args:
            positions: 位置データのリスト
            drone_ids: ドローンIDのリスト
        """
        if '3d' not in self.axes:
            self.create_3d_plot()
        
        ax = self.axes['3d']
        
        # 既存のポイントをクリア
        for key in list(self.scatter_points.keys()):
            if key.startswith('drone_'):
                if self.scatter_points[key] in ax.collections:
                    self.scatter_points[key].remove()
                del self.scatter_points[key]
        
        # 各ドローンの位置をプロット
        for i, (pos, drone_id) in enumerate(zip(positions, drone_ids)):
            color = self.drone_colors[i % len(self.drone_colors)]
            marker = 'o' if drone_id == 1 else '^'  # リーダーは円、フォロワーは三角
            
            scatter = ax.scatter(
                pos[0], pos[1], pos[2],
                color=color, marker=marker, s=100,
                label=f'Drone {drone_id}'
            )
            self.scatter_points[f'drone_{drone_id}'] = scatter
        
        # 凡例の更新
        ax.legend()
        
        # 描画更新
        self.fig.canvas.draw_idle()
        plt.pause(0.01)
    
    def update_trajectory_plot(self, time_points: List[float], 
                             trajectory_data: Dict[int, Dict[str, List[np.ndarray]]],
                             observer_data: Optional[Dict[int, Dict[str, Any]]] = None):
        """
        時系列データプロットの更新
        
        Args:
            time_points: 時間データ
            trajectory_data: ドローンIDごとの軌跡データ辞書
            observer_data: オブザーバーデータ（オプション）
        """
        if not self.axes.get('position_x'):
            self.create_time_series_plots()
        
        # 既存のラインをクリア
        for key in list(self.lines.keys()):
            if self.lines[key] in self.axes['position_x'].lines:
                self.lines[key].remove()
            del self.lines[key]
        
        # 各ドローンの軌跡を描画
        for drone_id, data in trajectory_data.items():
            color = self.drone_colors[(drone_id - 1) % len(self.drone_colors)]
            
            # X位置
            line, = self.axes['position_x'].plot(
                time_points, [pos[0] for pos in data['positions']],
                color=color, label=f'Drone {drone_id}'
            )
            self.lines[f'x_drone_{drone_id}'] = line
            
            # Y位置
            line, = self.axes['position_y'].plot(
                time_points, [pos[1] for pos in data['positions']],
                color=color
            )
            self.lines[f'y_drone_{drone_id}'] = line
            
            # Z位置
            line, = self.axes['position_z'].plot(
                time_points, [pos[2] for pos in data['positions']],
                color=color
            )
            self.lines[f'z_drone_{drone_id}'] = line
        
        # オブザーバーデータがある場合は残差と信頼度をプロット
        if observer_data:
            for drone_id, data in observer_data.items():
                color = self.drone_colors[(drone_id - 1) % len(self.drone_colors)]
                
                # 残差
                if 'residuals' in data:
                    line, = self.axes['residual'].plot(
                        time_points, data['residuals'],
                        color=color, label=f'Drone {drone_id}'
                    )
                    self.lines[f'residual_drone_{drone_id}'] = line
                
                # 信頼度
                if 'trust_values' in data:
                    line, = self.axes['trust'].plot(
                        time_points, data['trust_values'],
                        color=color, label=f'Drone {drone_id}'
                    )
                    self.lines[f'trust_drone_{drone_id}'] = line
            
            # リーダー指標
            if 'leader_history' in observer_data.get(1, {}):
                leader_history = observer_data[1]['leader_history']
                self.axes['leader'].step(
                    time_points, [lid + 1 for lid in leader_history],
                    where='post', color='black'
                )
        
        # 凡例の更新
        for ax_name in ['position_x', 'residual', 'trust']:
            self.axes[ax_name].legend()
        
        # グラフの表示
        plt.tight_layout()
        self.fig.canvas.draw_idle()
        plt.pause(0.01)
    
    def save_plots(self, filepath: str):
        """
        プロットを画像として保存
        
        Args:
            filepath: 保存先ファイルパス
        """
        self.fig.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {filepath}")
    
    def show(self):
        """プロットを表示"""
        plt.show()


class CSVLogger:
    """
    シミュレーション結果をCSVファイルに記録するクラス
    実機実験と同じ形式でログを保存
    """
    
    def __init__(self, filename: str):
        """
        CSVロガーの初期化
        
        Args:
            filename: 出力CSVファイルのパス
        """
        self.filename = filename
        self.header_written = False
    
    def write_header(self):
        """CSVヘッダーの書き込み"""
        import csv
        
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'drone_id', 'role', 'mode', 'x', 'y', 'z', 
                'target_x', 'target_y', 'target_z', 'error_x', 'error_y', 'error_z', 
                'rc_lr', 'rc_fb', 'rc_ud', 'rc_yaw', 'tilt_x', 'tilt_y',
                'trust', 'obs_error_x', 'obs_error_y', 'obs_error_z',
                'obs_state_x', 'obs_state_y', 'obs_state_z',
                'battery', 'height', 'leader_status', 'exec_time'
            ])
        
        self.header_written = True
    
    def log_simulation_step(self, timestamp: float, drone_data: Dict[int, Dict], 
                           observer_data: Optional[Dict] = None, rc_commands: Optional[Dict] = None):
        """
        シミュレーションステップの記録
        
        Args:
            timestamp: 現在のシミュレーション時間
            drone_data: ドローンの状態データ
            observer_data: オブザーバーの出力データ
            rc_commands: RC制御コマンド
        """
        import csv
        import time
        
        if not self.header_written:
            self.write_header()
        
        with open(self.filename, 'a', newline='') as f:
            writer = csv.writer(f)
            
            # 実際の日時を文字列化（シミュレーションでもこれを使う）
            datetime_str = time.strftime("%Y-%m-%d %H:%M:%S.") + f"{(timestamp % 1) * 1000:03.0f}"
            
            for drone_id, data in drone_data.items():
                # 位置データ
                position = data.get('position', np.zeros(3))
                x, y, z = position
                
                # 目標位置（ない場合はゼロ）
                target = data.get('target', np.zeros(3))
                tx, ty, tz = target
                
                # 姿勢データから傾斜角度を取得
                attitude = data.get('attitude', np.zeros(3))
                tilt_x, tilt_y, _ = attitude  # roll, pitch, yaw
                
                # エラー計算
                ex = tx - x
                ey = ty - y
                ez = tz - z
                
                # RCコマンド（ない場合はゼロ）
                rc = rc_commands.get(drone_id, [0, 0, 0, 0]) if rc_commands else [0, 0, 0, 0]
                lr, fb, ud, yaw = rc
                
                # オブザーバーデータの取得
                trust = 1.0
                obs_error_x, obs_error_y, obs_error_z = 0.0, 0.0, 0.0
                obs_state_x, obs_state_y, obs_state_z = x, y, z  # デフォルトは実際の位置
                leader_status = 0
                
                if observer_data and drone_id in observer_data:
                    obs_data = observer_data[drone_id]
                    trust = obs_data.get('trust', 1.0)
                    
                    if 'residual' in obs_data:
                        obs_error_x, obs_error_y, obs_error_z = obs_data['residual']
                    
                    if 'position' in obs_data:
                        obs_state_x, obs_state_y, obs_state_z = obs_data['position']
                    
                    # リーダー状態（0=正常、1=異常疑い、2=故障）
                    leader_status = 2 if obs_data.get('fault_detected', False) else 0
                
                # 役割の決定（IDに基づく簡易版）
                role = "leader" if drone_id == 1 else "follower"
                
                # 実行時間（シミュレーションでは0.001秒として記録）
                exec_time = 1.0
                
                # バッテリーと高度は固定値
                battery = 100
                height = int(z * 100)  # メートルからセンチメートルへ変換
                
                # CSVへの書き込み
                writer.writerow([
                    datetime_str, drone_id, role, 'auto', x, y, z,
                    tx, ty, tz, ex, ey, ez,
                    lr, fb, ud, yaw, tilt_x, tilt_y,
                    trust, obs_error_x, obs_error_y, obs_error_z,
                    obs_state_x, obs_state_y, obs_state_z,
                    battery, height, leader_status, exec_time
                ])
