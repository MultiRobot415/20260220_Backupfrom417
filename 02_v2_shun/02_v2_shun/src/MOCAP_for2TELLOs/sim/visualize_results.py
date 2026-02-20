#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
シミュレーション結果可視化スクリプト
CSVログを読み込み、位置・残差・信頼度などの時系列データを可視化します。
"""

import os
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
import argparse
from datetime import datetime
import pandas as pd
from typing import Dict, List, Tuple, Any

# 自作モジュールのインポート
from utils.visualization import SimulationVisualizer


class ResultAnalyzer:
    """
    シミュレーション結果の分析・可視化を行うクラス
    """
    
    def __init__(self, csv_path: str):
        """
        結果アナライザーの初期化
        
        Args:
            csv_path: 分析対象のCSVログファイルパス
        """
        self.csv_path = csv_path
        self.data = None
        self.drone_ids = []
        self.time_points = []
        
        # データの読み込み
        self._load_data()
    
    def _load_data(self):
        """CSVデータをロードしてDataFrameに変換"""
        try:
            # CSVファイルをパンダスで読み込み
            self.data = pd.read_csv(self.csv_path)
            
            # タイムスタンプを時間軸に変換
            self.data['seconds'] = 0.0
            start_time = None
            
            # タイムスタンプからの経過時間を計算
            for i, timestamp_str in enumerate(self.data['timestamp']):
                try:
                    timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f")
                    if start_time is None:
                        start_time = timestamp
                    delta = (timestamp - start_time).total_seconds()
                    self.data.at[i, 'seconds'] = delta
                except:
                    pass
            
            # ドローンIDの一覧を取得
            self.drone_ids = sorted(self.data['drone_id'].unique())
            
            # 時間点の配列を取得
            self.time_points = sorted(self.data['seconds'].unique())
            
            print(f"データ読み込み完了: {len(self.time_points)}タイムステップ, {len(self.drone_ids)}機のドローン")
            print(f"時間範囲: {self.time_points[0]:.1f}秒 - {self.time_points[-1]:.1f}秒")
            
        except Exception as e:
            print(f"データ読み込みエラー: {e}")
            sys.exit(1)
    
    def plot_position_time_series(self, warmup_time: float = 5.0):
        """位置の時系列データをプロット
        
        Args:
            warmup_time: 初期化時間（秒）- この時間以前のデータは表示しない
        """
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        
        axes[0].set_title(f"Position X (excluding first {warmup_time}s)")
        axes[1].set_title(f"Position Y (excluding first {warmup_time}s)")
        axes[2].set_title(f"Position Z (excluding first {warmup_time}s)")
        
        for drone_id in self.drone_ids:
            # ドローン別にデータをフィルタリング
            drone_data = self.data[self.data['drone_id'] == drone_id]
            
            # 初期化時間（ウォームアップ）以降のデータのみを使用する
            drone_data = drone_data[drone_data['seconds'] >= warmup_time]
            
            # 位置データをプロット
            axes[0].plot(drone_data['seconds'], drone_data['x'], 
                         label=f'Drone {drone_id}')
            axes[1].plot(drone_data['seconds'], drone_data['y'])
            axes[2].plot(drone_data['seconds'], drone_data['z'])
            
            # 目標位置があればプロット
            if 'target_x' in drone_data.columns:
                axes[0].plot(drone_data['seconds'], drone_data['target_x'], 
                             linestyle='--', alpha=0.7,
                             label=f'Target {drone_id}')
                axes[1].plot(drone_data['seconds'], drone_data['target_y'], 
                             linestyle='--', alpha=0.7)
                axes[2].plot(drone_data['seconds'], drone_data['target_z'], 
                             linestyle='--', alpha=0.7)
        
        # グラフの装飾
        axes[0].set_ylabel("X [m]")
        axes[1].set_ylabel("Y [m]")
        axes[2].set_ylabel("Z [m]")
        axes[2].set_xlabel("Time [s]")
        
        # 凡例の表示
        axes[0].legend()
        
        # 障害発生ラインの表示（leader_statusが変化した点）
        for drone_id in self.drone_ids:
            drone_data = self.data[self.data['drone_id'] == drone_id]
            status_changes = []
            
            # 故障状態の変化点を探索
            for i in range(1, len(drone_data)):
                if drone_data.iloc[i]['leader_status'] != drone_data.iloc[i-1]['leader_status'] and drone_data.iloc[i]['leader_status'] > 0:
                    status_changes.append(drone_data.iloc[i]['seconds'])
            
            # 故障発生を示す縦線を表示
            for time_point in status_changes:
                for ax in axes:
                    ax.axvline(x=time_point, color='red', linestyle='--', alpha=0.7)
                    ax.text(time_point, ax.get_ylim()[1] * 0.9, f"Fault D{drone_id}", 
                           rotation=90, verticalalignment='top')
        
        plt.tight_layout()
        
        # グラフを保存
        base_name = os.path.splitext(os.path.basename(self.csv_path))[0]
        save_path = os.path.join(os.path.dirname(self.csv_path), f"{base_name}_positions.png")
        plt.savefig(save_path, dpi=300)
        print(f"位置時系列グラフを保存: {save_path}")
        
        return fig
    
    def plot_observer_metrics(self, warmup_time: float = 5.0):
        """オブザーバーの残差・信頼度・エラーをプロット
        
        Args:
            warmup_time: 初期化時間（秒）- この時間以前のデータは表示しない
        """
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        
        axes[0].set_title("Observer Position Error (excluding first 5s)")
        axes[1].set_title("Observer Residual (excluding first 5s)")
        axes[2].set_title("Trust Metric (excluding first 5s)")
        
        for drone_id in self.drone_ids:
            # ドローン別にデータをフィルタリング
            drone_data = self.data[self.data['drone_id'] == drone_id]
            
            # 初期化時間（ウォームアップ）以降のデータのみを使用する
            drone_data = drone_data[drone_data['seconds'] >= warmup_time]
            
            # 位置誤差をプロット（観測値 - 推定値）
            if all(col in drone_data.columns for col in ['x', 'obs_state_x', 'y', 'obs_state_y', 'z', 'obs_state_z']):
                error_x = drone_data['x'] - drone_data['obs_state_x']
                error_y = drone_data['y'] - drone_data['obs_state_y']
                error_z = drone_data['z'] - drone_data['obs_state_z']
                
                # 位置誤差のノルム
                error_norm = np.sqrt(error_x**2 + error_y**2 + error_z**2)
                axes[0].plot(drone_data['seconds'], error_norm, label=f'Position Error D{drone_id}')
            
            # 残差をプロット
            if all(col in drone_data.columns for col in ['obs_error_x', 'obs_error_y', 'obs_error_z']):
                residual_norm = np.sqrt(drone_data['obs_error_x']**2 + 
                                       drone_data['obs_error_y']**2 + 
                                       drone_data['obs_error_z']**2)
                axes[1].plot(drone_data['seconds'], residual_norm, label=f'Residual D{drone_id}')
                
                # 閾値ラインを表示（仮の値0.08）
                axes[1].axhline(y=0.08, color='red', linestyle='--', alpha=0.5, label='Threshold')
            
            # 信頼度をプロット
            if 'trust' in drone_data.columns:
                axes[2].plot(drone_data['seconds'], drone_data['trust'], label=f'Trust D{drone_id}')
        
        # グラフの装飾
        axes[0].set_ylabel("Error [m]")
        axes[1].set_ylabel("Residual Norm")
        axes[2].set_ylabel("Trust Value")
        axes[2].set_xlabel("Time [s]")
        
        # Y軸の範囲を設定
        axes[2].set_ylim(0, 1.1)
        
        # 凡例の表示
        for ax in axes:
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        # 障害発生ラインの表示（leader_statusが変化した点）
        for drone_id in self.drone_ids:
            drone_data = self.data[self.data['drone_id'] == drone_id]
            status_changes = []
            
            # 故障状態の変化点を探索
            for i in range(1, len(drone_data)):
                if drone_data.iloc[i]['leader_status'] != drone_data.iloc[i-1]['leader_status'] and drone_data.iloc[i]['leader_status'] > 0:
                    status_changes.append(drone_data.iloc[i]['seconds'])
            
            # 故障発生を示す縦線を表示
            for time_point in status_changes:
                for ax in axes:
                    ax.axvline(x=time_point, color='red', linestyle='--', alpha=0.7)
                    ax.text(time_point, ax.get_ylim()[1] * 0.9, f"Fault D{drone_id}", 
                           rotation=90, verticalalignment='top')
        
        plt.tight_layout()
        
        # グラフを保存
        base_name = os.path.splitext(os.path.basename(self.csv_path))[0]
        save_path = os.path.join(os.path.dirname(self.csv_path), f"{base_name}_observer.png")
        plt.savefig(save_path, dpi=300)
        print(f"オブザーバーメトリクスグラフを保存: {save_path}")
        
        return fig
    
    def plot_3d_trajectory(self, warmup_time: float = 5.0):
        """3D軌跡のプロット
        
        Args:
            warmup_time: 初期化時間（秒）- この時間以前のデータは表示しない
        """
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title(f"3D Drone Trajectory (excluding first {warmup_time}s)")
        
        for drone_id in self.drone_ids:
            # ドローン別にデータをフィルタリング
            drone_data = self.data[self.data['drone_id'] == drone_id]
            
            # 初期化時間（ウォームアップ）以降のデータのみを使用する
            drone_data = drone_data[drone_data['seconds'] >= warmup_time]
            
            # 位置データをプロット
            ax.plot(drone_data['x'], drone_data['y'], drone_data['z'], 
                   label=f'Drone {drone_id}')
            
            # 故障検出時点でマーカーを表示
            fault_points = drone_data[drone_data['leader_status'] > 0]
            if not fault_points.empty:
                ax.scatter(fault_points['x'], fault_points['y'], fault_points['z'], 
                          color='red', marker='x', s=100, label=f'Fault D{drone_id}')
        
        # グラフの装飾
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.legend()
        
        # グラフを保存
        base_name = os.path.splitext(os.path.basename(self.csv_path))[0]
        save_path = os.path.join(os.path.dirname(self.csv_path), f"{base_name}_3d.png")
        plt.savefig(save_path, dpi=300)
        print(f"3D軌跡グラフを保存: {save_path}")
        
        return fig

    def plot_attitude(self, warmup_time: float = 5.0):
        """姿勢角度のプロット
        
        Args:
            warmup_time: 初期化時間（秒）- この時間以前のデータは表示しない
        """
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        
        axes[0].set_title(f"Tilt X (Roll) (excluding first {warmup_time}s)")
        axes[1].set_title(f"Tilt Y (Pitch) (excluding first {warmup_time}s)")
        
        for drone_id in self.drone_ids:
            # ドローン別にデータをフィルタリング
            drone_data = self.data[self.data['drone_id'] == drone_id]
            
            # 初期化時間（ウォームアップ）以降のデータのみを使用する
            drone_data = drone_data[drone_data['seconds'] >= warmup_time]
            
            # 姿勢データをプロット
            if 'tilt_x' in drone_data.columns and 'tilt_y' in drone_data.columns:
                axes[0].plot(drone_data['seconds'], drone_data['tilt_x'], 
                            label=f'Roll D{drone_id}')
                axes[1].plot(drone_data['seconds'], drone_data['tilt_y'], 
                            label=f'Pitch D{drone_id}')
        
        # グラフの装飾
        axes[0].set_ylabel("Roll [deg]")
        axes[1].set_ylabel("Pitch [deg]")
        axes[1].set_xlabel("Time [s]")
        
        # 凡例の表示
        for ax in axes:
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # グラフを保存
        base_name = os.path.splitext(os.path.basename(self.csv_path))[0]
        save_path = os.path.join(os.path.dirname(self.csv_path), f"{base_name}_attitude.png")
        plt.savefig(save_path, dpi=300)
        print(f"姿勢角度グラフを保存: {save_path}")
        
        return fig
    
    def analyze_all(self, show_plots: bool = True, warmup_time: float = 5.0):
        """すべての分析を実行して結果を表示
        
        Args:
            show_plots: プロットを表示するかどうか
            warmup_time: 初期化時間（秒）- この時間以前のデータは除外
        """
        # 各種グラフの生成
        fig_3d = self.plot_3d_trajectory(warmup_time)
        
        # 位置時系列グラフ
        fig_pos = self.plot_position_time_series(warmup_time)
        
        # オブザーバーメトリクスグラフ
        fig_metrics = self.plot_observer_metrics(warmup_time)
        
        # 姿勢角度グラフ
        fig_attitude = self.plot_attitude(warmup_time)
        
        if show_plots:
            plt.show()
        
        return [fig_pos, fig_metrics, fig_3d, fig_attitude]


def get_latest_csv(directory: str) -> str:
    """
    指定ディレクトリ内の最新のCSVファイルを取得
    
    Args:
        directory: 検索対象ディレクトリ
        
    Returns:
        最新CSVファイルの絶対パス
    """
    csv_files = []
    for file in os.listdir(directory):
        if file.endswith('.csv'):
            full_path = os.path.join(directory, file)
            csv_files.append((full_path, os.path.getmtime(full_path)))
    
    if not csv_files:
        return None
    
    # 最新ファイルを取得
    latest_file = sorted(csv_files, key=lambda x: x[1], reverse=True)[0][0]
    return latest_file


def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='シミュレーション結果の可視化')
    parser.add_argument('--file', type=str, help='分析対象のCSVファイルパス')
    parser.add_argument('--latest', action='store_true', help='最新のCSVファイルを使用')
    parser.add_argument('--no-show', action='store_true', help='プロットを表示しない（保存のみ）')
    
    args = parser.parse_args()
    
    # 結果ディレクトリのパス
    results_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'sim_results')
    
    # CSVファイルの選択
    csv_path = None
    if args.file:
        csv_path = args.file
    else:  # デフォルトで最新を選択
        csv_path = get_latest_csv(results_dir)
    
    if not csv_path or not os.path.exists(csv_path):
        print("CSVファイルが見つかりません。")
        return
    
    print(f"分析対象ファイル: {csv_path}")
    
    # 結果の分析と可視化
    analyzer = ResultAnalyzer(csv_path)
    analyzer.analyze_all(not args.no_show)


if __name__ == "__main__":
    main()
