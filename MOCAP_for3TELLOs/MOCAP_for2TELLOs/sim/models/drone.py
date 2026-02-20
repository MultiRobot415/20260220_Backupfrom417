#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ドローンシミュレーションモデル
ドローンの動力学モデルとRCコマンドの処理を提供します。
"""

import numpy as np
from typing import Tuple, List, Dict


class DroneModel:
    """
    ドローンの動力学モデル
    簡易的なドローンの運動方程式と制御入力の処理を実装
    """
    
    def __init__(self, dt: float = 0.1, drone_id: int = 0):
        """
        ドローンモデルの初期化
        
        Args:
            dt: シミュレーションの時間ステップ（秒）
            drone_id: ドローンID
        """
        # 状態変数
        self.position = np.zeros(3)  # [x, y, z]
        self.velocity = np.zeros(3)  # [vx, vy, vz]
        self.attitude = np.zeros(3)  # [roll, pitch, yaw]
        self.dt = dt
        self.drone_id = drone_id
        
        # RCコマンドの影響係数
        self.MAX_TILT_ANGLE = 15.0  # 最大傾斜角度（度）
        self.MAX_VELOCITY = 0.5     # 最大速度（m/s）
        
        # 動力学パラメータ
        self.velocity_decay = 0.95  # 速度減衰係数
        self.attitude_decay = 0.8   # 姿勢減衰係数
        
        # ノイズパラメータ
        self.position_noise_std = 0.01  # 位置ノイズ標準偏差
        self.attitude_noise_std = 0.5   # 姿勢ノイズ標準偏差
        
        # 状態履歴
        self.position_history = []
        self.attitude_history = []
        
        # 障害シミュレーション用
        self.fault_active = False
        self.fault_type = None
        self.fault_magnitude = 0.0
    
    def reset(self, initial_position: np.ndarray = None):
        """状態のリセット"""
        if initial_position is not None:
            self.position = initial_position.copy()
        else:
            self.position = np.zeros(3)
        
        self.velocity = np.zeros(3)
        self.attitude = np.zeros(3)
        self.position_history = []
        self.attitude_history = []
        self.fault_active = False
        self.fault_type = None
        self.fault_magnitude = 0.0
    
    def step(self, rc_command: List[int]) -> Tuple[np.ndarray, np.ndarray]:
        """
        時間ステップを進めて状態を更新
        
        Args:
            rc_command: RCコマンド [roll, pitch, throttle, yaw]
            
        Returns:
            (位置, 姿勢)のタプル
        """
        # RCコマンドはそのまま使用する（上位のsimulation_runner.pyで上書き処理済み）
        # デバッグ用に受け取ったRCコマンドを出力
        print(f"[Drone {self.drone_id}] Processing RC command: {rc_command}")
        
        # RCコマンドの処理
        roll_cmd, pitch_cmd, throttle_cmd, yaw_cmd = rc_command
        
        # 目標姿勢角を計算（度）
        target_roll = roll_cmd * self.MAX_TILT_ANGLE / 100.0
        target_pitch = pitch_cmd * self.MAX_TILT_ANGLE / 100.0
        
        # 姿勢の更新（単純な一次遅れ系でモデル化）
        self.attitude[0] = self.attitude_decay * self.attitude[0] + (1 - self.attitude_decay) * target_roll
        self.attitude[1] = self.attitude_decay * self.attitude[1] + (1 - self.attitude_decay) * target_pitch
        self.attitude[2] += yaw_cmd * 0.5 * self.dt  # ヨーは角速度に比例
        
        # 姿勢に基づく加速度の計算
        # ロール、ピッチから水平面での加速度を計算
        ax = np.sin(np.radians(self.attitude[1])) * self.MAX_VELOCITY
        ay = -np.sin(np.radians(self.attitude[0])) * self.MAX_VELOCITY
        az = (throttle_cmd / 100.0) * 0.2  # 上下の加速度
        
        # 速度の更新
        self.velocity[0] = self.velocity_decay * self.velocity[0] + ax * self.dt
        self.velocity[1] = self.velocity_decay * self.velocity[1] + ay * self.dt
        self.velocity[2] = self.velocity_decay * self.velocity[2] + az * self.dt
        
        # 位置の更新
        self.position += self.velocity * self.dt
        
        # 障害シミュレーション
        if self.fault_active:
            self._apply_fault()
        
        # ノイズの追加
        position_with_noise = self.position + np.random.normal(0, self.position_noise_std, 3)
        attitude_with_noise = self.attitude + np.random.normal(0, self.attitude_noise_std, 3)
        
        # 履歴の更新
        self.position_history.append(position_with_noise.copy())
        self.attitude_history.append(attitude_with_noise.copy())
        
        # 結果を返す
        return position_with_noise, attitude_with_noise
    
    def get_quaternion(self) -> np.ndarray:
        """
        現在の姿勢をクォータニオンに変換
        
        Returns:
            クォータニオン [w, x, y, z]
        """
        # オイラー角（ロール、ピッチ、ヨー）からクォータニオンに変換
        roll, pitch, yaw = np.radians(self.attitude)
        
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return np.array([w, x, y, z])
    
    def set_fault(self, fault_type: str, magnitude: float = 1.0):
        """
        障害を設定
        
        Args:
            fault_type: 障害の種類（'position_drift', 'attitude_bias', 'sensor_noise'）
            magnitude: 障害の大きさ
        """
        self.fault_active = True
        self.fault_type = fault_type
        self.fault_magnitude = magnitude
    
    def clear_fault(self):
        """障害をクリア"""
        self.fault_active = False
        self.fault_type = None
        self.fault_magnitude = 0.0
    
    def _apply_fault(self):
        """設定された障害を適用"""
        if self.fault_type == 'position_drift':
            # 位置のドリフト
            drift = np.array([0.01, 0.01, 0.0]) * self.fault_magnitude
            self.position += drift
        
        elif self.fault_type == 'attitude_bias':
            # 姿勢のバイアス
            bias = np.array([1.0, 0.0, 0.0]) * self.fault_magnitude
            self.attitude += bias
        
        elif self.fault_type == 'sensor_noise':
            # センサーノイズの増加
            self.position_noise_std = 0.05 * self.fault_magnitude
            self.attitude_noise_std = 1.0 * self.fault_magnitude
            
        elif self.fault_type == 'hardover_position':
            # 位置のハードオーバー故障（位置を固定値に置換）
            # 現在位置から固定のオフセットを持つ位置に強制移動
            hardover_offset = np.array([0.5, 0.5, 0.0]) * self.fault_magnitude
            self.position = self.position_history[0] + hardover_offset  # 初期位置からのオフセット
            # 速度をゼロに設定（位置が固定されるため）
            self.velocity = np.zeros(3)
            
        elif self.fault_type == 'hardover_attitude':
            # 姿勢のハードオーバー故障（姿勢角を固定値に置換）
            # 姿勢を固定値に強制設定
            hardover_attitude = np.array([15.0, 0.0, 0.0]) * self.fault_magnitude  # 15度のロール
            self.attitude = hardover_attitude
            
        elif self.fault_type == 'input_hardover':
            # 入力値（RCコマンド）を固定値に置換する故障
            # この故障は_apply_fault内では何もせず、updateメソッド内でRCコマンドを置換する
            pass  # この故障はコマンド処理で実装されているため、ここでは何もしない


class MultiDroneSimulator:
    """
    複数ドローンのシミュレーションを管理するクラス
    """
    
    def __init__(self, num_drones: int = 2, dt: float = 0.1):
        """
        マルチドローンシミュレータの初期化
        
        Args:
            num_drones: シミュレーションするドローンの数
            dt: シミュレーションの時間ステップ
        """
        self.drones = [DroneModel(dt, drone_id=i+1) for i in range(num_drones)]
        self.num_drones = num_drones
        self.dt = dt
        self.time = 0.0
        
        # 初期位置を設定（ドローン間で異なる位置）
        initial_positions = [
            np.array([0.0, 0.0, 1.0]),  # ドローン1
            np.array([0.5, 0.0, 1.0])   # ドローン2
        ]
        
        for i, drone in enumerate(self.drones):
            if i < len(initial_positions):
                drone.reset(initial_positions[i])
    
    def step(self, rc_commands: List[List[int]]) -> Dict:
        """
        すべてのドローンのシミュレーションステップを実行
        
        Args:
            rc_commands: 各ドローンのRCコマンドのリスト
            
        Returns:
            シミュレーション結果の辞書
        """
        results = {}
        
        for i, (drone, rc_command) in enumerate(zip(self.drones, rc_commands)):
            position, attitude = drone.step(rc_command)
            quaternion = drone.get_quaternion()
            
            drone_id = i + 1  # ドローンIDは1から始まる
            results[drone_id] = {
                'position': position,
                'attitude': attitude,
                'quaternion': quaternion,
                'velocity': drone.velocity.copy()
            }
        
        self.time += self.dt
        return results
    
    def set_drone_fault(self, drone_idx: int, fault_type: str, magnitude: float = 1.0):
        """
        指定したドローンに障害を設定
        
        Args:
            drone_idx: ドローンのインデックス（0ベース）
            fault_type: 障害の種類
            magnitude: 障害の大きさ
        """
        if 0 <= drone_idx < self.num_drones:
            self.drones[drone_idx].set_fault(fault_type, magnitude)
    
    def clear_drone_fault(self, drone_idx: int):
        """指定したドローンの障害をクリア"""
        if 0 <= drone_idx < self.num_drones:
            self.drones[drone_idx].clear_fault()
    
    def reset(self):
        """シミュレーションをリセット"""
        initial_positions = [
            np.array([0.0, 0.0, 1.0]),  # ドローン1
            np.array([0.5, 0.0, 1.0])   # ドローン2
        ]
        
        for i, drone in enumerate(self.drones):
            if i < len(initial_positions):
                drone.reset(initial_positions[i])
            else:
                drone.reset()
        
        self.time = 0.0
