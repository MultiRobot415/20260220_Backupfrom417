#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
virtual_leader.py - 仮想リーダー管理モジュール

PID階層型SLAFにおける仮想リーダー（ソフトウェアエージェント）の
目標軌道生成と状態管理を行います。

2機の仮想リーダーの位置・速度・加速度を管理し、
キーボード入力に応じて目標軌道を更新します。
"""

import numpy as np
import time
import logging

# ログ設定
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class VirtualLeader:
    """仮想リーダークラス（単一リーダー）"""
    
    def __init__(self, leader_id, initial_position, dt=0.1):
        """
        Args:
            leader_id: リーダーID（0 or 1）
            initial_position: 初期位置 [x, y, z]
            dt: 制御周期（秒）
        """
        self.leader_id = leader_id
        self.dt = dt
        
        # 状態変数（3D位置）
        self.position = np.array(initial_position, dtype=float)  # [x, y, z]
        self.velocity = np.zeros(3)  # [vx, vy, vz]
        self.acceleration = np.zeros(3)  # [ax, ay, az]
        
        # 目標位置（キーボード入力で更新）
        self.target_position = np.array(initial_position, dtype=float)
        
        # 速度・加速度推定用の履歴
        self.position_history = [self.position.copy()]
        self.velocity_history = [self.velocity.copy()]
        
        # 最大履歴長
        self.max_history = 5
        
        logger.info(f"仮想リーダー{leader_id}初期化: 位置={initial_position}")
    
    def set_target_position(self, x=None, y=None, z=None):
        """
        目標位置を設定
        
        Args:
            x, y, z: 目標位置座標（Noneの場合は変更しない）
        """
        if x is not None:
            self.target_position[0] = x
        if y is not None:
            self.target_position[1] = y
        if z is not None:
            self.target_position[2] = z
        
        logger.debug(f"リーダー{self.leader_id}目標位置更新: {self.target_position}")
    
    def update_target_position(self, dx=0.0, dy=0.0, dz=0.0):
        """
        目標位置を相対的に更新
        
        Args:
            dx, dy, dz: 相対移動量
        """
        self.target_position += np.array([dx, dy, dz])
        logger.debug(f"リーダー{self.leader_id}目標位置更新: {self.target_position}")
    
    def update(self):
        """
        仮想リーダーの状態を更新
        
        理想的には目標位置に即座に追従すると仮定するが、
        速度・加速度は数値微分で推定する
        """
        # 前回の状態を保存
        prev_position = self.position.copy()
        prev_velocity = self.velocity.copy()
        
        # 位置を目標位置に設定（理想的な追従）
        self.position = self.target_position.copy()
        
        # 速度を数値微分で推定
        self.velocity = (self.position - prev_position) / self.dt
        
        # 加速度を数値微分で推定
        self.acceleration = (self.velocity - prev_velocity) / self.dt
        
        # 履歴を更新
        self.position_history.append(self.position.copy())
        self.velocity_history.append(self.velocity.copy())
        
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
            self.velocity_history.pop(0)
    
    def get_state(self):
        """
        現在の状態を取得
        
        Returns:
            dict: {'position': [x,y,z], 'velocity': [vx,vy,vz], 'acceleration': [ax,ay,az]}
        """
        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'acceleration': self.acceleration.copy(),
            'target_position': self.target_position.copy()
        }
    
    def get_planar_state(self):
        """
        水平2次元（x-z平面）の状態を取得
        
        Returns:
            dict: {'position': [x,z], 'velocity': [vx,vz], 'acceleration': [ax,az]}
        """
        return {
            'position': np.array([self.position[0], self.position[2]]),
            'velocity': np.array([self.velocity[0], self.velocity[2]]),
            'acceleration': np.array([self.acceleration[0], self.acceleration[2]]),
            'target_position': np.array([self.target_position[0], self.target_position[2]])
        }


class VirtualLeaderManager:
    """仮想リーダー管理クラス（複数リーダー）"""
    
    def __init__(self, num_leaders=2, initial_positions=None, formation_offset=None, dt=0.1):
        """
        Args:
            num_leaders: リーダー数
            initial_positions: 初期位置のリスト [[x1,y1,z1], [x2,y2,z2], ...]
            formation_offset: リーダー間のフォーメーションオフセット [[dx1,dy1,dz1], ...]
            dt: 制御周期
        """
        self.num_leaders = num_leaders
        self.dt = dt
        
        # デフォルト初期位置
        if initial_positions is None:
            initial_positions = [
                [0.0, 1.0, -0.5],  # リーダー1: 左側
                [0.0, 1.0, 0.5]    # リーダー2: 右側
            ]
        
        # デフォルトフォーメーションオフセット（リーダー1基準）
        if formation_offset is None:
            formation_offset = [
                [0.0, 0.0, 0.0],   # リーダー1: 基準
                [0.0, 0.0, 1.0]    # リーダー2: Z軸+1m
            ]
        
        self.formation_offset = formation_offset
        
        # 各リーダーを初期化
        self.leaders = []
        for i in range(num_leaders):
            leader = VirtualLeader(
                leader_id=i,
                initial_position=initial_positions[i],
                dt=dt
            )
            self.leaders.append(leader)
        
        # 基準リーダー（通常はリーダー0）
        self.reference_leader_id = 0
        
        logger.info(f"仮想リーダー管理初期化: {num_leaders}機")
    
    def set_reference_leader_target(self, x=None, y=None, z=None):
        """
        基準リーダーの目標位置を設定し、他のリーダーはフォーメーションを維持
        
        Args:
            x, y, z: 基準リーダーの目標位置
        """
        ref_leader = self.leaders[self.reference_leader_id]
        
        # 基準リーダーの目標位置を設定
        if x is not None:
            ref_leader.target_position[0] = x
        if y is not None:
            ref_leader.target_position[1] = y
        if z is not None:
            ref_leader.target_position[2] = z
        
        # 他のリーダーはフォーメーションオフセットを維持
        for i, leader in enumerate(self.leaders):
            if i != self.reference_leader_id:
                leader.target_position = (
                    ref_leader.target_position + np.array(self.formation_offset[i])
                )
    
    def update_reference_leader_target(self, dx=0.0, dy=0.0, dz=0.0):
        """
        基準リーダーの目標位置を相対的に更新
        
        Args:
            dx, dy, dz: 相対移動量
        """
        ref_leader = self.leaders[self.reference_leader_id]
        ref_leader.target_position += np.array([dx, dy, dz])
        
        # 他のリーダーも同じ量だけ移動
        for i, leader in enumerate(self.leaders):
            if i != self.reference_leader_id:
                leader.target_position += np.array([dx, dy, dz])
    
    def update_all(self):
        """全リーダーの状態を更新"""
        for leader in self.leaders:
            leader.update()
    
    def get_all_states(self):
        """
        全リーダーの状態を取得
        
        Returns:
            list: [leader0_state, leader1_state, ...]
        """
        return [leader.get_state() for leader in self.leaders]
    
    def get_all_planar_states(self):
        """
        全リーダーの水平2次元状態を取得
        
        Returns:
            list: [leader0_planar_state, leader1_planar_state, ...]
        """
        return [leader.get_planar_state() for leader in self.leaders]
    
    def get_leader_state(self, leader_id):
        """
        特定のリーダーの状態を取得
        
        Args:
            leader_id: リーダーID
        
        Returns:
            dict: leader_state
        """
        if 0 <= leader_id < self.num_leaders:
            return self.leaders[leader_id].get_state()
        else:
            logger.error(f"無効なリーダーID: {leader_id}")
            return None


if __name__ == "__main__":
    # テストコード
    print("=== 仮想リーダー管理テスト ===")
    
    manager = VirtualLeaderManager(num_leaders=2, dt=0.1)
    
    # 初期状態
    states = manager.get_all_states()
    print("\n初期状態:")
    for i, state in enumerate(states):
        print(f"  リーダー{i}: 位置={state['position']}, 速度={state['velocity']}")
    
    # 目標位置を更新
    print("\n目標位置を[1, 1, 0]に設定")
    manager.set_reference_leader_target(x=1.0, y=1.0, z=0.0)
    
    # 数ステップ更新
    for step in range(5):
        manager.update_all()
        states = manager.get_all_states()
        print(f"\nステップ{step+1}:")
        for i, state in enumerate(states):
            print(f"  リーダー{i}: 位置={state['position']}, 速度={state['velocity']}")
    
    # 相対移動
    print("\n前進(dx=0.5)")
    manager.update_reference_leader_target(dx=0.5, dy=0.0, dz=0.0)
    manager.update_all()
    states = manager.get_all_states()
    for i, state in enumerate(states):
        print(f"  リーダー{i}: 位置={state['position']}")
