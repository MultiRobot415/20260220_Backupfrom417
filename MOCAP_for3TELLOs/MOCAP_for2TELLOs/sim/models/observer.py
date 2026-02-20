#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
オブザーバーモデル実装
ドローンの状態推定と故障検知のためのオブザーバークラスを提供します。
"""

import numpy as np
from typing import Tuple, Dict, List, Optional


class StateObserver:
    """
    ドローン状態推定のためのオブザーバークラス
    線形状態オブザーバーを実装し、位置・速度を推定します
    """
    
    def __init__(self, dt: float = 0.1, noise_level: float = 0.01):
        """
        オブザーバーの初期化
        
        Args:
            dt: サンプリング時間（秒）
            noise_level: モデル化に用いるノイズレベル
        """
        # システム行列の初期化（単純な積分モデル）
        # 状態ベクトル: [x, y, z, vx, vy, vz]
        self.A = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # 観測行列 (位置のみ観測)
        self.C = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # オブザーバーゲイン（チューニングが必要）
        self.L = np.array([
            [0.5, 0, 0],
            [0, 0.5, 0],
            [0, 0, 0.5],
            [0.7, 0, 0],
            [0, 0.7, 0],
            [0, 0, 0.7]
        ])
        
        # 状態変数の初期化
        self.x_hat = np.zeros(6)  # 推定状態
        self.dt = dt
        self.noise_level = noise_level
        
        # 残差（観測値と推定値の差）
        self.residual = np.zeros(3)
        
        # 残差の履歴（異常検出に使用）
        self.residual_history = []
        self.max_history_length = 50  # 履歴の最大長
        
        # 故障検出用パラメータ
        self.fault_threshold = 0.08  # 残差閾値（小さくして故障に敵感に）
        self.fault_counter = 0  # 連続して閾値を超えたカウント
        self.fault_counter_threshold = 3  # 故障と判定するカウント閾値（0.3秒で検出）
        
    def reset(self, initial_state: Optional[np.ndarray] = None):
        """状態をリセットする"""
        if initial_state is not None:
            self.x_hat = initial_state
        else:
            self.x_hat = np.zeros(6)
        self.residual = np.zeros(3)
        self.residual_history = []
        self.fault_counter = 0
    
    def _rc_to_acceleration(self, rc_command: np.ndarray) -> np.ndarray:
        """
        RCコマンドから予想される加速度を計算
        
        Args:
            rc_command: [roll, pitch, throttle, yaw]のRCコマンド
            
        Returns:
            予測される加速度 [ax, ay, az]
        """
        if rc_command is None:
            return np.zeros(3)
            
        # RCコマンドのスケーリング（10分の1を加速度と仮定）
        # RC値の範囲は通常 -100 から 100
        roll_cmd, pitch_cmd, throttle_cmd, yaw_cmd = rc_command
        
        # 単純なモデル：roll → y方向加速度、pitch → x方向加速度、throttle → z方向加速度
        # 符号は機体の座標系と一致するように調整
        ax = -pitch_cmd / 10.0  # 前進（+pitch）で+x方向の加速度
        ay = roll_cmd / 10.0    # 右傾斜（+roll）で+y方向の加速度
        az = throttle_cmd / 10.0 # +throttleで+z方向（上向き）の加速度
        
        return np.array([ax, ay, az])
    
    def predict(self, control_input: np.ndarray = None) -> np.ndarray:
        """
        状態を予測ステップ
        
        Args:
            control_input: 制御入力 [roll, pitch, throttle, yaw]
            
        Returns:
            予測された状態
        """
        # 基本的な状態遷移
        self.x_hat = self.A @ self.x_hat
        
        # 制御入力がある場合、その効果を加味
        if control_input is not None:
            # RCコマンドから予測される加速度
            acceleration = self._rc_to_acceleration(control_input)
            
            # 速度成分に加速度を適用
            self.x_hat[3:6] += acceleration * self.dt
        
        return self.x_hat[:3]  # 位置のみ返す
    
    def update(self, measurement: np.ndarray) -> np.ndarray:
        """
        測定値による状態の更新
        
        Args:
            measurement: 測定位置 [x, y, z]
            
        Returns:
            更新された状態推定値
        """
        # 予測値と測定値の差（残差）を計算
        z = measurement.reshape(3, 1)
        y_hat = self.C @ self.x_hat.reshape(6, 1)
        self.residual = (z - y_hat).flatten()
        
        # 残差履歴の更新
        self.residual_history.append(np.linalg.norm(self.residual))
        if len(self.residual_history) > self.max_history_length:
            self.residual_history.pop(0)
        
        # 状態の更新（故障ありなしに関わらず同じフィードバック）
        self.x_hat = self.x_hat + (self.L @ self.residual).flatten()
        
        return self.x_hat[:3]  # 位置のみ返す
    
    def get_velocity(self) -> np.ndarray:
        """推定速度を取得"""
        return self.x_hat[3:6]
    
    def get_state(self) -> np.ndarray:
        """推定状態全体を取得"""
        return self.x_hat
    
    def get_residual(self) -> np.ndarray:
        """現在の残差を取得"""
        return self.residual
    
    def get_trust_metric(self) -> float:
        """
        信頼度指標を計算
        元論文方式：閾値を超えた時間（fault_counter）に基づいて信頼度を計算
        fault_counterがfault_counter_thresholdに近づくほど信頼度は0に近づく
        fault_counterが0に近いほど信頼度は1に近づく
        """
        if not self.residual_history:
            return 1.0
        
        # 故障カウンター値と閾値の比率から信頼度を計算
        # 閾値を超えた時間が長いほど信頼度は低下する
        normalized_counter = min(1.0, self.fault_counter / self.fault_counter_threshold)
        
        # 逆の比率で信頼度を計算（0に近いほど信頼度高、1に近いほど信頼度低）
        trust = 1.0 - normalized_counter
        
        # シグモイドでより滑らかに変化させる（オプション）
        # 信頼度が0と1の間で急激に変化するのを防ぐ
        k = 10.0  # 傾斜係数
        x0 = 0.5   # 中心点
        trust = 1.0 / (1.0 + np.exp(-k * (trust - x0)))
        
        return float(trust)
    
    def detect_fault(self) -> Tuple[bool, float]:
        """
        故障検出処理
        
        Returns:
            (故障検出フラグ, 信頼度)
        """
        if not self.residual_history:
            return False, 1.0
        
        # 残差の大きさをチェック
        current_residual_norm = np.linalg.norm(self.residual)
        
        # 閾値を超えているかチェック
        if current_residual_norm > self.fault_threshold:
            # 閾値を超えたらカウンターを増加
            self.fault_counter += 1
        else:
            # 閾値以下なら徐々にカウンタをリセット
            # より早く回復するため減衰率を調整（0.5→1.0）
            self.fault_counter = max(0, self.fault_counter - 1.0)
        
        # 連続カウントが閾値を超えたら故障と判定
        fault_detected = self.fault_counter >= self.fault_counter_threshold
        
        # 閾値超過時間に基づいた信頼度計算
        trust = self.get_trust_metric()
        
        return fault_detected, trust


class DroneObserver:
    """
    ドローン用オブザーバーラッパークラス
    複数ドローンの状態推定と故障検出を管理します
    """
    
    def __init__(self, num_drones: int = 2, dt: float = 0.1):
        """
        複数ドローン用オブザーバーの初期化
        
        Args:
            num_drones: ドローンの数
            dt: サンプリング時間（秒）
        """
        self.observers = [StateObserver(dt=dt) for _ in range(num_drones)]
        self.num_drones = num_drones
        self.trust_metrics = [1.0] * num_drones  # 初期信頼度は1.0
        self.leader_idx = 0  # リーダーの初期インデックス（0=1号機）
        self.time_counter = 0  # 時間カウンター
        self.dt = dt
        self.trust_calculation_start_time = 5.0  # 信頼度計算開始時間（5秒後）
    
    def update(self, positions: List[np.ndarray], rc_commands: List[np.ndarray] = None) -> Dict:
        """
        全ドローンの状態を更新
        
        Args:
            positions: 各ドローンの測定位置のリスト
            rc_commands: 各ドローンのRCコマンドのリスト（オプション）
            
        Returns:
            更新結果の辞書
        """
        results = {}
        
        # デフォルトのRCコマンド（Noneの場合）
        if rc_commands is None:
            rc_commands = [None] * len(positions)
        
        for i, (observer, position, rc_command) in enumerate(zip(self.observers, positions, rc_commands)):
            # 予測ステップ（RCコマンド考慮）
            observer.predict(rc_command)
            
            # 更新ステップ（故障検出状態に関わらず同じ処理）
            estimated_pos = observer.update(position)
            
            # 故障検出（更新後）
            fault_detected, trust = observer.detect_fault()
            self.trust_metrics[i] = trust
            
            # 結果格納
            drone_id = i + 1  # ドローンIDは1から始まる
            results[drone_id] = {
                'position': estimated_pos,
                'velocity': observer.get_velocity(),
                'residual': observer.get_residual(),
                'trust': trust,
                'fault_detected': fault_detected,
                'is_leader': (i == self.leader_idx)
            }
        
        # リーダー選定のロジックを適用
        self._update_leader_selection()
        
        return results
    
    def _update_leader_selection(self):
        """
        信頼度に基づいてリーダーを選定する
        現在のリーダーの信頼度が閾値を下回り、他のドローンの信頼度が高い場合に切り替え
        """
        # リーダー切り替え閾値
        LEADER_SWITCH_THRESHOLD = 0.6
        MIN_TRUST_DIFFERENCE = 0.2
        
        current_leader_trust = self.trust_metrics[self.leader_idx]
        
        # リーダーの信頼度が低い場合
        if current_leader_trust < LEADER_SWITCH_THRESHOLD:
            # より高い信頼度を持つドローンを探す
            for i, trust in enumerate(self.trust_metrics):
                if i != self.leader_idx and trust > current_leader_trust + MIN_TRUST_DIFFERENCE:
                    # リーダーを切り替え
                    self.leader_idx = i
                    break
    
    def get_leader_index(self) -> int:
        """現在のリーダーインデックスを取得"""
        return self.leader_idx
    
    def get_trust_metrics(self) -> List[float]:
        """全ドローンの信頼度指標を取得"""
        return self.trust_metrics
    
    def reset(self):
        """全オブザーバーのリセット"""
        for observer in self.observers:
            observer.reset()
        self.trust_metrics = [1.0] * self.num_drones
