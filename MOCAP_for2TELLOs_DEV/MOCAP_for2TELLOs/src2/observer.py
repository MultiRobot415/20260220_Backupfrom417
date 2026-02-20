#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
オブザーバーモデル実装（実機用）
ドローンの状態推定と故障検知のためのオブザーバークラスを提供します。
"""

import numpy as np
import time
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
        
        # オブザーバーゲイン（誤検知防止とFloat故障検知のバランス）
        # 誤検知防止のため追従性を上げつつ、Float故障は検知可能に調整（2025-10-01修正）
        # 修正履歴: 位置1.5→0.8→0.4→0.3→0.5、速度0.6→0.4→0.2→0.15→0.25
        # Float故障（全方向0入力）は非常に大きな残差を生むため、通常移動と区別可能
        self.L = np.array([
            [0.5, 0, 0],  # 位置フィードバックゲイン: 0.3→0.5（誤検知防止とのバランス）
            [0, 0.5, 0],
            [0, 0, 0.5],
            [0.25, 0, 0],  # 速度フィードバックゲイン: 0.15→0.25（誤検知防止とのバランス）
            [0, 0.25, 0],
            [0, 0, 0.25]
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
        
        # 故障検出用パラメータ（制御入力モデル修正に合わせて閾値調整）
        self.fault_threshold = 0.2  # 残差閾値: 0.3 → 0.2（より敏感に、2025-09-30修正）
        self.fault_counter = 0  # 連続して閾値を超えたカウント
        self.fault_counter_threshold = 1  # 故障と判定するカウント閾値（即座に検出）
        
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
            
        修正内容（2025-09-30）:
            - スケーリング係数を10.0から40.0に変更（4倍）
            - 座標系を修正（Y軸とZ軸の対応を修正）
            - RC値20で約0.5m/s²の加速度（実機特性に基づく）
            - これにより故障時の残差が適切に増大する
        """
        if rc_command is None:
            return np.zeros(3)
            
        roll_cmd, pitch_cmd, throttle_cmd, yaw_cmd = rc_command
        
        # 適切なスケーリング係数（RC値20で約0.5m/s²の加速度）
        # 係数を10.0から40.0に変更（4倍）→制御入力の影響を適切なスケールに調整
        # 座標系を修正: throttle→Y軸、roll→Z軸（システム座標系に準拠）
        ax = -pitch_cmd / 40.0    # pitch → X軸（前後方向）
        ay = throttle_cmd / 40.0  # throttle → Y軸（上下方向）← 修正
        az = roll_cmd / 40.0      # roll → Z軸（左右方向）← 修正
        
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
    
    def update(self, measurement) -> np.ndarray:
        """
        測定値による状態の更新
        
        Args:
            measurement: 測定位置 [x, y, z]（リストまたはnumpy配列）
            
        Returns:
            更新された状態推定値
        """
        # 入力がNoneまたは不正な形式の場合は前回の状態を返す
        if measurement is None:
            return self.x_hat[:3]  # 位置のみ返す
        
        # 入力がリスト型またはタプル型の場合はNumPy配列に変換
        if isinstance(measurement, (list, tuple)):
            try:
                measurement = np.array(measurement, dtype=np.float64)
            except Exception as e:
                print(f"測定値のNumPy配列への変換に失敗: {e}")
                return self.x_hat[:3]  # 位置のみ返す
        
        # NumPy配列でない場合（np.ndarray型チェック）
        if not isinstance(measurement, np.ndarray):
            print(f"測定値が正しい形式ではありません: {type(measurement)}")
            return self.x_hat[:3]  # 位置のみ返す
            
        # 予測値と測定値の差（残差）を計算
        try:
            z = measurement.reshape(3, 1)
            y_hat = self.C @ self.x_hat.reshape(6, 1)
            self.residual = (z - y_hat).flatten()
        except Exception as e:
            print(f"オブザーバーの残差計算エラー: {e}")
            print(f"measurement: {measurement}, type: {type(measurement)}")
            return self.x_hat[:3]  # 位置のみ返す
        
        # 残差履歴の更新（X-Z平面のみで故障検知）
        # Y軸（高度）を除外し、水平面（X-Z）での故障検知に特化
        residual_xz = np.array([self.residual[0], self.residual[2]])  # [x, z]のみ
        self.residual_history.append(np.linalg.norm(residual_xz))
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
        論文通りの信頼度指標を計算
        - 残差の継続的な閾値超過に基づく故障検知
        - 閾値超過時間に応じた信頼度の時間的減少
        - 故障注入は制御値の異常を通じて間接的に残差を増加させる
        """
        if not self.residual_history:
            return 1.0
        
        # 現在の残差ノルムを計算（X-Z平面のみ）
        # Y軸（高度）を除外し、水平面での故障検知に特化
        residual_xz = np.array([self.residual[0], self.residual[2]])  # [x, z]のみ
        current_residual_norm = np.linalg.norm(residual_xz)
        
        # 閾値超過の継続時間に基づく信頼度計算
        if current_residual_norm > self.fault_threshold:
            # 閾値を超えている場合、カウンターを増加
            self.fault_counter += 1
        else:
            # 閾値以下の場合、カウンターを徐々にリセット（回復）
            self.fault_counter = max(0, self.fault_counter - 0.5)
        
        # 閾値超過継続時間に基づく信頼度計算
        # fault_counter_threshold回連続で閾値を超えると信頼度が大幅に低下
        if self.fault_counter >= self.fault_counter_threshold:
            # 継続的な閾値超過による信頼度減少（迅速低下版）
            excess_time = self.fault_counter - self.fault_counter_threshold
            # 時間的減少強化：3回（0.3秒）の超過で信頼度が0.01まで急速低下
            decay_factor = np.exp(-excess_time / 3.0)  # 10.0 → 3.0 (約3.3倍高速化)
            trust = max(0.01, decay_factor)  # 0.05 → 0.01 (ユーザー指定：最低信頼度1%)
            
            # デバッグ出力（しきい値判定はLeaderSwitchingで実施）
            print(f"⚠️  論文アルゴリズム: 残差継続超過による信頼度低下 -> {trust:.3f} (残差: {current_residual_norm:.3f}, 継続: {self.fault_counter}回)")
        else:
            # 正常時または軽微な異常時の信頼度
            normalized_counter = self.fault_counter / self.fault_counter_threshold
            trust = 1.0 - 0.3 * normalized_counter  # 最大30%の信頼度低下
        
        # 時間経過による信頼度回復（残差が正常範囲内の場合）
        if len(self.residual_history) > 5:
            recent_residuals = self.residual_history[-5:]
            avg_recent_residual = np.mean(recent_residuals)
            
            if avg_recent_residual < self.fault_threshold * 0.5:
                # 残差が小さい場合は信頼度を徐々に回復
                recovery_factor = 1.02
                trust = min(1.0, trust * recovery_factor)
        
        return float(trust)
    
    def _check_fault_injection_timeout(self):
        """
        Phase 2: 故障注入のタイムアウトをチェック（StateObserver用）
        DroneObserverクラスに委譲
        """
        if hasattr(self, '_parent_observer') and self._parent_observer:
            self._parent_observer._check_fault_injection_timeout()
    
    def detect_fault(self) -> Tuple[bool, float]:
        """
        故障検出処理（get_trust_metric()に処理を委譲）
        
        Returns:
            (故障検出フラグ, 信頼度)
        """
        if not self.residual_history:
            return False, 1.0
        
        # 信頼度計算（内部でfault_counterを更新）
        trust = self.get_trust_metric()
        
        # 連続カウントが閾値を超えたら故障と判定
        fault_detected = self.fault_counter >= self.fault_counter_threshold
        
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
        
        # 実機用に追加: 初期化時刻
        self.start_time = time.time()
        
        # Phase 2: 故障注入フラグと拡張機能
        self.fault_injected = False
        self.fault_injection_start_time = None
        self.fault_injection_duration = 5.0  # 故障注入の持続時間（秒）
        
        # Phase 2: StateObserverインスタンスに親オブザーバーへの参照を設定
        for observer in self.observers:
            observer._parent_observer = self
    
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
        
        # 時間カウンターの更新
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        self.time_counter = elapsed_time
        
        # 実行から5秒間は常に信頼度1.0とする（初期状態の安定化のため）
        trust_calculation_enabled = elapsed_time > self.trust_calculation_start_time
        
        # デフォルトのRCコマンド（Noneの場合）
        if rc_commands is None:
            rc_commands = [None] * len(positions)
        
        for i, (observer, position, rc_command) in enumerate(zip(self.observers, positions, rc_commands)):
            # 予測ステップ（RCコマンド考慮）
            observer.predict(rc_command)
            
            # 更新ステップ（故障検出状態に関わらず同じ処理）
            estimated_pos = observer.update(position)
            
            # Phase 2: 故障注入タイムアウトチェック
            observer._check_fault_injection_timeout()
            
            # 故障検出（更新後）
            fault_detected = False
            trust = 1.0
            
            if trust_calculation_enabled:
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
        
        # リーダー選定のロジックを適用（信頼度計算が有効な場合のみ）
        if trust_calculation_enabled:
            self._update_leader_selection()
        
        return results
    
    def _update_leader_selection(self):
        """
        信頼度に基づいてリーダーを選定する
        現在のリーダーの信頼度が閾値を下回り、他のドローンの信頼度が高い場合に切り替え
        """
        # リーダー切り替え閾値
        LEADER_SWITCH_THRESHOLD = 0.1  # ユーザー指定：リーダー交代しきい値を10%に設定
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
    
    def set_leader_index(self, leader_index: int):
        """
        Phase 2: リーダーインデックスを設定
        
        Args:
            leader_index: 新しいリーダーのインデックス (0ベース)
        """
        if 0 <= leader_index < len(self.trust_metrics):
            self.leader_idx = leader_index
            print(f"📊 Phase 2: DroneObserverのリーダーインデックスを{leader_index}に更新")
        else:
            print(f"⚠️  Phase 2: 無効なリーダーインデックス: {leader_index} (有効範囲: 0-{len(self.trust_metrics)-1})")
    
    def get_trust_metrics(self) -> List[float]:
        """全ドローンの信頼度指標を取得"""
        return self.trust_metrics
    
    def inject_fault(self, enable: bool = True):
        """
        Phase 2: 強化された故障注入機能
        
        Args:
            enable: True=故障を注入, False=故障を解除
        """
        if enable and not hasattr(self, 'fault_injected'):
            # 初回故障注入時にフラグを初期化
            self.fault_injected = False
            self.fault_injection_start_time = None
            
        if enable and not self.fault_injected:
            # 故障注入開始
            self.fault_injected = True
            self.fault_injection_start_time = time.time()
            print(f"🚨 Phase 2: 故障注入開始（{self.fault_injection_duration}秒間） - 信頼度が低下します")
        elif not enable:
            # 故障注入解除
            if hasattr(self, 'fault_injected') and self.fault_injected:
                print(f"✅ Phase 2: 故障注入解除 - 信頼度が回復します")
            self.fault_injected = False
            self.fault_injection_start_time = None
    
    def _check_fault_injection_timeout(self):
        """
        Phase 2: 故障注入のタイムアウトをチェック
        """
        if (self.fault_injected and 
            self.fault_injection_start_time is not None and 
            time.time() - self.fault_injection_start_time > self.fault_injection_duration):
            
            print(f"⏰ Phase 2: 故障注入タイムアウト（{self.fault_injection_duration}秒経過）")
            self.inject_fault(False)
    
    def is_fault_injected(self) -> bool:
        """故障注入フラグの状態を取得"""
        return self.fault_injected
    
    def get_state_estimate(self, drone_index: int) -> Optional[np.ndarray]:
        """
        指定されたドローンの状態推定値を取得
        
        Args:
            drone_index: ドローンのインデックス (0ベース)
            
        Returns:
            状態推定値 [x, y, z, vx, vy, vz] または None
        """
        if 0 <= drone_index < len(self.observers):
            return self.observers[drone_index].get_state()
        return None
    
    def get_residual(self, drone_index: int) -> Optional[np.ndarray]:
        """
        指定されたドローンの残差を取得
        
        Args:
            drone_index: ドローンのインデックス (0ベース)
            
        Returns:
            残差 [x, y, z] または None
        """
        if 0 <= drone_index < len(self.observers):
            return self.observers[drone_index].get_residual()
        return None
    
    def reset(self):
        """全オブザーバーのリセット"""
        for observer in self.observers:
            observer.reset()
        self.trust_metrics = [1.0] * self.num_drones
        self.fault_injected = False
        self.start_time = time.time()  # 開始時刻もリセット
