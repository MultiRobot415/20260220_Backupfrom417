#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slaf_pid_controller.py - PID階層型SLAF制御システム

ref/sim_PID_v1/system_dynamics.m の忠実なPython実装
PID階層型SLAF（速度フィードバック付き同時分散位置推定・隊形追従制御）を実現

構成：
- 上位層：速度フィードバック型PID推定器
- 下位層：位置制御器
- グラフ構造：仮想リーダー2機 + 実機フォロワー2機（4エージェント）
"""

import numpy as np
import logging
from weight_matrices import (
    calculate_xi_correction,
    calculate_psi_collinearity_avoidance,
    calculate_weight_matrices_for_agent
)

# ログ設定
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SLAFPIDController:
    """PID階層型SLAF制御器クラス"""
    
    def __init__(self, follower_id, neighbors, dt=0.1):
        """
        Args:
            follower_id: フォロワーID（3 or 4）
            neighbors: 隣接エージェントIDのリスト [j_idx, k_idx]
            dt: 制御周期（秒）
        """
        self.follower_id = follower_id
        self.neighbors = neighbors  # [j_idx, k_idx]
        self.dt = dt
        
        # === PID階層型パラメータ（ref/sim_PID_v1/define_trajectory_simple.m準拠） ===
        # 推定器パラメータ（上位層）
        self.k_p = 5.0    # 位置ゲイン（P制御）
        self.k_v = 1.0    # 速度ゲイン（D制御、速度フィードバック）
        self.k_i = 0.1
        # 制御器パラメータ（下位層）   
        self.k_cp = 5.0  # 制御器位置ゲイン
        self.k_cv = 2.0
        self.k_cv_nominal = 2.0  # Hモード時の速度ゲイン（元の値）
        # 補正項のゲイン
        self.xi_gain = 1.0
        # 共線回避パラメータ
        self.lambda_vec = np.array([0.1, 0.1])  # 水平2次元の調整ベクトル
        self.tau_threshold = 0.1  # 共線判定閾値
        self.tau_gain = 1.0  # τのゲイン（共線回避の強さを調整）
        
        # 不感帯パラメータ（src/MOCAP_for2TELLOsのposition_control.pyに合わせる）
        self.deadband_x = 0.0  # x方向（前後）の不感帯 (m)
        self.deadband_z = 0.0 # z方向（左右）の不感帯 (m)
        
        # オクルージョンフラグ
        self.is_occluded = False  # センサオクルージョン状態
        
        # 初期化フラグ（初期化直後の最初の更新で推定誤差をゼロにリセット）
        self.just_initialized = False
        
        # === 状態変数（2次元: x-z平面） ===
        # 実位置・実速度（MOCAP測定値から設定）
        self.p_actual = np.zeros(2)  # [x, z]
        self.v_actual = np.zeros(2)  # [vx, vz]
        
        # 推定位置・推定速度
        self.p_hat = np.zeros(2)  # [x_hat, z_hat]
        self.v_hat = np.zeros(2)  # [vx_hat, vz_hat]
        
        # 積分状態
        self.z_integral = np.zeros(2)  # [z_x, z_z]
        
        # 目標軌道（フォロワーの目標）
        self.p_star = np.zeros(2)
        self.v_star = np.zeros(2)
        self.a_star = np.zeros(2)
        
        # 速度推定用の履歴
        self.position_history = []
        self.max_history = 5
        
        # デバッグ情報
        self.debug_info = {
            'xi': np.zeros(2),
            'psi': np.zeros(2),
            'tau': 0.0,  # bearing誤差
            'is_collinear': False,
            'control_input': np.zeros(2)
        }
        
        logger.info(f"SLAFPIDコントローラ初期化: フォロワー{follower_id}, 隣接{neighbors}")
    
    def set_gains(self, k_p=None, k_v=None, k_i=None, k_cp=None, k_cv=None, xi_gain=None):
        """
        ゲインを動的に設定（オプション）
        
        Args:
            k_p, k_v, k_i: 推定器ゲイン
            k_cp, k_cv: 制御器ゲイン
            xi_gain: ξ項のゲイン
        """
        if k_p is not None:
            self.k_p = k_p
        if k_v is not None:
            self.k_v = k_v
        if k_i is not None:
            self.k_i = k_i
        if k_cp is not None:
            self.k_cp = k_cp
        if k_cv is not None:
            self.k_cv = k_cv
        if xi_gain is not None:
            self.xi_gain = xi_gain
    
    def set_deadbands(self, deadband_x=None, deadband_z=None):
        """
        不感帯を設定（src/MOCAP_for2TELLOsのposition_control.pyに合わせる）
        
        Args:
            deadband_x: x方向（前後）の不感帯 (m)
            deadband_z: z方向（左右）の不感帯 (m)
        """
        if deadband_x is not None:
            self.deadband_x = deadband_x
        if deadband_z is not None:
            self.deadband_z = deadband_z
        logger.info(f"フォロワー{self.follower_id}不感帯設定: x={self.deadband_x}, z={self.deadband_z}")
    
    def initialize_state(self, p_initial, v_initial=None):
        """
        初期状態を設定（Assumption: 初期推定誤差零）
        
        Args:
            p_initial: 初期位置 [x, z]（MOCAPから取得）
            v_initial: 初期速度 [vx, vz]（Noneの場合はゼロ）
        """
        self.p_actual = np.array(p_initial, dtype=float)
        self.v_actual = np.array(v_initial, dtype=float) if v_initial is not None else np.zeros(2)
        
        # Assumption: p̂_i(0) = p_i(0), v̂_i(0) = v_i(0)
        self.p_hat = self.p_actual.copy()
        self.v_hat = self.v_actual.copy()
        
        # z_i(0) = 0
        self.z_integral = np.zeros(2)
        
        # 履歴初期化
        self.position_history = [self.p_actual.copy()]
        
        # 初期化フラグを設定（次の更新で推定誤差を強制的にゼロにする）
        self.just_initialized = True
        
        logger.info(f"フォロワー{self.follower_id}初期化: p={self.p_actual}, v={self.v_actual}")
    
    def update_actual_state(self, p_mocap):
        """
        MOCAP測定値から実位置・実速度を更新
        
        Args:
            p_mocap: MOCAP測定位置 [x, z]
        """
        # 前回の位置を保存
        prev_position = self.p_actual.copy()
        
        # 実位置を更新
        self.p_actual = np.array(p_mocap, dtype=float)
        
        # 実速度を数値微分で推定
        self.v_actual = (self.p_actual - prev_position) / self.dt
        
        # 履歴を更新
        self.position_history.append(self.p_actual.copy())
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
    
    def set_target_trajectory(self, p_star, v_star, a_star):
        """
        目標軌道を設定
        
        Args:
            p_star: 目標位置 [x, z]
            v_star: 目標速度 [vx, vz]
            a_star: 目標加速度 [ax, az]
        """
        self.p_star = np.array(p_star, dtype=float)
        self.v_star = np.array(v_star, dtype=float)
        self.a_star = np.array(a_star, dtype=float)
    
    def calculate_xi(self, neighbor_positions_hat, neighbor_positions_star, neighbor_positions_actual):
        """
        幾何学的補正項ξを計算（論文Eq. (20)に準拠）
        
        重要：
        - 重み行列H：観測値（実位置、MOCAP測定）から計算
        - ξの計算：推定位置を使用
        
        Args:
            neighbor_positions_hat: 隣接エージェントの推定位置 {agent_id: [x, z], ...}
            neighbor_positions_star: 隣接エージェントの目標位置 {agent_id: [x, z], ...}
            neighbor_positions_actual: 隣接エージェントの実位置 {agent_id: [x, z], ...}
        
        Returns:
            xi: 補正項 (2x1)
        """
        j_idx, k_idx = self.neighbors
        
        # 隣接エージェントの位置を取得
        # 推定位置（ξの計算に使用）
        p_j_hat = neighbor_positions_hat.get(j_idx, self.p_hat)
        p_k_hat = neighbor_positions_hat.get(k_idx, self.p_hat)
        p_j_star = neighbor_positions_star.get(j_idx, self.p_star)
        p_k_star = neighbor_positions_star.get(k_idx, self.p_star)
        
        # 実位置（観測値、重み行列の計算に使用）
        p_j_actual = neighbor_positions_actual.get(j_idx, self.p_actual)
        p_k_actual = neighbor_positions_actual.get(k_idx, self.p_actual)
        
        # オクルージョン時はξ = 0
        if self.is_occluded:
            return np.zeros(2)
        
        # 制御則の重み行列を計算（観測値から）
        from weight_matrices import calculate_weight_matrices_for_agent
        Hij, Hik, is_collinear = calculate_weight_matrices_for_agent(
            self.p_actual, p_j_actual, p_k_actual)
        
        if not is_collinear:
            Hii = Hij + Hik
            # 重み行列H_iiのノルムを保存
            self.debug_info['Hii_norm'] = np.linalg.norm(Hii)
        else:
            self.debug_info['Hii_norm'] = 0.0
        
        # 幾何学的補正項ξを計算（論文Eq. (20)）
        # 重要：重み行列は実位置から、ξは推定位置から
        xi_total = calculate_xi_correction(
            self.p_hat,      # 推定位置
            p_j_hat,          # 推定位置
            p_k_hat,          # 推定位置
            self.p_star,     # 目標位置
            p_j_star,         # 目標位置
            p_k_star,         # 目標位置
            self.p_actual,   # 実位置（観測値）
            p_j_actual,       # 実位置（観測値）
            p_k_actual        # 実位置（観測値）
        )
        
        # 論文Eq. (20): ξ = ξ_ijk + Σ_s ξ_sig
        # 現在の4エージェントシステム（リーダー2機、フォロワー2機）では：
        # - ξ_ijk: 直接の隣接エージェント（j, k）からの情報 (リーダー1, 2)
        # - ξ_sig: 拡張エッジ（フォロワー間）からの情報 = 0（エッジなし）
        xi_ijk = xi_total.copy()  # 現在の実装ではξ_ijkのみ
        xi_sig = np.zeros(2)       # 拡張エッジなし
        
        # debug_infoに保存
        self.debug_info['xi'] = xi_total
        self.debug_info['xi_ijk'] = xi_ijk
        self.debug_info['xi_sig'] = xi_sig
        
        return xi_total
    
    def set_occlusion(self, is_occluded):
        """
        オクルージョン状態を設定
        
        Args:
            is_occluded: Trueの場合、センサオクルージョン状態
        """
        self.is_occluded = is_occluded
        logger.info(f"フォロワー{self.follower_id}オクルージョン: {is_occluded}")
    
    def calculate_psi_and_tau(self, neighbor_positions_hat, neighbor_positions_star):
        """
        共線回避項ψとbearing誤差τを計算
        
        Args:
            neighbor_positions_hat: 隣接エージェントの推定位置 {agent_id: [x, z], ...}
            neighbor_positions_star: 隣接エージェントの目標位置 {agent_id: [x, z], ...}
        
        Returns:
            psi: 共線回避項 (2x1)
            tau: bearing誤差ノルム（スカラー）
        """
        j_idx, k_idx = self.neighbors
        
        # 隣接エージェントの位置を取得
        p_j_hat = neighbor_positions_hat.get(j_idx, self.p_hat)
        p_k_hat = neighbor_positions_hat.get(k_idx, self.p_hat)
        p_j_star = neighbor_positions_star.get(j_idx, self.p_star)
        p_k_star = neighbor_positions_star.get(k_idx, self.p_star)
        
        # bearing誤差τの計算（ref/pid_slaf_japanese_proof.tex Eq. 124）
        # τ_i = ||g_ij - g_ij^*||^2 + ||g_ik - g_ik^*||^2
        v_ij_hat = p_j_hat - self.p_hat
        v_ik_hat = p_k_hat - self.p_hat
        v_ij_star = p_j_star - self.p_star
        v_ik_star = p_k_star - self.p_star
        
        norm_ij_hat = np.linalg.norm(v_ij_hat)
        norm_ik_hat = np.linalg.norm(v_ik_hat)
        norm_ij_star = np.linalg.norm(v_ij_star)
        norm_ik_star = np.linalg.norm(v_ik_star)
        
        # bearing単位ベクトル
        g_ij_hat = v_ij_hat / (norm_ij_hat + 1e-9)
        g_ik_hat = v_ik_hat / (norm_ik_hat + 1e-9)
        g_ij_star = v_ij_star / (norm_ij_star + 1e-9)
        g_ik_star = v_ik_star / (norm_ik_star + 1e-9)
        
        # τの計算
        bearing_error_ij = g_ij_hat - g_ij_star
        bearing_error_ik = g_ik_hat - g_ik_star
        tau = np.linalg.norm(bearing_error_ij)**2 + np.linalg.norm(bearing_error_ik)**2
        
        # ψの計算（ref/pid_slaf_japanese_proof.tex Eq. 120）
        # ψ_i = -τ_gain * τ_i(sign(p̂_i - p_i^*) - λ_i)
        if tau < self.tau_threshold:
            # 非共線（局所化可能）の場合、ψ = 0
            psi = np.zeros(2)
        else:
            # 共線の場合、ψを計算
            # sign関数の代わりにtanhを使用（実装上の安定性のため）
            tracking_error = self.p_hat - self.p_star
            sign_approx = np.tanh(tracking_error / 0.01)  # tanhで近似
            psi = -self.tau_gain * tau * (sign_approx - self.lambda_vec)
        
        return psi, tau
    
    def update_estimator(self, xi):
        """
        推定器の更新（Eq. 39, 40, 5）
        
        ref/sim_PID_v1/system_dynamics.m の Eq. 39-40, Eq. 5 に対応
        
        ż_i = p̂_i - p_i^*                                      (Eq. 39)
        p̂̇_i = v̂_i                                             (Eq. 40)
        v̂̇_i = -k_p(p̂_i - p_i^*) - k_v(v̂_i - v_i) - k_i*z_i + a_i^* + ξ_i  (Eq. 5)
        
        注：推定器はψ_iを含まない（制御入力計算時のみ使用）
        
        Args:
            xi: 幾何学的補正項
        """
        # ż_i = p̂_i - p_i^*  (Eq. 39)
        z_dot = self.p_hat - self.p_star
        
        # p̂̇_i = v̂_i  (Eq. 40)
        p_hat_dot = self.v_hat
        
        # v̂̇_i = -k_p(p̂_i - p_i^*) - k_v(v̂_i - v_i) - k_i*z_i + a_i^* + ξ_i  (Eq. 5)
        v_hat_dot = (
            - self.k_p * (self.p_hat - self.p_star)      # P項（位置誤差）
            - self.k_v * (self.v_hat - self.v_actual)    # D項（速度フィードバック）
            - self.k_i * self.z_integral                 # I項（積分状態）
            + self.a_star                                # フィードフォワード
            + self.xi_gain * xi                          # 幾何学的補正項
        )
        
        # オイラー法で更新
        self.z_integral += z_dot * self.dt
        self.p_hat += p_hat_dot * self.dt
        self.v_hat += v_hat_dot * self.dt
    
    def calculate_control_input(self, psi=None):
        """
        制御入力の計算（下位層）
        
        ref/sim_PID_v1/system_dynamics.m の Eq. 6 に対応
        
        u_i = -k_cp(p̂_i - p_i^*) - k_cv(v_i - v_i^*) + a_i^* + ψ_i  (Eq. 6)
        
        Args:
            psi: 共線回避項（オクルージョン時のみ使用、通常時は0）
        
        Returns:
            u: 制御入力（加速度指令） [ax, az]
        """
        if psi is None:
            psi = np.zeros(2)
        
        # 位置誤差を計算
        position_error = self.p_hat - self.p_star
        
        # 不感帯処理（src/MOCAP_for2TELLOsのposition_control.pyに合わせる）
        # 不感帯内の誤差はゼロにする
        if abs(position_error[0]) < self.deadband_x:
            position_error[0] = 0.0
        if abs(position_error[1]) < self.deadband_z:
            position_error[1] = 0.0
        
        # 制御入力の計算（Eq. 6）
        u = (
            - self.k_cp * position_error                 # 位置制御項（不感帯適用済）
            - self.k_cv * (self.v_actual - self.v_star)  # 速度制御項
            + self.a_star                                # フィードフォワード
            + psi                                        # 共線回避項（オクルージョン時のみ）
        )
        
        self.debug_info['control_input'] = u
        self.debug_info['position_error_raw'] = self.p_hat - self.p_star
        self.debug_info['position_error_deadband'] = position_error
        return u
    
    def update(self, p_mocap, neighbor_positions_hat, neighbor_positions_star, neighbor_positions_actual):
        """
        完全な更新サイクル
        
        1. MOCAP測定値から実状態を更新
        2. ξを計算（オクルージョン時は0）
        3. 推定器を更新
        4. ψを計算（オクルージョン時のみ）
        5. 制御入力を計算
        
        Args:
            p_mocap: MOCAP測定位置 [x, z]
            neighbor_positions_hat: 隣接エージェントの推定位置
            neighbor_positions_star: 隣接エージェントの目標位置
            neighbor_positions_actual: 隣接エージェントの実位置
        
        Returns:
            u: 制御入力（平面加速度） [ax, az]
        """
        # 1. 実状態を更新
        self.update_actual_state(p_mocap)
        
        # 初期化直後の最初の更新：推定誤差を強制的にゼロにリセット
        if self.just_initialized:
            self.p_hat = self.p_actual.copy()
            self.v_hat = self.v_actual.copy()
            self.z_integral = np.zeros(2)
            self.just_initialized = False
            logger.info(f"フォロワー{self.follower_id}初期化後リセット: p_hat={self.p_hat}, v_hat={self.v_hat}")
            # 初回は制御入力をゼロにする
            return np.zeros(2)
        
        # 2. ξを計算（オクルージョン時は0）
        # 重要：重み行列は実位置（観測値）から計算、ξは推定位置から計算
        xi = self.calculate_xi(neighbor_positions_hat, neighbor_positions_star, neighbor_positions_actual)
        
        # 3. 推定器を更新（ψは含まない）
        self.update_estimator(xi)
        
        # 4. ψとτの計算（オクルージョン時のみ）
        if self.is_occluded:
            psi, tau = self.calculate_psi_and_tau(neighbor_positions_hat, neighbor_positions_star)
        else:
            # 通常時はψ=0、τ=0（共線回避を行わない）
            psi = np.zeros(2)
            tau = 0.0
        
        # 5. デバッグ情報を保存
        self.debug_info['psi'] = psi
        self.debug_info['tau'] = tau
        self.debug_info['is_collinear'] = tau > self.tau_threshold if self.is_occluded else False
        
        # 6. 制御入力を計算（オクルージョン時のみψを使用）
        u = self.calculate_control_input(psi)
        
        return u
    
    def get_state(self):
        """
        現在の状態を取得
        
        Returns:
            dict: 全状態変数
        """
        # 制御則の重み行列H_iiのノルムを取得
        control_weight_norm = self.debug_info.get('Hii_norm', 0.0)
        
        return {
            'follower_id': self.follower_id,
            'p_actual': self.p_actual.copy(),
            'v_actual': self.v_actual.copy(),
            'p_hat': self.p_hat.copy(),
            'v_hat': self.v_hat.copy(),
            'z_integral': self.z_integral.copy(),
            'p_star': self.p_star.copy(),
            'v_star': self.v_star.copy(),
            'a_star': self.a_star.copy(),
            'xi': self.debug_info['xi'].copy(),
            'xi_ijk': self.debug_info.get('xi_ijk', np.zeros(2)).copy(),
            'xi_sig': self.debug_info.get('xi_sig', np.zeros(2)).copy(),
            'psi': self.debug_info['psi'].copy(),
            'tau': self.debug_info['tau'],  # bearing誤差
            'is_collinear': self.debug_info['is_collinear'],
            'is_occluded': self.is_occluded,  # オクルージョン状態
            'control_input': self.debug_info['control_input'].copy(),
            'control_weight_norm': control_weight_norm,  # 制御則の重み行列H_iiのノルム
            'k_p': self.k_p,  # 位置ゲイン
            'k_v': self.k_v,  # 速度ゲイン
            'k_cv': self.k_cv  # 制御器速度ゲイン（動的に変更される）
        }
    
    def get_errors(self):
        """
        誤差を計算
        
        Returns:
            dict: 各種誤差
        """
        # 推定誤差
        e_p_hat = self.p_hat - self.p_actual
        e_v_hat = self.v_hat - self.v_actual
        
        # 追跡誤差
        e_p_bar = self.p_actual - self.p_star
        e_v_bar = self.v_actual - self.v_star
        
        return {
            'estimation_position_error': e_p_hat,
            'estimation_velocity_error': e_v_hat,
            'tracking_position_error': e_p_bar,
            'tracking_velocity_error': e_v_bar,
            'estimation_position_error_norm': np.linalg.norm(e_p_hat),
            'estimation_velocity_error_norm': np.linalg.norm(e_v_hat),
            'tracking_position_error_norm': np.linalg.norm(e_p_bar),
            'tracking_velocity_error_norm': np.linalg.norm(e_v_bar)
        }


class SLAFSystemManager:
    """SLAF システム全体管理クラス（4エージェント構成）"""
    
    def __init__(self, dt=0.1):
        """
        Args:
            dt: 制御周期（秒）
        """
        self.dt = dt
        
        # グラフ構造（Plan仕様）
        # V = {1, 2, 3, 4}
        # V_l = {1, 2} (仮想リーダー)
        # V_f = {3, 4} (実機フォロワー)
        # N_3 = {1, 2}, N_4 = {1, 3}
        
        self.num_leaders = 2
        self.num_followers = 2
        
        # フォロワーの隣接関係
        self.neighbor_map = {
            3: [1, 2],  # フォロワー3の隣人：リーダー1, 2
            4: [1, 3]   # フォロワー4の隣人：リーダー1, フォロワー3
        }
        
        # フォロワーコントローラーを初期化
        self.follower_controllers = {}
        for follower_id, neighbors in self.neighbor_map.items():
            self.follower_controllers[follower_id] = SLAFPIDController(
                follower_id=follower_id,
                neighbors=neighbors,
                dt=dt
            )
        
        logger.info(f"SLAFシステム初期化: {self.num_leaders}リーダー + {self.num_followers}フォロワー")
    
    def initialize_followers(self, follower_positions, follower_velocities=None):
        """
        フォロワーの推定器を初期化（Assumption: 初期推定誤差零）
        
        理論的根拠（ref/pid_slaf_japanese_proof.tex Assumption 3.1）：
        - p̂_i(0) = p_i(0): 初期推定位置 = 初期実位置
        - v̂_i(0) = v_i(0): 初期推定速度 = 初期実速度
        - z_i(0) = 0: 積分項の初期値はゼロ
        
        この仮定により、完全収束定理（Theorem 3.1）が保証される。
        
        Args:
            follower_positions: {follower_id: np.array([x, z]), ...}
                                MOCAPから取得した初期位置
            follower_velocities: {follower_id: np.array([vx, vz]), ...} (オプション)
                                 初期速度（Noneの場合はゼロ）
        """
        if follower_velocities is None:
            follower_velocities = {}
        
        initialized_count = 0
        for follower_id, controller in self.follower_controllers.items():
            p_initial = follower_positions.get(follower_id, None)
            v_initial = follower_velocities.get(follower_id, None)
            
            if p_initial is not None:
                controller.initialize_state(p_initial, v_initial)
                v_str = f"[{v_initial[0]:.3f}, {v_initial[1]:.3f}]" if v_initial is not None else "[0.000, 0.000]"
                logger.info(f"フォロワー{follower_id}推定器初期化: p_initial=[{p_initial[0]:.3f}, {p_initial[1]:.3f}], v_initial={v_str}")
                initialized_count += 1
            else:
                logger.warning(f"フォロワー{follower_id}の初期位置が提供されませんでした（ゼロ初期化のまま）")
        
        if initialized_count == len(self.follower_controllers):
            logger.info(f"✅ 全フォロワー({initialized_count}個)の推定器初期化完了（初期推定誤差零）")
        else:
            logger.warning(f"⚠️ 一部のフォロワー初期化失敗: {initialized_count}/{len(self.follower_controllers)}")
    
    def set_follower_occlusion(self, follower_id, is_occluded):
        """
        特定のフォロワーのオクルージョン状態を設定
        
        Args:
            follower_id: フォロワーID（3 or 4）
            is_occluded: Trueの場合、センサオクルージョン状態
        """
        if follower_id in self.follower_controllers:
            self.follower_controllers[follower_id].set_occlusion(is_occluded)
            logger.info(f"フォロワー{follower_id}オクルージョン状態設定: {is_occluded}")
        else:
            logger.warning(f"フォロワー{follower_id}が見つかりません")
    
    def update_followers(self, mocap_positions, leader_states):
        """
        全フォロワーを更新
        
        Args:
            mocap_positions: {follower_id: [x, z], ...}
            leader_states: リーダーの状態リスト [leader1_state, leader2_state]
        
        Returns:
            dict: {follower_id: control_input, ...}
        """
        control_inputs = {}
        
        for follower_id, controller in self.follower_controllers.items():
            # 隣接エージェントの位置を収集
            neighbor_positions_hat = {}
            neighbor_positions_star = {}
            neighbor_positions_actual = {}
            
            for neighbor_id in controller.neighbors:
                if neighbor_id <= self.num_leaders:
                    # リーダーの場合
                    leader_idx = neighbor_id - 1
                    leader_state = leader_states[leader_idx]
                    neighbor_positions_hat[neighbor_id] = leader_state['position']
                    neighbor_positions_star[neighbor_id] = leader_state['target_position']
                    neighbor_positions_actual[neighbor_id] = leader_state['position']
                else:
                    # フォロワーの場合
                    other_controller = self.follower_controllers[neighbor_id]
                    neighbor_positions_hat[neighbor_id] = other_controller.p_hat
                    neighbor_positions_star[neighbor_id] = other_controller.p_star
                    neighbor_positions_actual[neighbor_id] = other_controller.p_actual
            
            # フォロワーを更新
            p_mocap = mocap_positions.get(follower_id, controller.p_actual)
            u = controller.update(
                p_mocap,
                neighbor_positions_hat,
                neighbor_positions_star,
                neighbor_positions_actual
            )
            
            control_inputs[follower_id] = u
        
        return control_inputs
    
    def set_follower_targets(self, follower_targets):
        """
        フォロワーの目標軌道を設定
        
        Args:
            follower_targets: {follower_id: {'position': [x,z], 'velocity': [vx,vz], 'acceleration': [ax,az]}, ...}
        """
        for follower_id, target in follower_targets.items():
            if follower_id in self.follower_controllers:
                controller = self.follower_controllers[follower_id]
                controller.set_target_trajectory(
                    target.get('position', [0.0, 0.0]),
                    target.get('velocity', [0.0, 0.0]),
                    target.get('acceleration', [0.0, 0.0])
                )
    
    def get_all_states(self):
        """
        全フォロワーの状態を取得
        
        Returns:
            dict: {follower_id: state, ...}
        """
        return {fid: controller.get_state() 
                for fid, controller in self.follower_controllers.items()}
    
    def get_all_errors(self):
        """
        全フォロワーの誤差を取得
        
        Returns:
            dict: {follower_id: errors, ...}
        """
        return {fid: controller.get_errors() 
                for fid, controller in self.follower_controllers.items()}


if __name__ == "__main__":
    # テストコード
    print("=== SLAF PIDコントローラテスト ===")
    
    # システムマネージャーを初期化
    manager = SLAFSystemManager(dt=0.1)
    
    # フォロワー初期位置
    follower_positions = {
        3: [0.0, -0.5],  # フォロワー3
        4: [0.0, 0.5]    # フォロワー4
    }
    manager.initialize_followers(follower_positions)
    
    # リーダー状態（仮想）
    leader_states = [
        {'position': np.array([0.0, -0.5]), 'target_position': np.array([0.0, -0.5])},
        {'position': np.array([0.0, 0.5]), 'target_position': np.array([0.0, 0.5])}
    ]
    
    # フォロワー目標
    follower_targets = {
        3: {'position': [1.0, -0.5], 'velocity': [0.1, 0.0], 'acceleration': [0.0, 0.0]},
        4: {'position': [1.0, 0.5], 'velocity': [0.1, 0.0], 'acceleration': [0.0, 0.0]}
    }
    manager.set_follower_targets(follower_targets)
    
    # シミュレーション
    print("\n初期状態:")
    states = manager.get_all_states()
    for fid, state in states.items():
        print(f"  フォロワー{fid}: p={state['p_actual']}, p_hat={state['p_hat']}")
    
    print("\n制御ループ:")
    for step in range(10):
        # MOCAP測定（簡易シミュレーション）
        mocap_positions = {
            3: follower_positions[3] + np.array([0.01 * step, 0.0]),
            4: follower_positions[4] + np.array([0.01 * step, 0.0])
        }
        
        # 更新
        control_inputs = manager.update_followers(mocap_positions, leader_states)
        
        if step % 2 == 0:
            print(f"  ステップ{step}:")
            for fid, u in control_inputs.items():
                state = manager.follower_controllers[fid].get_state()
                errors = manager.follower_controllers[fid].get_errors()
                print(f"    フォロワー{fid}: u={u}, 追跡誤差={errors['tracking_position_error_norm']:.4f}")
