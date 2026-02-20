#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slaf_observer_controller.py - オブザーバ型SLAF制御システム

ref_v2/sim_v2/system_dynamics.m と v4_observer.tex の忠実なPython実装
オブザーバ型SLAF（Luenbergerオブザーバ型同時分散位置推定・隊形追従制御）を実現

構成：
- 推定器：Luenbergerオブザーバ型（速度測定フィードバック）
- 制御器：PD型 + フィードフォワード
- グラフ構造：仮想リーダー2機 + 実機フォロワー2機（4エージェント）

理論：v4_observer.tex 式(143), 式(144), 式(174)
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


class SLAFObserverController:
    """オブザーバ型SLAF制御器クラス"""
    
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
        
        # === オブザーバ型パラメータ（ref_v2/sim_v2/define_trajectory_simple.m準拠） ===
        # オブザーバゲイン
        self.K_obs = 0.0   # 速度推定誤差の収束速度（v4_observer.tex 定理4.2）
        
        # 制御器ゲイン
        self.K_p = 5.0    # 位置制御ゲイン（固有振動数 ω_n = √K_p）
        self.K_v = 2.0     # 速度制御ゲイン（減衰比 ζ = K_v/(2√K_p)）
        
        # 補正項のゲイン（実装上の調整）
        self.w_xi = 5.0     # ξ項のゲイン（シミュレーションと同じ）
        self.w_psi = 1.0    # ψ項のゲイン（シミュレーションと同じ）
        
        # 共線回避パラメータ（v4_observer.tex 定理4.2）
        self.gamma = 1.0        # λ調整ゲイン (0 < γ ≤ 1)
        self.lambda_max = 0.99  # λ飽和限界 (0 < λ_max < 1)
        self.lambda_vec = np.array([0.1, 0.1])  # 水平2次元の調整ベクトル
        self.tau_threshold = 0.9  # 共線判定閾値
        self.tau_gain = 1.0  # τのゲイン（共線回避の強さを調整）
        
        # 不感帯パラメータ（v1と同じ）
        self.deadband_x = 0.0  # x方向（前後）の不感帯 (m)
        self.deadband_z = 0.0  # z方向（左右）の不感帯 (m)
        
        # オクルージョンフラグ
        self.is_occluded = False  # センサオクルージョン状態
        
        # 初期化フラグ（初期化直後の最初の更新で推定誤差をゼロにリセット）
        self.just_initialized = False
        
        # === 状態変数（2次元: x-z平面） ===
        # 実位置・実速度（MOCAP測定値から設定）
        self.p_actual = np.zeros(2)  # [x, z]
        self.v_actual = np.zeros(2)  # [vx, vz]（数値微分で推定）
        
        # 推定位置・推定速度（オブザーバ）
        self.p_hat = np.zeros(2)  # [x_hat, z_hat]
        self.v_hat = np.zeros(2)  # [vx_hat, vz_hat]
        
        # ★重要: v2では積分状態 z_integral は存在しない（I制御なし）★
        
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
            'tau': 0.0,
            'is_collinear': False,
            'control_input': np.zeros(2),
            'observer_feedback': np.zeros(2)
        }
        
        logger.info(f"SLAFオブザーバ型コントローラ初期化: フォロワー{follower_id}, 隣接{neighbors}")
    
    def set_gains(self, K_obs=None, K_p=None, K_v=None, 
                  gamma=None, lambda_max=None, w_xi=None, w_psi=None):
        """
        ゲインを動的に設定（オプション）
        
        Args:
            K_obs: オブザーバゲイン (> 0)
            K_p: 位置制御ゲイン (> 0)
            K_v: 速度制御ゲイン (> 0)
            gamma: λ調整ゲイン (0 < γ ≤ 1)
            lambda_max: λ飽和限界 (0 < λ_max < 1)
            w_xi: ξ項のゲイン（実装上の調整）
            w_psi: ψ項のゲイン（実装上の調整）
        """
        if K_obs is not None:
            assert K_obs > 0, "K_obs must be positive"
            self.K_obs = K_obs
        
        if K_p is not None:
            assert K_p > 0, "K_p must be positive"
            self.K_p = K_p
        
        if K_v is not None:
            assert K_v > 0, "K_v must be positive"
            self.K_v = K_v
        
        if gamma is not None:
            assert 0 < gamma <= 1, "gamma must be in (0, 1]"
            self.gamma = gamma
        
        if lambda_max is not None:
            assert 0 < lambda_max < 1, "lambda_max must be in (0, 1)"
            self.lambda_max = lambda_max
        
        if w_xi is not None:
            self.w_xi = w_xi
        
        if w_psi is not None:
            self.w_psi = w_psi
    
    def set_deadbands(self, deadband_x=None, deadband_z=None):
        """
        不感帯を設定
        
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
        Tキー押下時の初期化：実位置・推定位置を同じ値に設定（推定誤差ゼロ）
        
        v4_observer.tex 仮定1: 初期推定誤差の有界性
        Tキー押下時は推定誤差を零に設定
        
        Args:
            p_initial: 初期位置 [x, z]（MOCAPから取得）
            v_initial: 初期速度 [vx, vz]（Noneの場合はゼロ）
        """
        self.p_actual = np.array(p_initial, dtype=float)
        self.v_actual = np.array(v_initial, dtype=float) if v_initial is not None else np.zeros(2)
        
        # Assumption: p̂_i(0) = p_i(0), v̂_i(0) = v_i(0)
        self.p_hat = self.p_actual.copy()
        self.v_hat = self.v_actual.copy()
        
        # ★重要: v2では積分状態 z_i は存在しない★
        
        # 履歴初期化
        self.position_history = [self.p_actual.copy()]
        
        # 初期化フラグを設定（次の更新で推定誤差を強制的にゼロにする）
        self.just_initialized = True
        
        logger.info(f"フォロワー{self.follower_id}初期化: p={self.p_actual}, v={self.v_actual}")
    
    def set_estimated_state(self, p_hat_initial, v_hat_initial=None):
        """
        Hキー押下時の初期化：推定位置のみを設定、実位置は変更しない
        
        オブザーバ性能検証のため、初期推定誤差を与える
        実位置はMOCAPから取得し続けるため、ここでは変更しない
        
        Args:
            p_hat_initial: 初期推定位置 [x, z]（絶対座標、MOCAP座標系）
            v_hat_initial: 初期推定速度 [vx, vz]（Noneの場合はゼロ）
        """
        # 推定位置・速度を設定
        self.p_hat = np.array(p_hat_initial, dtype=float)
        self.v_hat = np.array(v_hat_initial, dtype=float) if v_hat_initial is not None else np.zeros(2)
        
        # just_initializedフラグは立てない（実位置との同期は不要）
        self.just_initialized = False
        
        # 推定誤差を計算（現在の実位置との差）
        estimation_error = self.p_hat - self.p_actual
        error_norm = np.linalg.norm(estimation_error)
        
        logger.info(f"フォロワー{self.follower_id}推定状態設定: p_hat={self.p_hat}, v_hat={self.v_hat}, "
                   f"推定誤差={estimation_error}, ノルム={error_norm:.3f}m")
    
    def update_actual_state(self, p_mocap):
        """
        MOCAP測定値から実位置・実速度を更新
        
        v2でも実速度は数値微分で推定（v1と同じ）
        
        Args:
            p_mocap: MOCAP測定位置 [x, z]
        """
        # 前回の位置を保存
        prev_position = self.p_actual.copy()
        
        # 実位置を更新
        self.p_actual = np.array(p_mocap, dtype=float)
        
        # 実速度を数値微分で推定（v1と同じ）
        self.v_actual = (self.p_actual - prev_position) / self.dt
        
        # 履歴を更新
        self.position_history.append(self.p_actual.copy())
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
    
    def set_target_trajectory(self, p_star, v_star, a_star):
        """
        目標軌道を設定
        
        v2では目標加速度 a_star は完全にキャンセルされる（v4_observer.tex 注釈）
        しかし、フィードフォワード項として明示的に使用
        
        Args:
            p_star: 目標位置 [x, z]
            v_star: 目標速度 [vx, vz]
            a_star: 目標加速度 [ax, az]（v1と同様に継承、一定値で検証）
        """
        self.p_star = np.array(p_star, dtype=float)
        self.v_star = np.array(v_star, dtype=float)
        self.a_star = np.array(a_star, dtype=float)
    
    def calculate_xi(self, neighbor_positions_hat, neighbor_positions_star, neighbor_positions_actual):
        """
        幾何学的補正項ξを計算
        
        MATLABのcalculate_control_logic.m:69-103に厳密準拠
        
        重要：
        - 重み行列H：観測値（実位置、MOCAP測定）から計算
        - オクルージョン時：Hをゼロに、ξもゼロに
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
        
        # ★MATLABのcalculate_control_logic.m:69-76に準拠★
        # オクルージョンフラグをチェック
        # フォロワー3: 隣接 = リーダー1, 2 → オクルージョンなし（リーダーは常に観測可能）
        # フォロワー4: 隣接 = リーダー1, フォロワー3 → self.is_occludedで制御
        # 実装: self.is_occluded = Trueの場合、j_idxまたはk_idxがフォロワーならオクルージョン
        
        # オクルージョンフラグ（MATLABのocclusion_j, occlusion_kに対応）
        # リーダー（1, 2）は常に観測可能、フォロワー（3, 4）は自身のオクルージョン状態に依存
        occlusion_j = False
        occlusion_k = False
        
        if self.is_occluded:
            # 自身がオクルージョン時、フォロワーへの観測が遮蔽される
            # フォロワー4の場合：j=1（リーダー、OK）、k=3（フォロワー、オクルージョン）
            if k_idx > 2:  # k_idxがフォロワー（3 or 4）
                occlusion_k = True
        
        # 制御則の重み行列を計算（観測値から、オクルージョンフラグを渡す）
        from weight_matrices import calculate_weight_matrices_for_agent
        Hij, Hik, is_unlocalizable = calculate_weight_matrices_for_agent(
            self.p_actual, p_j_actual, p_k_actual, occlusion_j, occlusion_k)
        
        # debug_infoに保存
        if not is_unlocalizable:
            Hii = Hij + Hik
            self.debug_info['Hii_norm'] = np.linalg.norm(Hii)
        else:
            self.debug_info['Hii_norm'] = 0.0
        
        self.debug_info['is_unlocalizable'] = is_unlocalizable
        
        # ★MATLABのcalculate_control_logic.m:88-103に準拠★
        # オクルージョン時（is_unlocalizable=True）はξ = 0
        if is_unlocalizable:
            xi_total = np.zeros(2)
        else:
            # B行列を構築してξを計算
            # 簡易実装：直接計算
            Hii = Hij + Hik
            p_rel_ij_hat = p_j_hat - self.p_hat
            p_rel_ik_hat = p_k_hat - self.p_hat
            xi_total = Hii.T @ Hij @ p_rel_ij_hat + Hii.T @ Hik @ p_rel_ik_hat
        
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
        
        手動でオクルージョンを起こす（v1と同じ）
        
        Args:
            is_occluded: Trueの場合、センサオクルージョン状態
        """
        self.is_occluded = is_occluded
        logger.info(f"フォロワー{self.follower_id}オクルージョン: {is_occluded}")
    
    def calculate_psi_and_tau(self, neighbor_positions_hat, neighbor_positions_star):
        """
        共線回避項ψとbearing誤差τを計算
        
        MATLABのcalculate_control_logic.m:105-200に厳密準拠
        
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
        
        # ★MATLABのcalculate_control_logic.m:107-122に準拠★
        # bearing単位ベクトルの計算
        v_ij_star = p_j_star - self.p_star
        g_ij_star = v_ij_star / (np.linalg.norm(v_ij_star) + 1e-9)
        
        v_ik_star = p_k_star - self.p_star
        g_ik_star = v_ik_star / (np.linalg.norm(v_ik_star) + 1e-9)
        
        v_ij_hat = p_j_hat - self.p_hat
        g_ij_hat = v_ij_hat / (np.linalg.norm(v_ij_hat) + 1e-9)
        
        v_ik_hat = p_k_hat - self.p_hat
        g_ik_hat = v_ik_hat / (np.linalg.norm(v_ik_hat) + 1e-9)
        
        # ★MATLABのcalculate_control_logic.m:120-122に準拠★
        # bearing誤差の計算
        bearing_error = np.linalg.norm(g_ij_hat - g_ij_star) + np.linalg.norm(g_ik_hat - g_ik_star)
        
        # ★MATLABのcalculate_control_logic.m:124-160に準拠★
        # is_unlocalizableの判定は既にcalculate_xiで実施済み
        # ここでは is_occluded または is_unlocalizable をチェック
        is_unlocalizable = self.debug_info.get('is_unlocalizable', False)
        
        if is_unlocalizable:
            # 局所化不可能状態（共線またはオクルージョン）
            # bearing誤差に基づいてτを設定
            tau = bearing_error
        else:
            # 局所化可能状態（正常）では、τは常にゼロ
            tau = 0.0
        
        # ★MATLABのcalculate_control_logic.m:164-200に準拠★
        # λの計算（式ref{eq:lambda}）
        # Δv_i = (v_i - v_i*) - (v_hat_i - v_i)
        Delta_v_i = (self.v_actual - self.v_star) - (self.v_hat - self.v_actual)
        
        # λ_i = -γ * Δv_i
        lambda_i = -self.gamma * Delta_v_i
        
        # 飽和処理
        lambda_norm = np.linalg.norm(lambda_i)
        if lambda_norm > self.lambda_max:
            lambda_i = self.lambda_max * lambda_i / lambda_norm
        
        # ★MATLABのcalculate_control_logic.m:195-200に準拠★
        # ψの計算（式ref{eq:psi}）
        # ψ_i = -τ_i(sign(p̂_i - p_i*) - λ_i)
        sign_term = np.sign(self.p_hat - self.p_star)
        psi = -tau * (sign_term - lambda_i)
        
        return psi, tau
    
    def update_estimator(self, xi):
        """
        推定器の更新（Luenbergerオブザーバ型）
        
        v4_observer.tex 式(143), 式(144) に厳密準拠
        ref_v2/sim_v2/system_dynamics.m 76-86行 に対応
        
        ˙p̂ᵢ = vᵢ + ξᵢ                    (式143)
        ˙v̂ᵢ = uᵢ + K_obs(vᵢ - v̂ᵢ)        (式144)
        
        ★重要な変更点（v1 PID階層型との違い）★
        - 積分状態 z_i は存在しない（I制御なし）
        - P項、D項、I項 → オブザーバフィードバック項
        - 実速度 vᵢ を直接使用
        - 前回の制御入力 u を使用（カスケード構造）
        
        Args:
            xi: 幾何学的補正項
        """
        # 前回の制御入力を取得（初期化時は零）
        u_prev = self.debug_info.get('control_input', np.zeros(2))
        
        # ˙p̂ᵢ = vᵢ + ξᵢ  (式143)
        # ★重要: 実速度 vᵢ を直接使用（測定可能と仮定）★
        p_hat_dot = self.v_actual + self.w_xi * xi
        
        # ˙v̂ᵢ = uᵢ + K_obs(vᵢ - v̂ᵢ)  (式144)
        # ★重要: オブザーバフィードバック項 K_obs(vᵢ - v̂ᵢ)★
        observer_feedback = self.K_obs * (self.v_actual - self.v_hat)
        v_hat_dot = u_prev + observer_feedback
        
        # Euler法で更新
        self.p_hat += p_hat_dot * self.dt
        self.v_hat += v_hat_dot * self.dt
        
        # デバッグ情報
        self.debug_info['p_hat_dot'] = p_hat_dot
        self.debug_info['v_hat_dot'] = v_hat_dot
        self.debug_info['observer_feedback'] = observer_feedback
    
    def calculate_control_input(self, psi=None):
        """
        制御入力の計算（PD型 + フィードフォワード）
        
        v4_observer.tex 式(174) に厳密準拠
        ref_v2/sim_v2/system_dynamics.m 56-64行 に対応
        
        uᵢ = p̈ᵢ* - K_p(p̂ᵢ - pᵢ*) - K_v(vᵢ - ṗᵢ*) + ψᵢ  (式174)
        
        ★重要な変更点（v1 PID階層型との違い）★
        - フィードフォワード項 p̈ᵢ* を先頭に配置
        - 速度誤差項は (vᵢ - ṗᵢ*) = (vᵢ - vᵢ*)
        - 符号を明示的に記述
        
        Args:
            psi: 共線回避項（オクルージョン時のみ使用） [2,]
        
        Returns:
            u: 制御入力（加速度指令） [ax, az]
        """
        if psi is None:
            psi = np.zeros(2)
        
        # 位置誤差を計算（推定位置ベース）
        position_error = self.p_hat - self.p_star
        
        # 不感帯処理（v1と同じ）
        if abs(position_error[0]) < self.deadband_x:
            position_error[0] = 0.0
        if abs(position_error[1]) < self.deadband_z:
            position_error[1] = 0.0
        
        # 速度誤差を計算（実速度ベース）
        velocity_error = self.v_actual - self.v_star
        
        # 制御入力の計算（式174）
        # uᵢ = p̈ᵢ* - K_p(p̂ᵢ - pᵢ*) - K_v(vᵢ - ṗᵢ*) + ψᵢ
        u = (
            self.a_star                      # p̈ᵢ* フィードフォワード
            - self.K_p * position_error      # -K_p(p̂ᵢ - pᵢ*) 位置制御項
            - self.K_v * velocity_error      # -K_v(vᵢ - ṗᵢ*) 速度制御項
            + self.w_psi * psi               # w_psi * ψᵢ 共線回避項（ゲイン調整）
        )
        
        # デバッグ情報
        self.debug_info['control_input'] = u
        self.debug_info['position_error_raw'] = self.p_hat - self.p_star
        self.debug_info['position_error_deadband'] = position_error
        self.debug_info['velocity_error'] = velocity_error
        self.debug_info['feedforward'] = self.a_star
        
        return u
    
    def update(self, p_mocap, neighbor_positions_hat, neighbor_positions_star, neighbor_positions_actual):
        """
        完全な更新サイクル
        
        1. MOCAP測定値から実状態を更新
        2. ξを計算（オクルージョン時は0）
        3. 推定器を更新（オブザーバ型）
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
            # ★重要: v2では積分状態 z_integral は存在しない★
            self.just_initialized = False
            logger.info(f"フォロワー{self.follower_id}初期化後リセット: p_hat={self.p_hat}, v_hat={self.v_hat}")
            # 初回は制御入力をゼロにする
            return np.zeros(2)
        
        # 2. ξを計算（オクルージョン時は0）
        # 重要：重み行列は実位置（観測値）から計算、ξは推定位置から計算
        xi = self.calculate_xi(neighbor_positions_hat, neighbor_positions_star, neighbor_positions_actual)
        
        # 3. 推定器を更新（オブザーバ型、ψは含まない）
        self.update_estimator(xi)
        
        # 4. ψとτの計算（MATLABのcalculate_control_logic.m:150-200に準拠）
        # is_unlocalizableの判定は calculate_xi で実施済み
        # オクルージョン時または共線時にψ/τを計算
        is_unlocalizable = self.debug_info.get('is_unlocalizable', False)
        
        if is_unlocalizable:
            # 局所化不可能状態（共線またはオクルージョン）
            psi, tau = self.calculate_psi_and_tau(neighbor_positions_hat, neighbor_positions_star)
        else:
            # 局所化可能状態（正常）では、ψ=0、τ=0
            psi = np.zeros(2)
            tau = 0.0
        
        # 5. デバッグ情報を保存
        self.debug_info['psi'] = psi
        self.debug_info['tau'] = tau
        self.debug_info['is_collinear'] = is_unlocalizable
        
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
            # ★重要: v2では積分状態 z_integral は存在しない★
            'p_star': self.p_star.copy(),
            'v_star': self.v_star.copy(),
            'a_star': self.a_star.copy(),
            'xi': self.debug_info['xi'].copy(),
            'xi_ijk': self.debug_info.get('xi_ijk', self.debug_info['xi']).copy(),  # v1互換
            'xi_sig': self.debug_info.get('xi_sig', np.zeros(2)).copy(),  # v1互換
            'psi': self.debug_info['psi'].copy(),
            'tau': self.debug_info['tau'],
            'is_collinear': self.debug_info['is_collinear'],
            'control_input': self.debug_info['control_input'].copy(),
            'observer_feedback': self.debug_info.get('observer_feedback', np.zeros(2)).copy(),
            'is_occluded': self.is_occluded,
            'control_weight_norm': control_weight_norm,
            'estimation_error_p': np.linalg.norm(self.p_hat - self.p_actual),
            'estimation_error_v': np.linalg.norm(self.v_hat - self.v_actual),
            'tracking_error': np.linalg.norm(self.p_actual - self.p_star)
        }
    
    def get_errors(self):
        """
        誤差を計算（v1互換）
        
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
    """
    複数フォロワーのSLAFシステムを統合管理するクラス
    
    v1と同じ構造、SLAFObserverControllerを使用
    """
    
    def __init__(self, follower_configs, dt=0.1):
        """
        Args:
            follower_configs: フォロワー設定のリスト
                例：[{'id': 3, 'neighbors': [1, 2]}, {'id': 4, 'neighbors': [1, 3]}]
            dt: 制御周期
        """
        self.controllers = {}
        for config in follower_configs:
            follower_id = config['id']
            neighbors = config['neighbors']
            self.controllers[follower_id] = SLAFObserverController(
                follower_id=follower_id,
                neighbors=neighbors,
                dt=dt
            )
        
        logger.info(f"SLAFシステムマネージャ初期化: {len(self.controllers)}機のフォロワー")
    
    def get_controller(self, follower_id):
        """指定IDのコントローラを取得"""
        return self.controllers.get(follower_id)
    
    # ★v1互換性のため★
    @property
    def follower_controllers(self):
        """v1互換性：controllersへのエイリアス"""
        return self.controllers
    
    def set_all_gains(self, **kwargs):
        """全フォロワーのゲインを一括設定"""
        for controller in self.controllers.values():
            controller.set_gains(**kwargs)
    
    def set_follower_occlusion(self, follower_id, is_occluded):
        """
        特定のフォロワーのオクルージョン状態を設定（v1互換）
        
        Args:
            follower_id: フォロワーID（3 or 4）
            is_occluded: Trueの場合、センサオクルージョン状態
        """
        if follower_id in self.controllers:
            self.controllers[follower_id].set_occlusion(is_occluded)
            logger.info(f"フォロワー{follower_id}オクルージョン状態設定: {is_occluded}")
        else:
            logger.warning(f"フォロワー{follower_id}が見つかりません")
    
    def set_follower_targets(self, follower_targets):
        """
        フォロワーの目標軌道を設定（v1互換）
        
        Args:
            follower_targets: {follower_id: {'position': [x,z], 'velocity': [vx,vz], 'acceleration': [ax,az]}, ...}
        """
        for follower_id, target in follower_targets.items():
            if follower_id in self.controllers:
                controller = self.controllers[follower_id]
                controller.set_target_trajectory(
                    target.get('position', [0.0, 0.0]),
                    target.get('velocity', [0.0, 0.0]),
                    target.get('acceleration', [0.0, 0.0])
                )
    
    def initialize_all_states(self, initial_positions):
        """
        全フォロワーの初期状態を設定
        
        Args:
            initial_positions: {follower_id: [x, z], ...}
        """
        for follower_id, controller in self.controllers.items():
            if follower_id in initial_positions:
                controller.initialize_state(initial_positions[follower_id])
    
    def set_all_targets(self, target_positions, target_velocities, target_accelerations):
        """
        全フォロワーの目標軌道を設定
        
        Args:
            target_positions: {follower_id: [x, z], ...}
            target_velocities: {follower_id: [vx, vz], ...}
            target_accelerations: {follower_id: [ax, az], ...}
        """
        for follower_id, controller in self.controllers.items():
            if follower_id in target_positions:
                controller.set_target_trajectory(
                    target_positions[follower_id],
                    target_velocities[follower_id],
                    target_accelerations[follower_id]
                )
    
    def update_followers(self, mocap_positions, leader_states):
        """
        全フォロワーを更新（v1互換API）
        
        Args:
            mocap_positions: {follower_id: [x, z], ...}
            leader_states: リーダーの状態リスト [leader1_state, leader2_state]
                各leader_state: {'position': [x,z], 'target_position': [x,z]}
        
        Returns:
            dict: {follower_id: control_input [ax, az], ...}
        """
        control_inputs = {}
        num_leaders = len(leader_states)
        
        for follower_id, controller in self.controllers.items():
            # 隣接エージェントの位置を収集
            neighbor_positions_hat = {}
            neighbor_positions_star = {}
            neighbor_positions_actual = {}
            
            for neighbor_id in controller.neighbors:
                if neighbor_id <= num_leaders:
                    # リーダーの場合
                    leader_idx = neighbor_id - 1
                    leader_state = leader_states[leader_idx]
                    neighbor_positions_hat[neighbor_id] = leader_state['position']
                    neighbor_positions_star[neighbor_id] = leader_state['target_position']
                    neighbor_positions_actual[neighbor_id] = leader_state['position']
                else:
                    # フォロワーの場合
                    other_controller = self.controllers[neighbor_id]
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
    
    def update_all(self, mocap_positions, leader_positions, follower_positions_hat, 
                   follower_positions_star, follower_positions_actual):
        """
        全フォロワーの制御入力を計算
        
        Args:
            mocap_positions: {follower_id: [x, z], ...}（MOCAP測定値）
            leader_positions: {leader_id: [x, z], ...}
            follower_positions_hat: {follower_id: [x, z], ...}（推定位置）
            follower_positions_star: {follower_id: [x, z], ...}（目標位置）
            follower_positions_actual: {follower_id: [x, z], ...}（実位置）
        
        Returns:
            control_inputs: {follower_id: [ax, az], ...}
        """
        # 全エージェント位置を統合
        all_positions_hat = {**leader_positions, **follower_positions_hat}
        all_positions_star = {**leader_positions, **follower_positions_star}
        all_positions_actual = {**leader_positions, **follower_positions_actual}
        
        control_inputs = {}
        for follower_id, controller in self.controllers.items():
            if follower_id in mocap_positions:
                u = controller.update(
                    mocap_positions[follower_id],
                    all_positions_hat,
                    all_positions_star,
                    all_positions_actual
                )
                control_inputs[follower_id] = u
        
        return control_inputs
    
    def get_all_states(self):
        """全フォロワーの状態を取得"""
        return {fid: ctrl.get_state() for fid, ctrl in self.controllers.items()}
