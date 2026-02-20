#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
weight_matrices.py - Bearingベースの重み行列計算モジュール

PID階層型SLAFにおける幾何学的補正項ξの計算に使用する重み行列を計算します。
ref/sim_PID_v1/calculate_weight_matrices.m の Python実装です。

Bearingベース（方向ベクトル）の相対測定から重み行列Hijを計算します。
ratio-of-distanceは使用しません。
"""

import numpy as np
import logging

# ログ設定
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 共線判定の閾値（MATLABのcalculate_weight_matrices.m:33に準拠）
COLLINEARITY_THRESHOLD = 0.999 # |cos(角度)| > 0.8 → 共線とみなす（約2.56度以下）


def calculate_weight_matrix_bearing(p_i, p_j):
    """
    Bearingベースの重み行列Hijを計算
    
    Args:
        p_i: エージェントiの位置 [x, y, z] (3D) または [x, z] (2D)
        p_j: エージェントjの位置 [x, y, z] (3D) または [x, z] (2D)
    
    Returns:
        Hij: 重み行列 (2x2 または 3x3)
    """
    p_i = np.array(p_i, dtype=float).flatten()
    p_j = np.array(p_j, dtype=float).flatten()
    
    dim = len(p_i)
    
    # 相対位置ベクトル
    q_ij = p_j - p_i
    norm_qij = np.linalg.norm(q_ij)
    
    if norm_qij < 1e-6:
        # 位置が一致している場合はゼロ行列
        logger.warning(f"エージェント位置が一致: p_i={p_i}, p_j={p_j}")
        return np.zeros((dim, dim))
    
    # 正規化ベクトル（bearing）
    g_ij = q_ij / norm_qij
    
    # 重み行列 Hij = (I - g_ij * g_ij^T) / ||q_ij||
    # これはbearing測定の情報行列に対応
    I = np.eye(dim)
    Hij = (I - np.outer(g_ij, g_ij)) / norm_qij
    
    return Hij


def check_collinearity(p_i, p_j, p_k):
    """
    3点の共線状態をチェック（MATLABのcalculate_weight_matrices.m:47-52に準拠）
    
    内積ベースの判定：|cos(θ)| > 0.999 で共線とみなす
    これにより、完全に共線でなくても、分母が小さくなって発散する前に
    早期にゼロ行列を返し、数値的安定性を確保する
    
    Args:
        p_i, p_j, p_k: 各エージェントの位置 (2D or 3D)
    
    Returns:
        bool: 共線ならTrue
    """
    p_i = np.array(p_i, dtype=float).flatten()
    p_j = np.array(p_j, dtype=float).flatten()
    p_k = np.array(p_k, dtype=float).flatten()
    
    # ベクトル v_ij = p_j - p_i, v_ik = p_k - p_i
    v_ij = p_j - p_i
    v_ik = p_k - p_i
    
    norm_ij = np.linalg.norm(v_ij)
    norm_ik = np.linalg.norm(v_ik)
    
    # 位置の重複判定（MATLABのEPS = 0.1に準拠）
    if norm_ij < 0.1 or norm_ik < 0.1:
        return True
    
    # 内積ベースの共線判定（MATLABのcalculate_weight_matrices.m:48-52に準拠）
    # cos(θ) = (v_ij · v_ik) / (||v_ij|| * ||v_ik||)
    cos_theta = np.dot(v_ij, v_ik) / (norm_ij * norm_ik)
    
    # |cos(θ)| > 0.999 の場合、共線とみなす
    # cos(θ) ≈ 1: 同じ方向（0度）
    # cos(θ) ≈ -1: 反対方向（180度）
    return abs(cos_theta) > COLLINEARITY_THRESHOLD


def calculate_weight_matrices_for_agent(p_i, p_j, p_k, occlusion_j=False, occlusion_k=False):
    """
    エージェントiの2つの隣接エージェントj, kに対する重み行列を計算
    
    MATLABのcalculate_weight_matrices.mに準拠
    オクルージョン時：bearing計測不可 → H = 0
    
    Args:
        p_i: エージェントiの位置
        p_j: 隣接エージェントjの位置
        p_k: 隣接エージェントkの位置
        occlusion_j: エージェントjのオクルージョンフラグ（デフォルト: False）
        occlusion_k: エージェントkのオクルージョンフラグ（デフォルト: False）
    
    Returns:
        tuple: (Hij, Hik, is_collinear)
            - Hij: エージェントjに対する重み行列
            - Hik: エージェントkに対する重み行列
            - is_collinear: 共線状態かどうか
    """
    p_i = np.array(p_i, dtype=float).flatten()
    p_j = np.array(p_j, dtype=float).flatten()
    p_k = np.array(p_k, dtype=float).flatten()
    
    dim = len(p_i)
    
    # オクルージョンチェック（MATLABのcalculate_weight_matrices.m:23-52に準拠）
    # オクルージョン時：bearing計測不可 → H = 0
    if occlusion_j and occlusion_k:
        # 両方オクルージョン → 両方ゼロ
        logger.warning(f"オクルージョン検出（両方）: j={occlusion_j}, k={occlusion_k}")
        return np.zeros((dim, dim)), np.zeros((dim, dim)), True
    
    # 共線チェック
    is_collinear = check_collinearity(p_i, p_j, p_k)
    
    if is_collinear:
        # 共線状態の場合は制御上の安全のためゼロ行列を返す
        # 実際の制御ではψ項（共線回避項）が作用する
        logger.warning(f"共線状態検出: i={p_i}, j={p_j}, k={p_k}")
        return np.zeros((dim, dim)), np.zeros((dim, dim)), True
    
    # 各隣接エージェントに対する重み行列を計算
    # オクルージョン時は該当エージェントの重み行列をゼロにする
    if occlusion_j:
        Hij = np.zeros((dim, dim))
        logger.warning(f"オクルージョン検出（j）")
    else:
        Hij = calculate_weight_matrix_bearing(p_i, p_j)
    
    if occlusion_k:
        Hik = np.zeros((dim, dim))
        logger.warning(f"オクルージョン検出（k）")
    else:
        Hik = calculate_weight_matrix_bearing(p_i, p_k)
    
    # オクルージョン時も共線状態として扱う（ψ/τ計算のため）
    is_unlocalizable = is_collinear or occlusion_j or occlusion_k
    
    return Hij, Hik, is_unlocalizable


def calculate_xi_correction(p_i_hat, p_j_hat, p_k_hat, p_i_star, p_j_star, p_k_star, 
                            p_i_actual, p_j_actual, p_k_actual):
    """
    幾何学的補正項ξiを計算（論文Eq. (20)に準拠）
    
    ξijk = H^T_ii * Hij * (p̂_j - p̂_i) + H^T_ii * Hik * (p̂_k - p̂_i)
    
    Args:
        p_i_hat, p_j_hat, p_k_hat: 推定位置
        p_i_star, p_j_star, p_k_star: 目標位置
        p_i_actual, p_j_actual, p_k_actual: 実位置（観測値、MOCAP測定）
    
    Returns:
        xi: 補正項 (dimx1 vector)
    """
    # 推定位置（ξの計算に使用）
    p_i_hat = np.array(p_i_hat, dtype=float).flatten()
    p_j_hat = np.array(p_j_hat, dtype=float).flatten()
    p_k_hat = np.array(p_k_hat, dtype=float).flatten()
    p_i_star = np.array(p_i_star, dtype=float).flatten()
    p_j_star = np.array(p_j_star, dtype=float).flatten()
    p_k_star = np.array(p_k_star, dtype=float).flatten()
    
    # 実位置（観測値、重み行列の計算に使用）
    p_i_actual = np.array(p_i_actual, dtype=float).flatten()
    p_j_actual = np.array(p_j_actual, dtype=float).flatten()
    p_k_actual = np.array(p_k_actual, dtype=float).flatten()
    
    dim = len(p_i_hat)
    
    # ===重要：重み行列は観測値（真の位置、センサ情報）から計算===
    # 論文Remark 5: "The weight matrices can be obtained by bearing measurements"
    # MATLABコード: calculate_weight_matrices(p_true_i, p_true_j, p_true_k, ...)
    # シミュレーション：真の位置 = センサ観測に相当
    # 実機：MOCAP測定値 = センサ観測に相当
    # オクルージョンフラグはここでは使わない（呼び出し側で処理）
    Hij, Hik, is_collinear = calculate_weight_matrices_for_agent(p_i_actual, p_j_actual, p_k_actual, False, False)
    
    if is_collinear:
        # 共線状態ではξはゼロ（ψ項が作用）
        return np.zeros(dim)
    
    # ===論文Eq. (20)の実装===
    # ξijk = H^T_ii * Hij * (p̂_j - p̂_i) + H^T_ii * Hik * (p̂_k - p̂_i)
    # ここで H_ii = H_ij + H_ik
    
    Hii = Hij + Hik
    
    # 推定位置の相対ベクトル
    p_rel_ij_hat = p_j_hat - p_i_hat
    p_rel_ik_hat = p_k_hat - p_i_hat
    
    # 論文Eq. (20)に従った補正項の計算
    # ξijk = H^T_ii * Hij * (p̂_j - p̂_i) + H^T_ii * Hik * (p̂_k - p̂_i)
    xi = Hii.T @ Hij @ p_rel_ij_hat + Hii.T @ Hik @ p_rel_ik_hat
    
    return xi


def calculate_psi_collinearity_avoidance(p_i, p_j, p_k, lambda_vec, tau_threshold=0.1):
    """
    共線回避項ψiを計算
    
    Args:
        p_i, p_j, p_k: エージェント位置
        lambda_vec: 調整ベクトル (dim x 1)
        tau_threshold: 共線判定閾値
    
    Returns:
        psi: 共線回避項 (dim x 1)
    """
    p_i = np.array(p_i, dtype=float).flatten()
    p_j = np.array(p_j, dtype=float).flatten()
    p_k = np.array(p_k, dtype=float).flatten()
    lambda_vec = np.array(lambda_vec, dtype=float).flatten()
    
    dim = len(p_i)
    
    # 共線度の計算
    v_ij = p_j - p_i
    v_ik = p_k - p_i
    norm_ij = np.linalg.norm(v_ij)
    norm_ik = np.linalg.norm(v_ik)
    
    if norm_ij < 1e-6 or norm_ik < 1e-6:
        return np.zeros(dim)
    
    # 外積のノルムで共線度を評価
    if dim == 2:
        cross_norm = abs(v_ij[0] * v_ik[1] - v_ij[1] * v_ik[0])
    else:
        cross = np.cross(v_ij, v_ik)
        cross_norm = np.linalg.norm(cross)
    
    sin_angle = cross_norm / (norm_ij * norm_ik)
    
    # τ = 1 - sin(角度)
    tau = 1.0 - sin_angle
    
    if tau < tau_threshold:
        # 共線でない場合はψ = 0
        return np.zeros(dim)
    
    # 共線の場合、tanh関数を使った滑らかな回避項
    # ψ = -λ * tanh(τ)
    psi = -lambda_vec * np.tanh(tau)
    
    return psi


if __name__ == "__main__":
    # テストコード
    print("=== 重み行列計算テスト ===")
    
    # 2Dテスト
    p_i = np.array([0.0, 0.0])
    p_j = np.array([1.0, 0.0])
    p_k = np.array([0.0, 1.0])
    
    Hij, Hik, is_collinear = calculate_weight_matrices_for_agent(p_i, p_j, p_k)
    print(f"\n2D非共線状態:")
    print(f"Hij =\n{Hij}")
    print(f"Hik =\n{Hik}")
    print(f"共線: {is_collinear}")
    
    # 共線テスト
    p_i = np.array([0.0, 0.0])
    p_j = np.array([1.0, 0.0])
    p_k = np.array([2.0, 0.0])
    
    Hij, Hik, is_collinear = calculate_weight_matrices_for_agent(p_i, p_j, p_k)
    print(f"\n2D共線状態:")
    print(f"共線: {is_collinear}")
    
    # ξ計算テスト
    p_i_hat = np.array([0.1, 0.1])
    p_j_hat = np.array([1.0, 0.0])
    p_k_hat = np.array([0.0, 1.0])
    p_i_star = np.array([0.0, 0.0])
    p_j_star = np.array([1.0, 0.0])
    p_k_star = np.array([0.0, 1.0])
    # 実位置（観測値）
    p_i_actual = np.array([0.05, 0.05])
    p_j_actual = np.array([0.95, 0.0])
    p_k_actual = np.array([0.0, 0.95])
    
    xi = calculate_xi_correction(p_i_hat, p_j_hat, p_k_hat, 
                                   p_i_star, p_j_star, p_k_star,
                                   p_i_actual, p_j_actual, p_k_actual)
    print(f"\nξ補正項: {xi}")
    
    # ψ計算テスト
    psi = calculate_psi_collinearity_avoidance(p_i, p_j, p_k, 
                                                lambda_vec=np.array([0.1, 0.1]))
    print(f"ψ共線回避項: {psi}")
