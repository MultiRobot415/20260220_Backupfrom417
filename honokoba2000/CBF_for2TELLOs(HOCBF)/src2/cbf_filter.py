#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CBF filter module

Applies an obstacle-avoidance Control Barrier Function (CBF) as a minimal
modification to nominal inputs for the horizontal axes (proj X, proj Z),
using the test-coordinate formulation:
  - x_test corresponds to proj Z
  - y_test corresponds to proj X

The QP solved (analytically when possible):
  minimize 0.5 * ||u - u_nom||^2
  subject to a^T u + b >= 0  (CBF half-space)
             u_min <= u <= u_max (box bounds)

Where u = [u_x, u_y] in test coordinates.

We avoid external solvers: project onto the feasible set with simple steps.
If infeasible or numerically problematic, we fall back to clipping.

Author: Cascade
Created: 2025-08-26
"""
from typing import Tuple, Dict
import math

import numpy as np


class CBFParams:
    def __init__(self, K1=0.009, K2=0.009, alpha1=1.0, alpha2=1.0, alpha3=1.0, 
                 alpha4=1.0, alpha5=1.0, Delta=0.4, v_ref=0.01, p_ref=1.0,
                 u_min=-30.0, u_max=30.0, enable_velocity_constraints=False):
        self.K1 = K1
        self.K2 = K2
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.alpha3 = alpha3
        self.alpha4 = alpha4  # 速度差制約のゲイン（X方向）
        self.alpha5 = alpha5  # 速度差制約のゲイン（Y方向）
        self.Delta = Delta
        self.v_ref = v_ref    # 参照速度差（上限）
        self.p_ref = p_ref
        self.u_min = u_min
        self.u_max = u_max
        self.enable_velocity_constraints = enable_velocity_constraints  # 速度差制約のON/OFF


def cbf_halfspace_coeffs(x: float, y: float, x_dot: float, y_dot: float,
                         x_o: float, y_o: float, p: CBFParams) -> Tuple[np.ndarray, float]:
    """
    Compute half-space coefficients a, b for constraint a^T u + b >= 0
    using the provided CBF inequality in test coordinates.
    a = [2*K1*(x-x_o), 2*K2*(y-y_o)]
    b = 2 x_dot^2 + 2(α2+α3)(x−x_o)x_dot + 2 y_dot^2 + 2(α2+α3)(y−y_o)y_dot
        + α1α2[(x−x_o)^2 + (y−y_o)^2 − Δ^2]
    """
    dx = x - x_o
    dy = y - y_o
    a = np.array([2.0 * p.K1 * dx, 2.0 * p.K2 * dy], dtype=float)
    b = (
        2.0 * (x_dot ** 2) + 2.0 * (p.alpha2 + p.alpha3) * dx * x_dot
        + 2.0 * (y_dot ** 2) + 2.0 * (p.alpha2 + p.alpha3) * dy * y_dot
        + p.alpha1 * p.alpha2 * (dx * dx + dy * dy - p.Delta * p.Delta)
    )
    return a, b


def velocity_diff_constraint_x(x_dot_i: float, x_dot_j: float, p: CBFParams) -> Tuple[np.ndarray, float]:
    """
    速度差制約（X方向）の係数計算
    制約式: 2(ẋi - ẋj)ux + α4(ẋi - ẋj)² - α4v_ref² ≥ 0
    形式: a^T u + b ≥ 0, where u = [ux, uy]
    a = [2(ẋi - ẋj), 0]
    b = α4(ẋi - ẋj)² - α4v_ref²
    """
    v_diff = x_dot_i - x_dot_j
    a = np.array([2.0 * v_diff, 0.0], dtype=float)
    b = p.alpha4 * (v_diff ** 2) - p.alpha4 * (p.v_ref ** 2)
    return a, b


def velocity_diff_constraint_y(y_dot_i: float, y_dot_j: float, p: CBFParams) -> Tuple[np.ndarray, float]:
    """
    速度差制約（Y方向）の係数計算
    制約式: 2(ẏi - ẏj)uy + α5(ẏi - ẏj)² - α5v_ref² ≥ 0
    形式: a^T u + b ≥ 0, where u = [ux, uy]
    a = [0, 2(ẏi - ẏj)]
    b = α5(ẏi - ẏj)² - α5v_ref²
    """
    v_diff = y_dot_i - y_dot_j
    a = np.array([0.0, 2.0 * v_diff], dtype=float)
    b = p.alpha5 * (v_diff ** 2) - p.alpha5 * (p.v_ref ** 2)
    return a, b


def pair_distance_constraint(x_i: float, y_i: float, x_dot_i: float, y_dot_i: float,
                             x_j: float, y_j: float, p: CBFParams) -> Tuple[np.ndarray, float]:
    dx = x_i - x_j
    dy = y_i - y_j
    a = np.array([-2.0 * p.K1 * dx, -2.0 * p.K2 * dy], dtype=float)
    b = (
        -2.0 * (x_dot_i ** 2)
        - 2.0 * (p.alpha2 + p.alpha3) * dx * x_dot_i
        - 2.0 * (y_dot_i ** 2)
        - 2.0 * (p.alpha2 + p.alpha3) * dy * y_dot_i
        - p.alpha1 * p.alpha2 * (dx * dx + dy * dy)
        + p.alpha1 * p.alpha2 * (p.p_ref ** 2)
    )
    return a, b


def project_onto_box(u: np.ndarray, u_min: float, u_max: float) -> np.ndarray:
    return np.clip(u, u_min, u_max)


def project_onto_halfspace(u_nom: np.ndarray, a: np.ndarray, b: float) -> np.ndarray:
    """
    Euclidean projection of u_nom onto the half-space {u | a^T u + b >= 0}.
    If already feasible, returns u_nom.
    """
    aTa = float(a @ a)
    if aTa <= 1e-12:
        return u_nom.copy()
    margin = float(a @ u_nom + b)
    if margin >= 0.0:
        return u_nom.copy()
    # Move along a to satisfy a^T u + b = 0
    return u_nom - (margin / aTa) * a


def enforce_cbf(u_nom: Tuple[float, float],
                state_test: Tuple[float, float, float, float],
                obstacle_test: Tuple[float, float],
                params: CBFParams = None,
                other_velocity: Tuple[float, float] = None) -> Tuple[np.ndarray, Dict]:
    """
    Apply the CBF inequality to adjust u_nom minimally.
    Supports 3 constraints: obstacle avoidance (h1), X velocity diff (h2), Y velocity diff (h3).

    Inputs:
      - u_nom: (u_x, u_y) in test coordinates
      - state_test: (x, y, x_dot, y_dot) in test coordinates
      - obstacle_test: (x_o, y_o) in test coordinates
      - params: CBFParams. If None, defaults are used.
      - other_velocity: (x_dot_j, y_dot_j) other drone's velocity in test coords. 
                        Required if enable_velocity_constraints=True.

    Returns: (u_safe (2,), info dict)
      info: {
        'feasible_nom': bool,
        'projected': bool,
        'fell_back': bool,
        'h1_value': float,  # 障害物回避制約の値
        'h1_satisfied': bool,
        'h2_value': float,  # X方向速度差制約の値
        'h2_satisfied': bool,
        'h3_value': float,  # Y方向速度差制約の値
        'h3_satisfied': bool,
        'velocity_diff_x': float,
        'velocity_diff_y': float,
      }
    """
    if params is None:
        params = CBFParams()

    u_nom = np.array(u_nom, dtype=float)
    x, y, x_dot, y_dot = state_test
    x_o, y_o = obstacle_test

    # h1: 障害物回避制約
    a1, b1 = cbf_halfspace_coeffs(x, y, x_dot, y_dot, x_o, y_o, params)
    
    # h2: 機体間距離制約（有効な場合のみ）
    use_velocity_constraints = params.enable_velocity_constraints and other_velocity is not None
    
    if use_velocity_constraints:
        x_j, y_j, x_dot_j, y_dot_j = other_velocity
        a2, b2 = pair_distance_constraint(x, y, x_dot, y_dot, x_j, y_j, params)
        a3, b3 = None, None
        velocity_diff_x = 0.0
        velocity_diff_y = 0.0
    else:
        a2, b2 = None, None
        a3, b3 = None, None
        velocity_diff_x = 0.0
        velocity_diff_y = 0.0
    
    # Start with bound-clipped nominal
    u = project_onto_box(u_nom, params.u_min, params.u_max)

    # 制約のチェック
    h1_value = float(a1 @ u + b1)
    h2_value = float(a2 @ u + b2) if use_velocity_constraints and a2 is not None else 0.0
    h3_value = float(a3 @ u + b3) if use_velocity_constraints and a3 is not None else 0.0
    
    info = {
        'feasible_nom': False,
        'projected': False,
        'fell_back': False,
        'h1_value': h1_value,
        'h1_satisfied': h1_value >= 0.0,
        'h2_value': h2_value,
        'h2_satisfied': h2_value >= 0.0 if use_velocity_constraints else True,
        'h3_value': h3_value,
        'h3_satisfied': h3_value >= 0.0 if use_velocity_constraints else True,
        'velocity_diff_x': velocity_diff_x,
        'velocity_diff_y': velocity_diff_y,
    }

    # すべての制約が満たされているかチェック
    all_satisfied = info['h1_satisfied'] and info['h2_satisfied'] and info['h3_satisfied']
    
    if all_satisfied:
        info['feasible_nom'] = True
        print(f"🜢 CBF: すべての制約満足 h1={h1_value:.3f}, h2={h2_value:.3f}, h3={h3_value:.3f}")
        return u, info
    else:
        print(f"🔴 CBF: 制約違反 h1={h1_value:.3f}(ok={info['h1_satisfied']}), h2={h2_value:.3f}(ok={info['h2_satisfied']}), h3={h3_value:.3f}(ok={info['h3_satisfied']})")

    # 3制約対応の逐次射影法
    u = project_onto_halfspace(u, a1, b1)
    if use_velocity_constraints:
        if a2 is not None:
            u = project_onto_halfspace(u, a2, b2)
        if a3 is not None:
            u = project_onto_halfspace(u, a3, b3)
    u = project_onto_box(u, params.u_min, params.u_max)

    # 再度制約のチェック
    h1_value = float(a1 @ u + b1)
    h2_value = float(a2 @ u + b2) if use_velocity_constraints and a2 is not None else 0.0
    h3_value = float(a3 @ u + b3) if use_velocity_constraints and a3 is not None else 0.0

    info['h1_value'] = h1_value
    info['h1_satisfied'] = h1_value >= 0.0
    info['h2_value'] = h2_value
    info['h2_satisfied'] = h2_value >= 0.0 if use_velocity_constraints else True
    info['h3_value'] = h3_value
    info['h3_satisfied'] = h3_value >= 0.0 if use_velocity_constraints else True
    
    # 最終チェック
    all_satisfied_after = info['h1_satisfied'] and info['h2_satisfied'] and info['h3_satisfied']
    
    if all_satisfied_after:
        info['projected'] = True
        print(f"🟡 CBF: 逐次射影成功 h1={h1_value:.3f}, h2={h2_value:.3f}, h3={h3_value:.3f}")
        return u, info
    else:
        # 完全には満たせなかった場合（保守的）
        info['fell_back'] = True
        print(f"🟠 CBF: 逐次射影後も制約違反 h1={h1_value:.3f}(ok={info['h1_satisfied']}), h2={h2_value:.3f}(ok={info['h2_satisfied']}), h3={h3_value:.3f}(ok={info['h3_satisfied']})")
        return u, info
