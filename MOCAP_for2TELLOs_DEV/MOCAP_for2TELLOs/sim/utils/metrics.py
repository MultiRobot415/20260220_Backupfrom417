#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
評価指標計算ユーティリティ
オブザーバー性能と故障検出の評価指標を計算する関数を提供します。
"""

import numpy as np
from typing import List, Dict, Any, Tuple


def calculate_rmse(true_values: List[np.ndarray], estimated_values: List[np.ndarray]) -> float:
    """
    二乗平均平方根誤差（RMSE）の計算
    
    Args:
        true_values: 真値のリスト
        estimated_values: 推定値のリスト
        
    Returns:
        RMSE値
    """
    errors = [np.linalg.norm(true - est) for true, est in zip(true_values, estimated_values)]
    return np.sqrt(np.mean(np.square(errors)))


def calculate_mae(true_values: List[np.ndarray], estimated_values: List[np.ndarray]) -> float:
    """
    平均絶対誤差（MAE）の計算
    
    Args:
        true_values: 真値のリスト
        estimated_values: 推定値のリスト
        
    Returns:
        MAE値
    """
    errors = [np.linalg.norm(true - est) for true, est in zip(true_values, estimated_values)]
    return np.mean(errors)


def calculate_fault_detection_metrics(
    actual_faults: List[bool],
    detected_faults: List[bool]
) -> Dict[str, float]:
    """
    故障検出の性能指標を計算
    
    Args:
        actual_faults: 実際の故障フラグのリスト
        detected_faults: 検出された故障フラグのリスト
        
    Returns:
        性能指標を含む辞書
    """
    # 混同行列の要素を計算
    true_positives = sum(1 for a, d in zip(actual_faults, detected_faults) if a and d)
    false_positives = sum(1 for a, d in zip(actual_faults, detected_faults) if not a and d)
    true_negatives = sum(1 for a, d in zip(actual_faults, detected_faults) if not a and not d)
    false_negatives = sum(1 for a, d in zip(actual_faults, detected_faults) if a and not d)
    
    # 各指標の計算
    accuracy = (true_positives + true_negatives) / len(actual_faults) if actual_faults else 0
    
    precision = true_positives / (true_positives + false_positives) if (true_positives + false_positives) > 0 else 0
    
    recall = true_positives / (true_positives + false_negatives) if (true_positives + false_negatives) > 0 else 0
    
    f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
    
    # 検出遅延の計算（最初の故障から検出までの時間）
    detection_delay = None
    for i, (a, d) in enumerate(zip(actual_faults, detected_faults)):
        if a and not detection_delay:  # 最初の故障発生
            for j in range(i, len(detected_faults)):
                if detected_faults[j]:  # 故障検出
                    detection_delay = j - i
                    break
    
    return {
        'accuracy': accuracy,
        'precision': precision,
        'recall': recall,
        'f1_score': f1_score,
        'detection_delay': detection_delay
    }


def evaluate_observer_performance(
    true_positions: List[np.ndarray],
    estimated_positions: List[np.ndarray],
    actual_fault_time: int = None,
    detected_fault_times: List[int] = None
) -> Dict[str, Any]:
    """
    オブザーバーの性能を総合的に評価
    
    Args:
        true_positions: 真の位置のリスト
        estimated_positions: 推定位置のリスト
        actual_fault_time: 実際の故障が発生した時間インデックス
        detected_fault_times: 検出された故障の時間インデックスのリスト
        
    Returns:
        評価結果を含む辞書
    """
    # 基本的な精度指標
    rmse = calculate_rmse(true_positions, estimated_positions)
    mae = calculate_mae(true_positions, estimated_positions)
    
    results = {
        'rmse': rmse,
        'mae': mae
    }
    
    # 故障検出の評価（故障情報がある場合）
    if actual_fault_time is not None and detected_fault_times is not None:
        actual_faults = [i >= actual_fault_time for i in range(len(true_positions))]
        detected_faults = [i in detected_fault_times for i in range(len(true_positions))]
        
        fault_metrics = calculate_fault_detection_metrics(actual_faults, detected_faults)
        results.update(fault_metrics)
    
    return results


def calculate_trust_correlation(
    trust_values: List[float],
    error_norms: List[float]
) -> float:
    """
    信頼度と実際の誤差の相関関係を計算
    
    Args:
        trust_values: 信頼度値のリスト
        error_norms: 誤差ノルムのリスト
        
    Returns:
        相関係数
    """
    # 逆相関が期待される（誤差大→信頼度小）ため、負の相関が望ましい
    if len(trust_values) != len(error_norms):
        return 0.0
    
    return np.corrcoef(trust_values, error_norms)[0, 1]


def analyze_simulation_results(
    simulation_data: Dict[str, Any],
    observer_data: Dict[str, Any]
) -> Dict[str, Any]:
    """
    シミュレーション結果の総合分析
    
    Args:
        simulation_data: シミュレーションデータ
        observer_data: オブザーバーデータ
        
    Returns:
        分析結果を含む辞書
    """
    results = {}
    
    # 各ドローンの評価
    for drone_id in simulation_data.get('drone_ids', []):
        # 真の位置とオブザーバー推定位置を取得
        true_positions = simulation_data.get('positions', {}).get(drone_id, [])
        estimated_positions = observer_data.get('positions', {}).get(drone_id, [])
        
        if true_positions and estimated_positions:
            # 故障情報の取得
            actual_fault_time = simulation_data.get('fault_times', {}).get(drone_id)
            detected_fault_times = observer_data.get('fault_times', {}).get(drone_id, [])
            
            # 性能評価
            drone_results = evaluate_observer_performance(
                true_positions,
                estimated_positions,
                actual_fault_time,
                detected_fault_times
            )
            
            # 信頼度と誤差の相関
            trust_values = observer_data.get('trust_values', {}).get(drone_id, [])
            error_norms = [np.linalg.norm(t - e) for t, e in zip(true_positions, estimated_positions)]
            
            if trust_values and error_norms:
                correlation = calculate_trust_correlation(trust_values, error_norms)
                drone_results['trust_error_correlation'] = correlation
            
            results[f'drone_{drone_id}'] = drone_results
    
    return results


def print_evaluation_summary(results: Dict[str, Any]):
    """
    評価結果のサマリーを出力
    
    Args:
        results: 評価結果辞書
    """
    print("\n===== Observer Performance Evaluation =====")
    
    for drone_key, metrics in results.items():
        print(f"\n--- {drone_key} ---")
        print(f"Position Estimation RMSE: {metrics.get('rmse', 'N/A'):.4f} m")
        print(f"Position Estimation MAE: {metrics.get('mae', 'N/A'):.4f} m")
        
        if 'accuracy' in metrics:
            print("\nFault Detection:")
            print(f"Accuracy: {metrics.get('accuracy', 'N/A'):.4f}")
            print(f"Precision: {metrics.get('precision', 'N/A'):.4f}")
            print(f"Recall: {metrics.get('recall', 'N/A'):.4f}")
            print(f"F1 Score: {metrics.get('f1_score', 'N/A'):.4f}")
            
            delay = metrics.get('detection_delay', 'N/A')
            if delay is not None:
                print(f"Detection Delay: {delay} steps")
            else:
                print("Detection Delay: N/A")
        
        if 'trust_error_correlation' in metrics:
            corr = metrics.get('trust_error_correlation')
            print(f"\nTrust-Error Correlation: {corr:.4f}")
            if corr < -0.7:
                print("Trust metric strongly correlates with errors (good)")
            elif corr < -0.3:
                print("Trust metric moderately correlates with errors")
            else:
                print("Trust metric weakly correlates with errors (needs improvement)")
    
    print("\n===========================================")
