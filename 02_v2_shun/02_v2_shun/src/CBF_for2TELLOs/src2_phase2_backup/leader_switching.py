#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
リーダー切り替えロジック
ドローンの信頼度に基づいてリーダーを選定・切り替えるためのモジュール
"""

import numpy as np
from typing import List, Dict, Tuple
import time


class LeaderSelector:
    """
    信頼度メトリクスに基づいてリーダーを選定するクラス
    """
    
    def __init__(self, num_drones: int = 2):
        """
        初期化
        
        Args:
            num_drones: ドローンの数
        """
        self.num_drones = num_drones
        self.leader_idx = 0  # 初期リーダーは1号機（インデックス0）
        
        # Phase 2: シンプルな緊急交代モード
        self.emergency_threshold = 50.0  # 緊急交代の信頼度閾値（ユーザー指定値：50）
        self.fixed_backup_leader = 1    # 固定バックアップリーダー（2号機）
        
        # Phase 2: シンプルな切り替え履歴
        self.switch_history = []
        self.last_switch_time = 0
        self.start_time = time.time()
        
        # Phase 2: シンプルなリーダー切り替え制御
        self.switching_enabled = True
    
    def update(self, trust_metrics: List[float]) -> Dict:
        """
        リーダー選定の更新
        
        Args:
            trust_metrics: 各ドローンの信頼度のリスト
            
        Returns:
            更新結果の辞書
        """
        if not self.switching_enabled:
            return {
                'leader_index': self.leader_idx,
                'leader_changed': False,
                'leader_trust': trust_metrics[self.leader_idx] if trust_metrics else 1.0,
                'switching_enabled': False
            }
        
        previous_leader = self.leader_idx
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # リーダー切り替えロジックの適用
        if len(trust_metrics) >= self.num_drones:
            self._update_leader_selection(trust_metrics, current_time)
        
        # 切り替え状態を返す
        leader_changed = previous_leader != self.leader_idx
        if leader_changed:
            # リーダー切り替えログの記録
            self.switch_history.append({
                'time': elapsed_time,
                'old_leader': previous_leader,
                'new_leader': self.leader_idx,
                'trust_metrics': trust_metrics.copy()
            })
            self.last_switch_time = current_time
        
        return {
            'leader_index': self.leader_idx,
            'leader_changed': leader_changed,
            'leader_trust': trust_metrics[self.leader_idx] if trust_metrics else 1.0,
            'switching_enabled': self.switching_enabled
        }
    
    def _update_leader_selection(self, trust_metrics: List[float], current_time: float):
        """
        Phase 2: シンプルな緊急交代モード
        現在のリーダーの信頼度が0.3以下になったら、2号機に即座切り替え
        
        Args:
            trust_metrics: 各ドローンの信頼度のリスト
            current_time: 現在時刻
        """
        current_leader_trust = trust_metrics[self.leader_idx]
        
        # 緊急交代判定：現リーダーの信頼度が闾値以下の場合
        if current_leader_trust < self.emergency_threshold:
            # 2号機が利用可能かチェック
            if (self.fixed_backup_leader < len(trust_metrics) and 
                self.leader_idx != self.fixed_backup_leader):
                
                backup_trust = trust_metrics[self.fixed_backup_leader]
                print(f"🚨 Phase 2: 緊急リーダー交代! {self.leader_idx+1}号機({current_leader_trust:.3f}) → {self.fixed_backup_leader+1}号機({backup_trust:.3f})")
                
                # リーダーを固定バックアップ（2号機）に切り替え
                self.leader_idx = self.fixed_backup_leader
                return
        
        # 緊急交代条件を満たさない場合は現状維持
        # （シンプル化のため、通常交代ロジックは削除）
    
    # Phase 2: シンプル化のため、最適候補選定メソッドは不要
    # 固定バックアップリーダー（2号機）を使用
    
    def get_leader_index(self) -> int:
        """現在のリーダーインデックスを取得"""
        return self.leader_idx
    
    def enable_switching(self, enabled: bool = True):
        """リーダー切り替え機能の有効/無効を設定"""
        self.switching_enabled = enabled
    
    def is_switching_enabled(self) -> bool:
        """リーダー切り替え機能が有効かどうかを取得"""
        return self.switching_enabled
    
    def get_switch_history(self) -> List[Dict]:
        """リーダー切り替え履歴を取得"""
        return self.switch_history
    
    def reset(self):
        """状態をリセット"""
        self.leader_idx = 0  # 初期リーダーに戻す
        self.switch_history = []
        self.last_switch_time = 0
        self.start_time = time.time()
