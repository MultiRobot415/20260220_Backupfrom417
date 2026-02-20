#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
故障注入・解除モジュール
Telloドローンへの故障注入と解除のためのモジュールです。
"""

import logging

# ロギング設定
log_format = '%(asctime)s - %(levelname)s - [FAULT] %(message)s'
if not logging.getLogger().handlers:
    logging.basicConfig(level=logging.DEBUG, format=log_format)

class FaultHandler:
    """
    故障注入・解除用のハンドラークラス
    """
    def __init__(self):
        """初期化"""
        self.is_fault_active = False
        self.fault_target_drone = 0  # デフォルトは1号機（インデックス0）
        self.fault_type = "hardover"  # デフォルトはピッチhardover
        self.fault_value = 50  # 固定値
        
    def inject_fault(self, target_drone=0, fault_type="hardover", value=50):
        """
        故障を注入する
        
        Args:
            target_drone: 対象ドローンのインデックス
            fault_type: 故障タイプ ("hardover"/"bias"/"loss")
            value: 故障値
        
        Returns:
            bool: 成功したかどうか
        """
        self.is_fault_active = True
        self.fault_target_drone = target_drone
        self.fault_type = fault_type
        self.fault_value = value
        logging.info(f"故障を注入: ドローン{target_drone+1}, タイプ={fault_type}, 値={value}")
        return True
        
    def clear_fault(self):
        """
        故障を解除する
        
        Returns:
            bool: 成功したかどうか
        """
        was_active = self.is_fault_active
        self.is_fault_active = False
        logging.info(f"故障を解除: ドローン{self.fault_target_drone+1}")
        return was_active
        
    def modify_control_values(self, drone_index, control_values):
        """
        制御値に故障を適用する
        
        Args:
            drone_index: ドローンのインデックス
            control_values: 元の制御値 [lr, fb, ud, yv]
            
        Returns:
            list: 修正された制御値
        """
        # 故障が有効でないか対象ドローンでなければ変更なし
        if not self.is_fault_active or drone_index != self.fault_target_drone:
            return control_values
            
        # 元の値をコピー
        modified = control_values.copy()
        
        # 故障タイプに応じて制御値を修正
        if self.fault_type == "hardover":
            # 全方向に0を適用（完全停止故障）
            modified[0] = 0  # 左右方向
            modified[1] = 0  # 前後方向
            modified[2] = 0  # 上下方向
            modified[3] = 0  # ヨー方向
            
        # 他の故障タイプは必要に応じて実装
            
        logging.debug(f"制御値修正: 元={control_values}, 修正後={modified}")
        return modified

# グローバルインスタンス
fault_handler = FaultHandler()

# 公開関数
def inject_fault(target_drone=0, fault_type="hardover", value=50):
    """グローバルハンドラーで故障を注入"""
    return fault_handler.inject_fault(target_drone, fault_type, value)
    
def clear_fault():
    """グローバルハンドラーで故障を解除"""
    return fault_handler.clear_fault()
    
def modify_control_values(drone_index, control_values):
    """グローバルハンドラーで制御値を修正"""
    return fault_handler.modify_control_values(drone_index, control_values)
