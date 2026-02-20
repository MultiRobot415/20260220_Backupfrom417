#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位置制御モジュール

モーションキャプチャシステムから得た位置データを使用して、
Telloドローンの位置を制御するためのアルゴリズムを提供するモジュール。

作成日: 2025-06-26
"""

import time
import math
import numpy as np
import logging

# ログ設定
log_format = '%(asctime)s - %(levelname)s - [CONTROL] %(message)s'
if not logging.getLogger().handlers:
    logging.basicConfig(level=logging.DEBUG, format=log_format)

class PositionController:
    """
    位置制御クラス
    P制御（比例制御）アルゴリズムを使用して、
    目標位置と現在位置の差に基づいて制御値を計算する。
    """
    
    def __init__(self):
        """
        位置制御クラスの初期化
        デフォルトのゲイン値と制限値を設定
        """
        # P制御のゲイン係数（x, y, z, yaw）
        self.gain_x = 0.4  # 前後方向のゲイン
        self.gain_y = 0.6  # 上下方向のゲイン
        self.gain_z = 0.4  # 左右方向のゲイン
        self.gain_yaw = 0.3  # 旋回方向のゲイン
        
        # 速度の制限値
        self.max_speed_x = 50  # 前後方向の最大速度
        self.max_speed_y = 50  # 上下方向の最大速度
        self.max_speed_z = 50  # 左右方向の最大速度
        self.max_speed_yaw = 50  # 旋回方向の最大速度
        
        # 不感帯（この範囲内の誤差は無視）
        self.deadband_x = 0.05  # 前後方向の不感帯 (m)
        self.deadband_y = 0.05  # 上下方向の不感帯 (m)
        self.deadband_z = 0.05  # 左右方向の不感帯 (m)
        self.deadband_yaw = 5.0  # 旋回方向の不感帯 (度)
        
        # 目標位置と姿勢（初期値）
        self.target_position = [0.0, 1.0, 0.0]  # x, y, z (m)
        self.target_yaw = 0.0  # yaw (度)

    def set_gains(self, gain_x=None, gain_y=None, gain_z=None, gain_yaw=None):
        """
        制御ゲインを設定する
        
        Args:
            gain_x: 前後方向のゲイン
            gain_y: 上下方向のゲイン
            gain_z: 左右方向のゲイン
            gain_yaw: 旋回方向のゲイン
        """
        if gain_x is not None:
            self.gain_x = gain_x
        if gain_y is not None:
            self.gain_y = gain_y
        if gain_z is not None:
            self.gain_z = gain_z
        if gain_yaw is not None:
            self.gain_yaw = gain_yaw
    
    def set_max_speeds(self, max_x=None, max_y=None, max_z=None, max_yaw=None):
        """
        最大速度を設定する
        
        Args:
            max_x: 前後方向の最大速度
            max_y: 上下方向の最大速度
            max_z: 左右方向の最大速度
            max_yaw: 旋回方向の最大速度
        """
        if max_x is not None:
            self.max_speed_x = max_x
        if max_y is not None:
            self.max_speed_y = max_y
        if max_z is not None:
            self.max_speed_z = max_z
        if max_yaw is not None:
            self.max_speed_yaw = max_yaw
    
    def set_deadbands(self, db_x=None, db_y=None, db_z=None, db_yaw=None):
        """
        不感帯を設定する
        
        Args:
            db_x: 前後方向の不感帯 (m)
            db_y: 上下方向の不感帯 (m)
            db_z: 左右方向の不感帯 (m)
            db_yaw: 旋回方向の不感帯 (度)
        """
        if db_x is not None:
            self.deadband_x = db_x
        if db_y is not None:
            self.deadband_y = db_y
        if db_z is not None:
            self.deadband_z = db_z
        if db_yaw is not None:
            self.deadband_yaw = db_yaw
    
    def set_target_position(self, x=None, y=None, z=None):
        """
        目標位置を設定する
        
        Args:
            x: 前後方向の目標位置 (m)
            y: 上下方向の目標位置 (m)
            z: 左右方向の目標位置 (m)
        """
        if x is not None:
            self.target_position[0] = x
        if y is not None:
            self.target_position[1] = y
        if z is not None:
            self.target_position[2] = z
    
    def set_target_yaw(self, yaw):
        """
        目標ヨー角を設定する
        
        Args:
            yaw: ヨー角 (度)
        """
        self.target_yaw = yaw
    
    def get_target_position(self):
        """
        現在の目標位置を取得する
        
        Returns:
            list: [x, y, z] 形式の目標位置
        """
        return self.target_position.copy()
    
    def get_target_yaw(self):
        """
        現在の目標ヨー角を取得する
        
        Returns:
            float: ヨー角 (度)
        """
        return self.target_yaw
    
    @staticmethod
    def quaternion_to_yaw(quaternion):
        """
        クォータニオンからヨー角（度数法）を抽出する
        
        Args:
            quaternion: {"x": x, "y": y, "z": z, "w": w} 形式のクォータニオン辞書
        
        Returns:
            float: ヨー角（度数法、-180〜180度）
        """
        try:
            # クォータニオンの各成分を取得
            x = quaternion.get("x", 0)
            y = quaternion.get("y", 0)
            z = quaternion.get("z", 0)
            w = quaternion.get("w", 1)  # wのデフォルト値は1（単位クォータニオン）
            
            # ヨー角（y軸周り）の計算
            # 数式: atan2(2*(w*y + x*z), 1 - 2*(y*y + x*x))
            siny_cosp = 2.0 * (w * y + x * z)
            cosy_cosp = 1.0 - 2.0 * (y * y + x * x)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)
            
            # ラジアンから度数法に変換し、-180〜180度の範囲に正規化
            yaw_deg = math.degrees(yaw_rad)
            
            return yaw_deg
        except Exception as e:
            logging.warning(f"ヨー角の計算中にエラーが発生しました: {e}")
            return 0.0
    
    def calculate_control(self, current_position, current_yaw=None, quaternion=None):
        """
        現在位置と目標位置の差に基づいて制御値を計算する
        
        Args:
            current_position: 現在位置 [x, y, z]
            current_yaw: 現在のヨー角 (度) (省略可)
            quaternion: MOCAPから取得したクォータニオン (省略可)
        
        Returns:
            list: [lr, fb, ud, yaw] 形式の制御値
                lr: 左右方向の速度 (-100〜100)
                fb: 前後方向の速度 (-100〜100)
                ud: 上下方向の速度 (-100〜100)
                yaw: 旋回方向の速度 (-100〜100)
        """
        # 位置の差分を計算
        error_x = self.target_position[0] - current_position[0]  # 前後方向の誤差
        error_y = self.target_position[1] - current_position[1]  # 上下方向の誤差
        error_z = self.target_position[2] - current_position[2]  # 左右方向の誤差
        
        # クォータニオンからヨー角を計算（もし提供されていれば）
        if quaternion is not None and current_yaw is None:
            current_yaw = self.quaternion_to_yaw(quaternion)
        
        # ヨー角の差分を計算（ヨー角が与えられた場合のみ）
        error_yaw = 0
        if current_yaw is not None:
            # ヨー角の安定化: 非常に小さい値（±5度以内）は0とみなす
            # これにより±0度境界付近での符号反転による不安定性を防止
            YAW_STABILITY_THRESHOLD = 5.0  # 度
            original_yaw = current_yaw  # 元の値を保存（ログ用）
            
            if abs(current_yaw) <= YAW_STABILITY_THRESHOLD:
                logging.debug(f"ヨー角安定化: {current_yaw}° → 0°に補正")
                current_yaw = 0.0
            
            error_yaw = self._calculate_yaw_error(self.target_yaw, current_yaw)
            
            # ヨー角を考慮した座標変換の改善版
            # ヨー角の絶対値と方向符号を分離して処理することで安定性を向上
            yaw_direction = 1 if abs(current_yaw) <= 90 else -1
            abs_yaw_rad = math.radians(abs(current_yaw))
            
            # 改善された座標変換（方向符号を明示的に扱う）
            error_x_body = error_x * math.cos(abs_yaw_rad)
            if current_yaw != 0:  # 0度の場合はsin項が0になるので計算不要
                error_x_body += yaw_direction * error_z * math.sin(abs_yaw_rad)
                
            error_z_body = error_z * math.cos(abs_yaw_rad)
            if current_yaw != 0:  # 0度の場合はsin項が0になるので計算不要
                error_z_body -= yaw_direction * error_x * math.sin(abs_yaw_rad)
            
            # デバッグ: 変換前後の値を表示
            logging.debug(f"元のヨー角: {original_yaw}°, 補正後: {current_yaw}°")
            logging.debug(f"座標変換前: error_x={error_x:.2f}, error_z={error_z:.2f}")
            logging.debug(f"座標変換後: error_x_body={error_x_body:.2f}, error_z_body={error_z_body:.2f}")
            
            # ドローンが反転している場合の補正（より安定性を考慮）
            # 安全マージンを含めた判定（85度以上で反転とみなす）
            YAW_SAFETY_MARGIN = 5.0  # 度
            if abs(current_yaw) > (90.0 - YAW_SAFETY_MARGIN):
                logging.debug(f"ドローンが反転姿勢: yaw={current_yaw}° -> 制御方向を反転")
                # 反転補正を適用
                error_x_body = -error_x_body
                error_z_body = -error_z_body
                logging.debug(f"反転補正後: error_x_body={error_x_body:.2f}, error_z_body={error_z_body:.2f}")
            
            # 変換後の値を使用
            error_x = error_x_body
            error_z = error_z_body
        
        # 不感帯内の誤差はゼロにする
        if abs(error_x) < self.deadband_x:
            error_x = 0
        if abs(error_y) < self.deadband_y:
            error_y = 0
        if abs(error_z) < self.deadband_z:
            error_z = 0
        if abs(error_yaw) < self.deadband_yaw:
            error_yaw = 0
        
        # デバッグ情報を記録
        logging.debug(f"位置誤差 [x,y,z]=[{error_x:.2f},{error_y:.2f},{error_z:.2f}], ヨー誤差={error_yaw:.1f}°")
        
        # P制御：誤差にゲインを掛けて制御値を計算
        control_x = int(error_x * self.gain_x * 100)  # 前後方向の制御値
        control_y = int(error_y * self.gain_y * 100)  # 上下方向の制御値
        control_z = int(error_z * self.gain_z * 100)  # 左右方向の制御値
        control_yaw = int(error_yaw * self.gain_yaw)  # 旋回方向の制御値
        
        logging.debug(f"制御出力 [lr,fb,ud,yaw]=[{control_z},{control_x},{control_y},{control_yaw}]")
        
        # 制限値を適用
        control_x = self._clamp(control_x, -self.max_speed_x, self.max_speed_x)
        control_y = self._clamp(control_y, -self.max_speed_y, self.max_speed_y)
        control_z = self._clamp(control_z, -self.max_speed_z, self.max_speed_z)
        control_yaw = self._clamp(control_yaw, -self.max_speed_yaw, self.max_speed_yaw)
        
        # Telloの制御形式に合わせて返す
        # Telloの制御値： [left_right, forward_backward, up_down, yaw]
        result = [control_z, control_x, control_y, control_yaw]
        logging.debug(f"Tello制御値 [lr,fb,ud,yaw]={result}")
        return result
    
    def calculate_error_distance(self, current_position):
        """
        現在位置と目標位置の距離を計算する
        
        Args:
            current_position: 現在位置 [x, y, z]
        
        Returns:
            float: 距離 (m)
        """
        dx = self.target_position[0] - current_position[0]
        dy = self.target_position[1] - current_position[1]
        dz = self.target_position[2] - current_position[2]
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def is_at_target(self, current_position, current_yaw=None):
        """
        現在位置が目標位置に十分近いかどうかを判定する
        
        Args:
            current_position: 現在位置 [x, y, z]
            current_yaw: 現在のヨー角 (度) (省略可)
        
        Returns:
            bool: 目標位置に到達しているかどうか
        """
        # 位置の誤差を計算
        error_x = abs(self.target_position[0] - current_position[0])
        error_y = abs(self.target_position[1] - current_position[1])
        error_z = abs(self.target_position[2] - current_position[2])
        
        # 不感帯の2倍を許容範囲とする
        pos_ok = (error_x <= self.deadband_x*2 and
                  error_y <= self.deadband_y*2 and
                  error_z <= self.deadband_z*2)
        
        # ヨー角が指定されていない場合は位置のみで判定
        if current_yaw is None:
            return pos_ok
        
        # ヨー角の誤差を計算
        error_yaw = abs(self._calculate_yaw_error(self.target_yaw, current_yaw))
        yaw_ok = error_yaw <= self.deadband_yaw*2
        
        return pos_ok and yaw_ok
    
    def _calculate_yaw_error(self, target_yaw, current_yaw):
        """
        ヨー角の最短経路の差分を計算する（-180°〜180°の範囲）
        
        Args:
            target_yaw: 目標ヨー角 (度)
            current_yaw: 現在のヨー角 (度)
        
        Returns:
            float: ヨー角の誤差 (-180°〜180°)
        """
        error = target_yaw - current_yaw
        
        # -180°〜180°の範囲に正規化
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        return error
    
    def _clamp(self, value, min_val, max_val):
        """
        値を指定範囲内に制限する
        
        Args:
            value: 対象の値
            min_val: 最小値
            max_val: 最大値
        
        Returns:
            float: 制限された値
        """
        return max(min_val, min(value, max_val))


# テスト用コード
if __name__ == "__main__":
    print("位置制御モジュールのテスト")
    print("-------------------")
    
    # 位置制御クラスを初期化
    controller = PositionController()
    
    # テスト用の目標位置を設定
    controller.set_target_position(1.0, 1.2, 0.5)
    controller.set_target_yaw(90)
    
    # テスト用のいくつかの現在位置でコマンドを計算
    test_positions = [
        [0.0, 0.0, 0.0],  # 原点
        [1.0, 1.2, 0.5],  # 目標位置
        [0.95, 1.15, 0.45],  # 目標位置近く
        [2.0, 2.0, 0.0],  # 目標から離れた位置
    ]
    
    test_yaws = [0, 90, 85, 180]
    
    for i, pos in enumerate(test_positions):
        yaw = test_yaws[i]
        print(f"\nテスト {i+1}:")
        print(f"目標位置: {controller.get_target_position()}, ヨー: {controller.get_target_yaw()}")
        print(f"現在位置: {pos}, ヨー: {yaw}")
        
        # 制御値を計算
        control = controller.calculate_control(pos, yaw)
        print(f"制御値: [LR={control[0]}, FB={control[1]}, UD={control[2]}, YAW={control[3]}]")
        
        # エラー距離を計算
        distance = controller.calculate_error_distance(pos)
        print(f"エラー距離: {distance:.3f} m")
        
        # 目標位置に到達しているかを確認
        is_at_target = controller.is_at_target(pos, yaw)
        print(f"目標位置に到達: {is_at_target}")
    
    print("\nテスト完了")
