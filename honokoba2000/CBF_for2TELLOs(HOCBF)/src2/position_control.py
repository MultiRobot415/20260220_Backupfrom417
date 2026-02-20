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
from cbf_filter import enforce_cbf, CBFParams

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
        # P制御のゲイン係数（x, y, z, yaw）- デフォルト値に戻す
        self.gain_x = 1.0  # 前後方向のゲイン (デフォルト)
        self.gain_y = 1.0  # 上下方向のゲイン (安定性重視: 1.6→1.0)
        self.gain_z = 1.0  # 左右方向のゲイン (デフォルト)
        self.gain_yaw = 0.2  # 旋回方向のゲイン (精度向上: 0.15→0.2)
        
        # 速度の制限値 - CBF制御入力上限と整合性を保つため30に変更
        self.max_speed_x = 30  # 前後方向の最大速度
        self.max_speed_y = 30  # 上下方向の最大速度
        self.max_speed_z = 30  # 左右方向の最大速度
        self.max_speed_yaw = 25  # 旋回方向の最大速度
        
        # 不感帯（この範囲内の誤差は無視）
        self.deadband_x = 0 # 前後方向の不感帯 (m)
        self.deadband_y = 0 # 上下方向の不感帯 (m)
        self.deadband_z = 0 # 左右方向の不感帯 (m)
        self.deadband_yaw = 2.0  # 旋回方向の不感帯 (度)
        
        # 目標位置と姿勢（初期値）
        self.target_position = [0.0, 1.0, 0.0]  # x, y, z (m)
        self.target_yaw = 0.0  # yaw (度)
        
        # フォーメーション制御パラメータ
        self.formation_enabled = False  # フォーメーション制御の有効/無効
        self.drone_id = 0  # ドローンID（0=1号機、1=2号機）
        self.is_leader = True  # リーダーフラグ（デフォルトはリーダー）
        self.formation_offset = [0.0, 0.0, 0.0]  # フォーメーション内での相対位置
        
        # 合意アルゴリズムのパラメータ（論文の式(2)に基づく）- 個体差対応調整
        self.k1 = 1.4  # 位置ゲイン（定常偏差改善: 2.0→3.0→5.0）
        self.k2 = 0.6  # 速度ゲイン（フォーメーション維持: 2.0→3.0→4.0）
        self.trust_value = 1.0  # 信頼度（Phase 1では固定値1.0）
        
        # 他のドローンの情報を保持するための変数
        self.other_drone_positions = {}  # 他のドローンの位置情報
        self.other_drone_targets = {}    # 他のドローンの目標位置
        
        # Phase 2: リーダー交代対応
        self.last_leader_target = [0.0, 1.0, 0.0]  # 最後のリーダー目標位置
        self.other_drone_velocities = {}  # {drone_id: [vx, vy, vz]}
        
        # リーダー交代時の目標位置保護
        self.leader_switched = False  # リーダー交代発生フラグ（一度交代したら永続的に保護）
        
        # 速度推定用の履歴
        self.position_history = []
        self.velocity_estimate = [0.0, 0.0, 0.0]
        self.max_history_length = 5

        # CBF関連
        self.cbf_enabled = False  # Tモード時のみ上位でTrueに設定する想定
        # test.mdに基づくCBFパラメータ設定
        self.cbf_params = CBFParams(
            K1=0.009, K2=0.009,
            alpha1=1.0, alpha2=1.0, alpha3=1.0,
            alpha4=1.0, alpha5=1.0,
            Delta=0.0,  # 障害物回避の安全半径 [m]（test.mdの仮設定に合わせる）
            v_ref=0.01,  # 速度差制約の参照値
            p_ref=1.0,  # 機体間距離の下限 [m]
            enable_velocity_constraints=True  # 2式目（機体間距離CBF）を有効化
        )
        # 障害物はtest座標系で設定（x_test, y_test）
        self.cbf_obstacle_test = (-0.5, 0)  # ユーザー指定: test座標系(xo,yo)=(-0.5,0)
        # 速度算出用（MOCAP差分）
        self._last_pos = None  # proj座標 [x, y, z]
        self._last_time = None

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
        
    def increment_target_position(self, dx=0.0, dy=0.0, dz=0.0):
        """
        現在の目標位置を指定された増分だけ変更する
        
        Args:
            dx: x方向（前後）の増分 (m)
            dy: y方向（上下）の増分 (m)
            dz: z方向（左右）の増分 (m)
            
        Returns:
            list: 更新後の目標位置 [x, y, z]
        """
        # 現在の目標位置を取得
        current_target = self.get_target_position()
        
        # 増分を加算
        new_x = current_target[0] + dx
        new_y = current_target[1] + dy
        new_z = current_target[2] + dz
        
        # 更新した目標位置を設定
        self.set_target_position(new_x, new_y, new_z)
        
        # 更新後の目標位置を返す
        return self.get_target_position()
    
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
        try:
            # 入力値の検証（より詳細なエラー特定のため）
            if current_position is None:
                logging.error("現在位置がNoneです。有効な位置データが必要です。")
                return [0, 0, 0, 0]  # 安全なデフォルト値
                
            if not isinstance(current_position, list):
                logging.error(f"現在位置が正しい形式ではありません。リスト形式が必要です。渡された型: {type(current_position)}")
                return [0, 0, 0, 0]  # 安全なデフォルト値
                
            if len(current_position) < 3:
                logging.error(f"現在位置リストの要素数が不足しています。3要素が必要ですが、{len(current_position)}要素です。")
                return [0, 0, 0, 0]  # 安全なデフォルト値
                
            # NumPy配列が渡された場合はリストに変換
            if hasattr(current_position, 'tolist'):
                current_position = current_position.tolist()
            
            # target_positionも検証
            if not isinstance(self.target_position, list) or len(self.target_position) < 3:
                logging.error(f"目標位置が正しい形式ではありません: {self.target_position}")
                return [0, 0, 0, 0]  # 安全なデフォルト値
                
            # 位置の差分を計算
            error_x = self.target_position[0] - current_position[0]  # 前後方向の誤差
            error_y = self.target_position[1] - current_position[1]  # 上下方向の誤差
            error_z = self.target_position[2] - current_position[2]  # 左右方向の誤差
        except Exception as e:
            logging.error(f"位置差分計算中にエラーが発生しました: {e}")
            logging.error(f"current_position: {current_position}, target_position: {self.target_position}")
            return [0, 0, 0, 0]  # 安全なデフォルト値
        
        # クォータニオンからヨー角を計算（もし提供されていれば）
        if quaternion is not None and current_yaw is None:
            current_yaw = self.quaternion_to_yaw(quaternion)
        
        # ヨー角の差分を計算（ヨー角が与えられた場合のみ）
        error_yaw = 0
        if current_yaw is not None:
            # ヨー角の安定化: 非常に小さい値（±5度以内）は0とみなす
            YAW_STABILITY_THRESHOLD = 5.0  # 度
            
            # ヨー角を安定化（小さい値は0に）
            original_yaw = current_yaw  # 元の値を保存（ログ用）
            
            if abs(current_yaw) <= YAW_STABILITY_THRESHOLD:
                logging.debug(f"ヨー角安定化: {current_yaw}° → 0°に補正")
                current_yaw = 0.0
            
            error_yaw = self._calculate_yaw_error(self.target_yaw, current_yaw)
            
            # 簡略化された座標変換 - 単一の回転行列計算に統合
            yaw_rad = math.radians(current_yaw)
            sin_yaw = math.sin(yaw_rad)
            cos_yaw = math.cos(yaw_rad)
            
            # 回転行列を使った座標変換（より効率的）
            error_x_body = error_x * cos_yaw + error_z * sin_yaw
            error_z_body = error_z * cos_yaw - error_x * sin_yaw
            
            # デバッグ: 変換前後の値を表示
            logging.debug(f"元のヨー角: {original_yaw}°, 補正後: {current_yaw}°")
            logging.debug(f"座標変換前: error_x={error_x:.2f}, error_z={error_z:.2f}")
            logging.debug(f"座標変換後: error_x_body={error_x_body:.2f}, error_z_body={error_z_body:.2f}")
            
            # ドローンが反転している場合の簡易判定と補正
            if abs(current_yaw) > 85.0:  # 85度以上は反転とみなす
                logging.debug(f"ドローンが反転姿勢: yaw={current_yaw}° -> 制御方向を反転")
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

        # === CBFフィルタ（Tモード時のみ上位で有効化） ===
        print(f"🔍 CBF処理チェック: cbf_enabled={self.cbf_enabled}")
        logging.info(f"CBF処理チェック: cbf_enabled={self.cbf_enabled}")
        if self.cbf_enabled:
            # MOCAP差分から速度推定（前処理なし）
            now = time.time()
            vx, vz = 0.0, 0.0  # proj座標系の前後(X),左右(Z)
            if self._last_pos is not None and self._last_time is not None:
                dt = max(1e-6, now - self._last_time)
                vx = (current_position[0] - self._last_pos[0]) / dt
                vz = (current_position[2] - self._last_pos[2]) / dt
            # 更新
            self._last_pos = current_position.copy() if isinstance(current_position, list) else list(current_position)
            self._last_time = now

            # test座標系に変換: x_test<-Z_proj, y_test<-X_proj
            x_test = current_position[2]
            y_test = current_position[0]
            xdot_test = vz
            ydot_test = vx

            # 他のドローンの速度を取得（test座標系に変換）
            other_velocity_test = None
            if self.cbf_params.enable_velocity_constraints and len(self.other_drone_positions) > 0:
                # 他のドローンのIDを取得（自分以外）
                other_ids = [id for id in self.other_drone_positions.keys() if id != self.drone_id]
                if other_ids:
                    other_id = other_ids[0]  # 2台システムなので最初の1台
                    other_pos = self.other_drone_positions[other_id]
                    # proj座標系 [x, y, z] -> test座標系 [x_test, y_test]
                    # x_test<-Z_proj, y_test<-X_proj
                    xj_test = other_pos[2]
                    yj_test = other_pos[0]
                    other_velocity_test = (xj_test, yj_test, 0.0, 0.0)

            # 名目入力（test座標系）: u_x(test)=rc_roll(control_z), u_y(test)=rc_pitch(control_x)
            u_nom_test = (float(control_z), float(control_x))

            u_safe, info = enforce_cbf(
                u_nom=u_nom_test,
                state_test=(x_test, y_test, xdot_test, ydot_test),
                obstacle_test=self.cbf_obstacle_test,
                params=self.cbf_params,
                other_velocity=other_velocity_test,
            )

            # CBF情報をインスタンス変数に保存（CSVログ用、仕様書準拠）
            rc_nom_x, rc_nom_z = float(control_x), float(control_z)  # 名目入力（修正前, fb=X, lr=Z）
            # u_safe[0]: test x軸(左右) → rc_lr(Z), u_safe[1]: test y軸(前後) → rc_fb(X)
            rc_safe_x, rc_safe_z = float(u_safe[1]), float(u_safe[0])  # CBF適用後（fb, lr）
            rc_diff_norm = ((rc_safe_x - rc_nom_x)**2 + (rc_safe_z - rc_nom_z)**2)**0.5
            
            # 数学的値の計算（test.mdの数式に基づく）
            # 障害物位置: (x_o, y_o) = (-0.6, 1.2) in test coordinates
            x_o, y_o = self.cbf_obstacle_test[0], self.cbf_obstacle_test[1]
            Delta = self.cbf_params.Delta
            alpha1 = self.cbf_params.alpha1
            alpha2 = self.cbf_params.alpha2
            alpha3 = self.cbf_params.alpha3
            K1 = self.cbf_params.K1
            K2 = self.cbf_params.K2
            
            # h(x) = (x-x_o)^2 + (y-y_o)^2 - Delta^2
            h_x = (x_test - x_o)**2 + (y_test - y_o)**2 - Delta**2
            
            # HOh(x) = CBF制約式の左辺（test.md仕様通り）
            # 2*xdot^2 + 2*(alpha2+alpha3)*(x-x_o)*xdot + 2*ydot^2 + 2*(alpha2+alpha3)*(y-y_o)*ydot
            # + 2*K1*(x-x_o)*u_x + 2*K2*(y-y_o)*u_y + alpha1*alpha2*(x-x_o)^2 + alpha1*alpha2*(y-y_o)^2 - alpha1*alpha2*Delta^2
            
            # 距離と速度の計算（デバッグ用）
            dx = x_test - x_o
            dy = y_test - y_o
            dist_sq = dx**2 + dy**2
            
            # test.md仕様通りのHOh(x)計算
            HOh_x = (2 * xdot_test**2 + 
                     2 * (alpha2 + alpha3) * dx * xdot_test +
                     2 * ydot_test**2 + 
                     2 * (alpha2 + alpha3) * dy * ydot_test +
                     2 * K1 * dx * u_nom_test[0] +
                     2 * K2 * dy * u_nom_test[1] +
                     alpha1 * alpha2 * dx**2 +
                     alpha1 * alpha2 * dy**2 -
                     alpha1 * alpha2 * Delta**2)
            
            # 各項の詳細分析（デバッグ用）
            velocity_term = 2 * xdot_test**2 + 2 * ydot_test**2
            coupling_term = 2 * (alpha2 + alpha3) * (dx * xdot_test + dy * ydot_test)
            control_term = 2 * K1 * dx * u_nom_test[0] + 2 * K2 * dy * u_nom_test[1]
            position_term = alpha1 * alpha2 * (dx**2 + dy**2)
            barrier_term = -alpha1 * alpha2 * Delta**2
            
            print(f"🚀 CBF実行: rc_nom=[{rc_nom_x:.1f},{rc_nom_z:.1f}] -> rc_safe=[{rc_safe_x:.1f},{rc_safe_z:.1f}], diff_norm={rc_diff_norm:.3f}")
            print(f"📊 CBF数学値: h(x)={h_x:.3f}, HOh(x)={HOh_x:.3f}")
            print(f"🔍 CBF詳細: 位置=({x_test:.2f},{y_test:.2f}), 障害物=({x_o},{y_o}), 距離²={dist_sq:.3f}, Δ²={Delta**2:.3f}")
            print(f"🔍 CBF分解: velocity={velocity_term:.3f}, coupling={coupling_term:.3f}, control={control_term:.3f}")
            print(f"🔍 CBF分解: position={position_term:.3f}, barrier={barrier_term:.3f}")
            print(f"🔍 CBF状態: {'制約違反' if HOh_x < 0 else '制約満足'}, QP状態={info.get('qp_status', 'unknown')}")
            logging.info(f"CBF実行: rc_nom=[{rc_nom_x:.1f},{rc_nom_z:.1f}] -> rc_safe=[{rc_safe_x:.1f},{rc_safe_z:.1f}], diff_norm={rc_diff_norm:.3f}")
            logging.info(f"CBF制約: HOh_x={HOh_x:.3f}, h(x)={h_x:.3f}")
            
            self._cbf_last_info = {
                'fire_flag': True,  # CBFが発火した
                'rc_safe_x': rc_safe_x,
                'rc_safe_z': rc_safe_z,
                'rc_diff_norm': rc_diff_norm,
                'qp_status': 'optimal' if not info['fell_back'] else 'infeasible',
                'active_constraint_id': 0,
                'rc_nom_x': rc_nom_x,
                'rc_nom_z': rc_nom_z,
                'h_x': h_x,  # 元のh(x)を使用
                'HOh_x': HOh_x,  # test.md仕様通りのHOh(x)
                # 詳細分析用項目
                'velocity_term': velocity_term,  # 速度項
                'coupling_term': coupling_term,  # 結合項
                'control_term': control_term,  # 制御項
                'position_term': position_term,  # 位置項
                'barrier_term': barrier_term,  # バリア項
                'distance_sq': dist_sq,  # 障害物からの距離²
                # 速度差制約情報
                'h2_value': info.get('h2_value', 0.0),  # X方向速度差制約の値
                'h2_satisfied': info.get('h2_satisfied', True),
                'h3_value': info.get('h3_value', 0.0),  # Y方向速度差制約の値
                'h3_satisfied': info.get('h3_satisfied', True),
                'velocity_diff_x': info.get('velocity_diff_x', 0.0),
                'velocity_diff_y': info.get('velocity_diff_y', 0.0),
            }
            
            # 結果をproj/Tello入力へ反映（rc_fb=proj X= u_y, rc_lr=proj Z= u_x）
            control_x = int(self._clamp(float(u_safe[1]), -self.max_speed_x, self.max_speed_x))
            control_z = int(self._clamp(float(u_safe[0]), -self.max_speed_z, self.max_speed_z))
            # 参考ログ（必要に応じて上位のCSVへ拡張）
            logging.debug(f"CBF: feasible_nom={info['feasible_nom']}, projected={info['projected']}, fallback={info['fell_back']}, "
                          f"h1={info['h1_value']:.3f}(ok={info['h1_satisfied']}), h2={info['h2_value']:.3f}(ok={info['h2_satisfied']}), h3={info['h3_value']:.3f}(ok={info['h3_satisfied']})")
        else:
            # CBF無効時はデフォルト値を設定（仕様書準拠）
            logging.debug(f"CBF無効: rc=[{control_x},{control_z}]")
            self._cbf_last_info = {
                'fire_flag': False,  # CBF発火せず
                'rc_safe_x': float(control_x),
                'rc_safe_z': float(control_z),
                'rc_diff_norm': 0.0,  # 名目と同じなので差分は0
                'qp_status': None,  # CBF使用せず
                'active_constraint_id': None,
                # h2, h3情報もデフォルト値で埋める
                'h2_value': 0.0,
                'h2_satisfied': True,
                'h3_value': 0.0,
                'h3_satisfied': True,
                'velocity_diff_x': 0.0,
                'velocity_diff_y': 0.0
            }
        
        # Telloの制御形式に合わせて返す
        # Telloの制御値： [left_right, forward_backward, up_down, yaw]
        result = [control_z, control_x, control_y, control_yaw]
        logging.debug(f"Tello制御値 [lr,fb,ud,yaw]={result}")
        return result
    
    def get_cbf_info(self):
        """
        CBF関連情報を取得（CSVログ用、仕様書準拠）
        
        Returns:
            dict: CBF情報辞書 (fire_flag, rc_safe_*, rc_diff_norm, qp_status, active_constraint_id, fallback_flag)
        """
        return getattr(self, '_cbf_last_info', {
            'fire_flag': False, 'rc_safe_x': None, 'rc_safe_z': None, 'rc_diff_norm': None,
            'qp_status': None, 'active_constraint_id': None
        })
    
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
    
    # =============================================================================
    # フォーメーション制御関連メソッド
    # =============================================================================
    
    def enable_formation_control(self, drone_id: int, is_leader: bool = False):
        """
        フォーメーション制御を有効化する
        
        Args:
            drone_id: ドローンID（0=1号機、1=2号機）
            is_leader: リーダーフラグ
        """
        self.formation_enabled = True
        self.drone_id = drone_id
        self.is_leader = is_leader
        
        # フォーメーションオフセットを設定（横一列、1m間隔）
        self._update_formation_offset()
        
        print(f"ドローン{drone_id+1}: フォーメーション制御有効化 (リーダー: {is_leader}, オフセット: {self.formation_offset})")
    
    def update_leader_status(self, is_leader: bool):
        """
        リーダー状態を動的に更新する（Phase 2: リーダー交代対応）
        
        Args:
            is_leader: 新しいリーダーフラグ
        """
        if self.is_leader != is_leader:
            print(f"ドローン{self.drone_id+1}: リーダー状態変更 {self.is_leader} -> {is_leader}")
            self.is_leader = is_leader
            self._update_formation_offset()
            
            # リーダー交代時の目標位置継承処理
            if is_leader:
                print(f"ドローン{self.drone_id+1}: 新リーダーとして目標位置を継承")
            
            # リーダー交代時の目標位置継承処理
            if is_leader:
                print(f"ドローン{self.drone_id+1}: 新リーダーとして目標位置を継承")
                # リーダー交代発生フラグを設定
                self.leader_switched = True
    
    def _update_formation_offset(self):
        """
        現在のリーダー状態に基づいてフォーメーションオフセットを更新
        """
        if self.is_leader:
            self.formation_offset = [0.0, 0.0, 0.0]  # リーダーは基準位置
        else:
            # フォロワーのオフセット（ドローンIDに基づく）
            if self.drone_id == 0:  # 1号機がフォロワーの場合
                self.formation_offset = [0.0, 0.0, 1.0]  # Z軸方向に+1mオフセット
            elif self.drone_id == 1:  # 2号機がフォロワーの場合
                self.formation_offset = [-0.8, 0.0, 0.0]  # X軸方向に-0.8mオフセット
    
    def disable_formation_control(self):
        """
        フォーメーション制御を無効化する
        """
        self.formation_enabled = False
        print(f"ドローン{self.drone_id+1}: フォーメーション制御無効化")
    
    def update_other_drone_info(self, drone_id: int, position: list, velocity: list = None, target: list = None):
        """
        他のドローンの情報を更新する
        
        Args:
            drone_id: 他のドローンのID
            position: 位置 [x, y, z]
            velocity: 速度 [vx, vy, vz] (オプション)
            target: 目標位置 [x, y, z] (オプション)
        """
        self.other_drone_positions[drone_id] = position.copy() if position else [0.0, 0.0, 0.0]
        
        if velocity is not None:
            self.other_drone_velocities[drone_id] = velocity.copy()
        
        if target is not None:
            self.other_drone_targets[drone_id] = target.copy()
    
    def _estimate_velocity(self, current_position: list, dt: float = 0.1) -> list:
        """
        位置履歴から速度を推定する
        
        Args:
            current_position: 現在位置 [x, y, z]
            dt: 時間ステップ
        
        Returns:
            list: 推定速度 [vx, vy, vz]
        """
        # 位置履歴を更新
        self.position_history.append(current_position.copy())
        if len(self.position_history) > self.max_history_length:
            self.position_history.pop(0)
        
        # 履歴が十分にある場合は速度を推定
        if len(self.position_history) >= 2:
            prev_pos = self.position_history[-2]
            curr_pos = self.position_history[-1]
            
            velocity = [
                (curr_pos[0] - prev_pos[0]) / dt,
                (curr_pos[1] - prev_pos[1]) / dt,
                (curr_pos[2] - prev_pos[2]) / dt
            ]
            
            # 指数移動平均でスムージング
            alpha = 0.7
            self.velocity_estimate = [
                alpha * velocity[0] + (1 - alpha) * self.velocity_estimate[0],
                alpha * velocity[1] + (1 - alpha) * self.velocity_estimate[1],
                alpha * velocity[2] + (1 - alpha) * self.velocity_estimate[2]
            ]
        
        return self.velocity_estimate.copy()
    
    def calculate_formation_control(self, current_position: list, dt: float = 0.1) -> list:
        """
        合意アルゴリズムに基づくフォーメーション制御を計算する
        論文の式(2): u_i = -Σ w_ij [k1(r_i - r_j) - l_i * k1(d_i - d_j) + k2(ṙ_i - ṙ_j) - l_i * k2(ḋ_i - ḋ_j)]
        
        Args:
            current_position: 現在位置 [x, y, z]
            dt: 時間ステップ
        
        Returns:
            list: フォーメーション制御入力 [ax, ay, az] (加速度)
        """
        if not self.formation_enabled:
            return [0.0, 0.0, 0.0]
        
        # 現在の速度を推定
        current_velocity = self._estimate_velocity(current_position, dt)
        
        # フォーメーション制御入力の初期化
        formation_input = [0.0, 0.0, 0.0]
        
        # 他のドローンとの相互作用を計算
        for other_drone_id, other_position in self.other_drone_positions.items():
            if other_drone_id == self.drone_id:
                continue  # 自分自身はスキップ
            
            # 信頼度重み w_ij = η_i * η_j * a_ij
            # Phase 1では信頼度は1.0固定、隣接行列要素a_ij=1.0と仮定
            w_ij = self.trust_value * 1.0 * 1.0  # η_i * η_j * a_ij
            
            # 他のドローンの速度を取得（なければゼロと仮定）
            other_velocity = self.other_drone_velocities.get(other_drone_id, [0.0, 0.0, 0.0])
            
            # 目標相対位置の計算（d_i - d_j）
            # リーダーの場合はd_i = [0,0,0]、フォロワーの場合はd_i = formation_offset
            my_desired_offset = self.formation_offset if not self.is_leader else [0.0, 0.0, 0.0]
            
            # 他のドローンの目標オフセット（簡単化のため、相手がリーダーなら[0,0,0]、フォロワーなら[0,0,1]と仮定）
            if other_drone_id == 0:  # 相手が1号機（リーダー）
                other_desired_offset = [0.0, 0.0, 0.0]
            else:  # 相手が2号機（フォロワー）
                other_desired_offset = [0.0, 0.0, 1.0]
            
            desired_offset_diff = [
                my_desired_offset[0] - other_desired_offset[0],
                my_desired_offset[1] - other_desired_offset[1],
                my_desired_offset[2] - other_desired_offset[2]
            ]
            
            # 位置誤差項：k1(r_i - r_j)
            position_error = [
                current_position[0] - other_position[0],
                current_position[1] - other_position[1],
                current_position[2] - other_position[2]
            ]
            
            # 速度誤差項：k2(ṙ_i - ṙ_j)
            velocity_error = [
                current_velocity[0] - other_velocity[0],
                current_velocity[1] - other_velocity[1],
                current_velocity[2] - other_velocity[2]
            ]
            
            # リーダーフラグ l_i（リーダー=0、フォロワー=1）
            l_i = 0.0 if self.is_leader else 1.0
            
            # 合意アルゴリズムの制御入力を計算
            for axis in range(3):  # x, y, z軸
                control_term = (
                    self.k1 * position_error[axis] - 
                    l_i * self.k1 * desired_offset_diff[axis] +
                    self.k2 * velocity_error[axis] - 
                    l_i * self.k2 * 0.0  # 目標速度の差分はゼロと仮定
                )
                
                formation_input[axis] -= w_ij * control_term
        
        return formation_input
    
    def update_target_with_formation(self, leader_target: list) -> list:
        """
        リーダーの目標位置に基づいてフォロワーの目標位置を計算する
        
        Args:
            leader_target: リーダーの目標位置 [x, y, z]
        
        Returns:
            list: 更新された目標位置 [x, y, z]
        """
        if not self.formation_enabled or self.is_leader:
            # リーダーの場合はそのまま返す
            return leader_target.copy()
        
        # 🔒 リーダー交代フラグチェック: 一度リーダー交代が発生したら目標位置の上書きを防ぐ
        if self.leader_switched:
            print(f"🔒 ドローン{self.drone_id+1}: リーダー交代後のため、フォーメーション制御による目標位置上書きをスキップ")
            return self.get_target_position()  # 現在の目標位置をそのまま返す
        
        # フォロワーの場合はリーダーの目標位置 + フォーメーションオフセット
        follower_target = [
            leader_target[0] + self.formation_offset[0],
            leader_target[1] + self.formation_offset[1],
            leader_target[2] + self.formation_offset[2]
        ]
        
        # 自分の目標位置を更新
        self.set_target_position(follower_target[0], follower_target[1], follower_target[2])
        
        return follower_target


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
