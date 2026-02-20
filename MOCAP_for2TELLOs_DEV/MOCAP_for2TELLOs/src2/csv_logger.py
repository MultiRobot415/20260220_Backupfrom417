#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CSVロガーモジュール
ドローン制御データとTello状態データをCSVファイルに記録するための機能を提供します

作成日: 2025-07-25
更新日: 2025-07-29 (ドローン状態ログ機能追加)
"""

import os
import csv
import time
import datetime
import numpy as np
import re

# ログファイルの保存先ディレクトリ
LOG_DIR = "src2_results"  # 結果保存ディレクトリ
control_log_file = None
control_csv_writer = None
observer_log_file = None
observer_csv_writer = None
observer_log_path = None
# Tello状態ログ用の変数
tello_status_log_file = None
tello_status_csv_writer = None

def init_csv_logger():
    """
    CSVロガーを初期化する
    """
    try:
        global control_log_file, control_csv_writer, observer_log_file, observer_csv_writer, observer_log_path
        global tello_status_log_file, tello_status_csv_writer
        
        # 結果保存用ディレクトリの作成
        os.makedirs(LOG_DIR, exist_ok=True)
        
        # 現在時刻を取得してファイル名を生成
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        control_log_path = os.path.join(LOG_DIR, f"control_log_{timestamp}.csv")
        observer_log_path = os.path.join(LOG_DIR, f"observer_log_{timestamp}.csv")
        tello_status_log_path = os.path.join(LOG_DIR, f"tello_status_{timestamp}.csv")
        
        # コントロールデータ用CSVファイルを開いてヘッダーを書き込み
        control_log_file = open(control_log_path, 'w', newline='')
        control_csv_writer = csv.writer(control_log_file)
        
        # コントロールデータ用CSVヘッダー（不要列削除済み）
        control_csv_writer.writerow([
            'timestamp', 'drone_id', 'role', 'mode',
            'x', 'y', 'z',
            'target_x', 'target_y', 'target_z',
            'error_x', 'error_y', 'error_z',
            'rc_lr', 'rc_fb', 'rc_ud', 'rc_yaw',
            # S列(battery)からX列(quat_w)まで削除済み
            'trust',
            'obs_error_x', 'obs_error_y', 'obs_error_z',
            'obs_state_x', 'obs_state_y', 'obs_state_z'
            # AF列(battery)からAK列(position_missing)まで削除済み
        ])
        
        # オブザーバーデータ用CSVファイルを開いてヘッダーを書き込み
        observer_log_file = open(observer_log_path, 'w', newline='')
        observer_csv_writer = csv.writer(observer_log_file)
        
        # オブザーバーデータCSVヘッダー
        observer_csv_writer.writerow([
            'timestamp', 'drone_id', 'role', 'mode',
            'x', 'y', 'z',
            'target_x', 'target_y', 'target_z',
            'error_x', 'error_y', 'error_z',
            'rc_lr', 'rc_fb', 'rc_ud', 'rc_yaw',
            'tilt_x', 'tilt_y', 'trust',
            'obs_error_x', 'obs_error_y', 'obs_error_z',
            'obs_state_x', 'obs_state_y', 'obs_state_z',
            'battery', 'height', 'leader_status', 'exec_time'
        ])
        
        # Tello状態データ用CSVファイルを開いてヘッダーを書き込み
        tello_status_log_file = open(tello_status_log_path, 'w', newline='')
        tello_status_csv_writer = csv.writer(tello_status_log_file)
        
        # Tello状態データCSVヘッダー
        tello_status_csv_writer.writerow([
            'timestamp', 'drone_id', 
            'pitch', 'roll', 'yaw',  # IMU角度
            'vgx', 'vgy', 'vgz',    # 速度
            'templ', 'temph',        # 温度
            'tof', 'h',              # 距離センサーと高度
            'bat', 'baro',           # バッテリーと気圧
            'time', 'agx', 'agy', 'agz'  # 飛行時間と加速度
        ])
        
        print(f"CSVロガーを初期化しました: {control_log_path}")
        print(f"コントロールデータCSVファイルを作成しました: {control_log_path}")
        print(f"オブザーバーデータCSVファイルを作成しました: {observer_log_path}")
        print(f"Tello状態データCSVファイルを作成しました: {tello_status_log_path}")
        
    except Exception as e:
        print(f"CSVロガー初期化エラー: {e}")

def log_tello_status(drone_id, status_data):
    """
    Tello状態データをCSVに記録する
    
    Args:
        drone_id (int): ドローンID（1または2）
        status_data (str): Telloから取得した状態データ文字列
                           例: "pitch:0;roll:0;yaw:0;vgx:0;vgy:0;vgz:0;templ:60;temph:63;tof:10;h:0;bat:87;baro:14.64;time:0;agx:0.00;agy:0.00;agz:0.00;"
    """
    global tello_status_csv_writer
    
    if not tello_status_csv_writer:
        print("警告: Tello状態ログファイルが初期化されていません。init_csv_logger()を先に呼び出してください。")
        return
    
    # 状態データが空またはNoneの場合は記録しない
    if not status_data:
        print(f"警告: ドローン{drone_id}の状態データが空です。スキップします。")
        return
    
    try:
        # 現在のタイムスタンプを取得
        timestamp = time.time()
        
        print(f"ドローン{drone_id}の状態データ解析開始: {status_data}")
        
        # status_dataがタプルか文字列かをチェック
        if isinstance(status_data, tuple):
            # タプルの場合は最初の要素（文字列）を使用
            state_str = status_data[0]
            is_dummy = status_data[1] if len(status_data) > 1 else False
            # ダミーデータであることをログに記録
            if is_dummy:
                print(f"Note: ドローン{drone_id}の状態データはダミーです")
        else:
            # 通常の文字列の場合
            state_str = status_data
            is_dummy = False
        
        # 状態データを解析（フォーマット: "pitch:0;roll:0;yaw:0;...")
        data_dict = {}
        pairs = state_str.split(';')
        
        # デバッグ出力を削減
        
        for pair in pairs:
            if ':' in pair:
                key, value = pair.split(':', 1)
                # 空白を取り除く
                key = key.strip()
                value = value.strip()
                
                # デバッグ出力を削減
                if value:  # 値が空でない場合のみ処理
                    try:
                        # 数値に変換を試みる
                        data_dict[key] = float(value)
                    except ValueError:
                        # 変換できない場合はそのまま文字列として保存
                        data_dict[key] = value
        
        # 特定キーがない場合はダミーデータを生成
        if len(data_dict) < 3:  # あまりにもデータが少ない場合
            print(f"警告: 解析できるデータが少なすぎます。ダミーデータを生成します。")
            # 最低限のダミーデータ
            data_dict['pitch'] = 0.0
            data_dict['roll'] = 0.0
            data_dict['yaw'] = 0.0
            data_dict['vgx'] = 0.0
            data_dict['vgy'] = 0.0
            data_dict['vgz'] = 0.0
            data_dict['h'] = 0.0
            data_dict['bat'] = 0.0
            
        # 取得できたキーの表示を削減
        
        # CSVに記録するデータ行を作成
        row = [timestamp, drone_id]
        
        # 各状態パラメータを取得（存在しない場合は0を使用）
        target_params = ['pitch', 'roll', 'yaw', 'vgx', 'vgy', 'vgz', 
                        'templ', 'temph', 'tof', 'h', 'bat', 'baro', 
                        'time', 'agx', 'agy', 'agz']
        
        for param in target_params:
            # 値がない場合は0を使用して必ずCSVに値を記録
            value = data_dict.get(param, 0)
            row.append(value)
            if param not in data_dict:
                print(f"  情報: パラメータ '{param}' にはデフォルト0を使用します")
        
        # CSVに書き込み
        tello_status_csv_writer.writerow(row)
        print(f"ドローン{drone_id}の状態データをCSVに記録しました。パラメータ数: {len(data_dict)}")
        
    except Exception as e:
        print(f"Tello状態データログエラー: {e}")
        import traceback
        traceback.print_exc()

def close_csv_logger():
    """
    CSVログファイルを閉じる
    """
    global control_log_file, observer_log_file, tello_status_log_file
    
    if control_log_file:
        control_log_file.close()
        print("コントロールデータCSVファイルを閉じました")
        
    if observer_log_file:
        observer_log_file.close()
        print("オブザーバーデータCSVファイルを閉じました")
        
    if tello_status_log_file:
        tello_status_log_file.close()
        print("Tello状態データCSVファイルを閉じました")


def log_control_data(drone_index, mode, mocap_position, target_position, error, rc_values, 
                    height, battery, start_time, quaternion=None, trust_metric=None, role="",
                    obs_error=None, obs_state=None,  # Phase 2: オブザーバー値を追加
                    control_anomaly_count=0, position_missing_count=0):  # 異常カウンター追加
    """
    制御データをCSVファイルに記録
    
    Args:
        drone_index: ドローンのインデックス
        mode: 制御モード (文字列)
        mocap_position: MOCAPから取得した位置 [x, y, z]
        target_position: 目標位置 [x, y, z]
        error: 位置誤差 [x, y, z]
        rc_values: 制御値 [lr, fb, ud, yaw]
        height: ドローンから報告された高度 (cm)
        battery: バッテリー残量 (%)
        start_time: 処理開始時刻 (time.time()の値)
        quaternion: 回転クォータニオン (省略可) {"x": x, "y": y, "z": z, "w": w}
        trust_metric: 信頼度メトリック (省略可)
        role: ドローンの役割 ("leader"または"follower", 省略可)
        obs_error: オブザーバー残差 [x, y, z] (省略可)
        obs_state: オブザーバー推定状態 [x, y, z] (省略可)
    """
    global control_csv_writer
    
    # CSVライターが初期化されていない場合はスキップ
    if not control_csv_writer:
        return
    
    # 処理時間を計算
    process_time = 0 if start_time is None else time.time() - start_time
    
    # クォータニオンの処理
    qx, qy, qz, qw = None, None, None, None
    if quaternion:
        qx = quaternion.get("x", None)
        qy = quaternion.get("y", None)
        qz = quaternion.get("z", None)
        qw = quaternion.get("w", None)
    
    # MOCAP位置データの処理 (タプル形式とダミーデータのチェック)
    is_dummy_position = False
    if isinstance(mocap_position, tuple):
        # タプル形式 (位置データ, ダミーフラグ)
        position_data = mocap_position[0]
        is_dummy_position = mocap_position[1] if len(mocap_position) > 1 else False
    else:
        position_data = mocap_position
        
    # ダミーデータの場合はログに記録
    if is_dummy_position:
        print(f"注: ドローン{drone_index}のMOCAP位置データはダミーまたは利用不可")
    
    # Noneチェック (NoneはCSVに書き込めないため、空文字列に変換)
    if position_data is not None:
        pos_x, pos_y, pos_z = position_data
    else:
        pos_x, pos_y, pos_z = None, None, None
    tar_x, tar_y, tar_z = target_position if target_position is not None else [None, None, None]
    err_x, err_y, err_z = error if error is not None else [None, None, None]
    rc_lr, rc_fb, rc_ud, rc_yaw = rc_values if rc_values is not None else [None, None, None, None]
    
    # Phase 2: オブザーバー値の処理
    obs_err_x, obs_err_y, obs_err_z = obs_error if obs_error is not None else [None, None, None]
    obs_st_x, obs_st_y, obs_st_z = obs_state if obs_state is not None else [None, None, None]
    
    # データを記録（ヘッダーと一致させるため、削除した列を除外）
    row = [
        time.time(), drone_index, role, mode,
        pos_x, pos_y, pos_z,
        tar_x, tar_y, tar_z,
        err_x, err_y, err_z,
        rc_lr, rc_fb, rc_ud, rc_yaw,
        # height, battery, process_time, qx, qy, qz, qw は削除済み
        trust_metric,
        obs_err_x, obs_err_y, obs_err_z,  # Phase 2: オブザーバー残差
        obs_st_x, obs_st_y, obs_st_z     # Phase 2: オブザーバー推定状態
        # control_anomaly_count, position_missing_count も削除済み
    ]
    
    # None値を空文字列に変換
    row = ["" if v is None else v for v in row]
    
    try:
        control_csv_writer.writerow(row)
        # データ書き込み後にファイルをフラッシュして確実に書き込む
        if control_log_file:
            control_log_file.flush()
        print(f"CSV記録: ドローン{drone_index}のデータを記録しました [mode={mode}]")
    except Exception as e:
        print(f"CSVログ記録エラー: {e}")
    


def log_observer_data(drone_index, position_est, velocity_est, residual, trust, is_leader):
    """
    オブザーバーデータをCSVに記録する
    
    Args:
        drone_index: ドローンのインデックス
        position_est: 推定位置 [x, y, z]
        velocity_est: 推定速度 [vx, vy, vz]
        residual: 残差 [rx, ry, rz]
        trust: 信頼度
        is_leader: リーダーかどうか
    """
    try:
        global observer_log_file, observer_log_path, observer_csv_writer
        
        # 初期化されていない場合は初期化する
        if not observer_log_path or not observer_log_file:
            print("警告: オブザーバーCSVログが初期化されていません。init_csv_logger()を呼び出してください。")
            # 緊急対応として初期化
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            os.makedirs(LOG_DIR, exist_ok=True)
            observer_log_path = os.path.join(LOG_DIR, f"observer_log_{timestamp}_emergency.csv")
            observer_log_file = open(observer_log_path, 'w', newline='')
            observer_csv_writer = csv.writer(observer_log_file)
            observer_csv_writer.writerow([
                'timestamp', 'drone_id', 'role', 'mode',
                'x', 'y', 'z',
                'target_x', 'target_y', 'target_z',
                'error_x', 'error_y', 'error_z',
                'rc_lr', 'rc_fb', 'rc_ud', 'rc_yaw',
                'tilt_x', 'tilt_y', 'trust',
                'obs_error_x', 'obs_error_y', 'obs_error_z',
                'obs_state_x', 'obs_state_y', 'obs_state_z',
                'battery', 'height', 'leader_status', 'exec_time'
            ])
        
        # 位置と速度のデータを展開
        pos_x, pos_y, pos_z = position_est if position_est is not None else [None, None, None]
        vel_x, vel_y, vel_z = velocity_est if velocity_est is not None else [None, None, None]
        res_x, res_y, res_z = residual if residual is not None else [None, None, None]
        
        # データをCSV形式に変換して書き込み
        role = "leader" if is_leader else "follower"
        row = [
            time.time(), drone_index + 1, role, "auto",
            pos_x, pos_y, pos_z,
            None, None, None,  # 目標位置（オブザーバー側では記録しない）
            None, None, None,  # 誤差（オブザーバー側では記録しない）
            None, None, None, None,  # RC値（オブザーバー側では記録しない）
            None, None, trust,  # tiltとtrust
            res_x, res_y, res_z,  # 残差
            pos_x, pos_y, pos_z,  # 状態（位置）
            None, None,  # バッテリーと高度
            1 if is_leader else 0,  # リーダー状態
            None  # 実行時間
        ]
        
        # None値を空文字列に変換
        row = ["" if v is None else v for v in row]
        
        # CSVファイルに書き込み
        # グローバル変数のwriterを使用
        observer_csv_writer.writerow(row)
        
        # データ書き込み後にファイルをフラッシュして確実に書き込む
        observer_log_file.flush()
        print(f"オブザーバーデータ記録: ドローン{drone_index+1}のデータを記録しました [trust={trust:.2f}, leader={is_leader}]")
    
    except Exception as e:
        print(f"オブザーバーデータログ記録エラー: {e}")


def csv_debug_log(event_type, subject, data=None):
    """
    デバッグ情報をCSVに記録する簡易関数
    
    Args:
        event_type: イベントタイプ
        subject: 対象
        data: 記録するデータ (省略可)
    """
    try:
        # 結果保存用ディレクトリの作成
        os.makedirs(LOG_DIR, exist_ok=True)
        
        # デバッグログファイルパス
        debug_log_path = os.path.join(LOG_DIR, "debug_events.csv")
        
        # ファイルが存在しない場合はヘッダーを書き込む
        write_header = not os.path.exists(debug_log_path)
        
        with open(debug_log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(['timestamp', 'event_type', 'subject', 'data'])
            
            # データをCSV形式に変換
            data_str = str(data) if data is not None else ""
            writer.writerow([time.time(), event_type, subject, data_str])
    
    except Exception as e:
        print(f"デバッグログ記録エラー: {e}")
