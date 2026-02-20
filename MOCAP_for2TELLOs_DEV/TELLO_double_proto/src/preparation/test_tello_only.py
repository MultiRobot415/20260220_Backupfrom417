#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tello通信テスト - Telloドローンの通信のみをテストするプログラム

このプログラムは、Telloドローンとの通信のみをテストします。
モーションキャプチャシステムとの通信は行いません。
"""

import sys
import time
import KeyPressModule as kp
from custom_tello import CustomTello, TelloManager

# グローバル変数
tello_manager = None  # TelloManagerオブジェクト
drone = None  # Telloオブジェクト
is_flying = False  # 飛行状態
should_stop = False  # プログラム終了フラグ

# 制御パラメータ（デフォルト値）
SPEED = 50  # 移動速度
ROTATION_SPEED = 50  # 回転速度
INTERVAL = 0.05  # コマンド送信間隔

def initialize_drone():
    """ドローンの初期化と接続"""
    global tello_manager, drone
    
    print("ドローンへの接続を開始します...")
    
    # TelloManagerを初期化
    print("TelloManagerを初期化しています...")
    tello_manager = TelloManager()
    
    # 1機のドローンを検索
    print("1機のTelloドローンを検索しています...")
    tello_manager.find_available_tello(1)
    
    # 検出されたドローンを取得
    drones = tello_manager.get_tello_list()
    
    if len(drones) == 0:
        print("ドローンが見つかりません。プログラムを終了します。")
        return False
    
    # 1機目のドローンを使用
    drone = drones[0]
    
    # 接続確認
    print("ドローンに接続しています...")
    try:
        drone.connect()
        # バッテリー残量を確認
        battery = drone.get_battery()
        print(f"ドローン (IP: {drone.tello_ip}) に接続しました。バッテリー残量: {battery}%")
        
        # バッテリー残量が少ない場合は警告
        if battery < 20:
            print(f"警告: バッテリー残量が少なくなっています ({battery}%)")
    except Exception as e:
        print(f"ドローンへの接続に失敗しました: {e}")
        return False
    
    return True

def takeoff():
    """ドローンを離陸させる"""
    global is_flying
    if is_flying:
        print("既に飛行中です")
        return
        
    print("ドローンを離陸させます...")
    try:
        drone.takeoff()
        time.sleep(2)  # 安定化のための待機
        is_flying = True
        print("ドローンが離陸しました")
    except Exception as e:
        print(f"ドローンの離陸に失敗しました: {e}")

def land():
    """ドローンを着陸させる"""
    global is_flying
    if not is_flying:
        print("飛行していません")
        return
        
    # 着陸前にRCコマンドをリセット
    drone.send_rc_control(0, 0, 0, 0)
    
    print("ドローンを着陸させます...")
    try:
        drone.land()
        time.sleep(2)  # 安全のための待機
        is_flying = False
        print("ドローンが着陸しました")
    except Exception as e:
        print(f"ドローンの着陸に失敗しました: {e}")

def emergency_stop():
    """ドローンを緊急停止させる"""
    print("緊急停止します！")
    
    try:
        drone.emergency()
    except:
        pass
    
    global should_stop, is_flying
    is_flying = False
    should_stop = True

def get_keyboard_input():
    """キーボード入力を取得して移動値を返す"""
    lr, fb, ud, yv = 0, 0, 0, 0  # 左右, 前後, 上下, ヨー回転
    speed = SPEED
    
    # キー入力をチェック
    if kp.getKey("LEFT"):
        lr = -speed
    elif kp.getKey("RIGHT"):
        lr = speed
    
    if kp.getKey("UP"):
        fb = speed
    elif kp.getKey("DOWN"):
        fb = -speed
    
    if kp.getKey("w"):
        ud = speed
    elif kp.getKey("s"):
        ud = -speed
    
    if kp.getKey("a"):
        yv = -ROTATION_SPEED
    elif kp.getKey("d"):
        yv = ROTATION_SPEED
    
    # 離陸/着陸
    if kp.getKey("q"):
        takeoff()
    elif kp.getKey("e"):
        land()
    
    # 緊急停止
    if kp.getKey("ESCAPE"):
        emergency_stop()
    
    return [lr, fb, ud, yv]

def control_drone():
    """メインのドローン制御ループ"""
    global should_stop
    
    print("ドローン制御ループを開始します...")
    print("キー操作:")
    print("- Q: 離陸")
    print("- E: 着陸")
    print("- W/S: 上昇/下降")
    print("- A/D: 左右回転")
    print("- 矢印キー上/下: 前進/後退")
    print("- 矢印キー左/右: 左右移動")
    print("- ESC: 緊急停止")
    
    while not should_stop:
        # キーボード入力を取得
        vals = get_keyboard_input()
        
        # ドローンが飛行中の場合のみコマンドを送信
        if is_flying:
            drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])
        
        # 一定間隔で更新
        time.sleep(INTERVAL)

def cleanup():
    """リソースをクリーンアップする"""
    global tello_manager, drone, is_flying
    
    print("クリーンアップを開始します...")
    
    # ドローンが飛行中の場合は着陸
    if is_flying and drone:
        try:
            print("ドローンを着陸させています...")
            drone.land()
            time.sleep(2)  # 安全のための待機
        except:
            pass
    
    # ドローンとの接続を閉じる
    if drone:
        try:
            drone.end()
            print(f"ドローンとの接続を閉じました")
        except:
            pass
    
    # ソケットを閉じる
    if tello_manager:
        del tello_manager
    
    print("クリーンアップが完了しました")

def main():
    """メイン関数"""
    print("Tello通信テスト - Telloドローンの通信のみをテストするプログラム")
    print("-----------------------------------------------------------------------")
    
    # キーボード入力の初期化
    print("キーボード入力を初期化しています...")
    kp.init()
    
    # ドローンの初期化
    if not initialize_drone():
        print("ドローンの初期化に失敗しました。プログラムを終了します。")
        return
    
    try:
        # ドローン制御ループを開始
        control_drone()
    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        # クリーンアップ
        cleanup()

if __name__ == "__main__":
    main()
