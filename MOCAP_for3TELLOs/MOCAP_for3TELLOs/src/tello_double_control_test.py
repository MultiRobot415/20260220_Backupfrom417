#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tello Double Control Test - 2機のTelloドローンの接続確認テスト

このプログラムは、2機のTelloドローンを同時に検出・接続し、
キーボード操作によって基本的な制御機能（離陸・着陸・移動など）を確認します。

キー操作:
- Q: 離陸
- E: 着陸
- W/S: 上昇/下降
- A/D: 左右回転
- 矢印キー上/下: 前進/後退
- 矢印キー左/右: 左右移動
- ESC: 緊急停止
- SPACE: 正常終了

作成日: 2025-06-26
"""

import time
import threading
import sys
import os
import keyboard_control as kp
from custom_tello import CustomTello, TelloManager

# グローバル変数
tello_manager = None  # TelloManagerオブジェクト
drones = []  # CustomTelloオブジェクトのリスト
is_flying = False  # 飛行状態
should_stop = False  # プログラム終了フラグ

# 制御パラメータ（デフォルト値）
SPEED = 50  # 移動速度
ROTATION_SPEED = 50  # 回転速度
INTERVAL = 0.05  # コマンド送信間隔

def initialize_drones():
    """ドローンの初期化と接続"""
    global tello_manager, drones
    
    print("ドローンへの接続を開始します...")
    
    # TelloManagerインスタンスの生成
    print("TelloManagerを初期化しています...")
    tello_manager = TelloManager()
    
    # 2機のドローンを検索と接続
    print("2機のTelloドローンを接続します...")
    if not tello_manager.find_available_tello(2):
        print("ドローンの接続に失敗しました。プログラムを終了します。")
        return False
    
    # 検出されたドローンを取得
    drones = tello_manager.get_tello_list()
    
    if len(drones) < 2:
        print(f"警告: 2機のドローンが必要ですが、{len(drones)}機しか見つかりませんでした")
        if len(drones) == 0:
            print("ドローンが見つかりません。プログラムを終了します。")
            return False
    
    # 検出されたドローンに接続
    print("ドローンに接続しています...")
    for i, tello in enumerate(drones):
        try:
            battery = tello.get_battery()
            print(f"ドローン {i+1} (IP: {tello.tello_ip}) に接続しました。バッテリー残量: {battery}%")
            
            # バッテリー残量が低い場合は警告
            if battery is not None and battery < 20:
                print(f"警告: ドローン {i+1} のバッテリー残量が低下しています ({battery}%)")
        except Exception as e:
            print(f"ドローン {i+1} (IP: {tello.tello_ip}) への接続に失敗しました: {e}")
    
    print(f"{len(drones)}機のドローンに接続しました。")
    return True

def takeoff_all():
    """全てのドローンを同時に離陸させる"""
    global is_flying
    if is_flying:
        print("既に飛行中です")
        return
        
    print("全てのドローンを同時に離陸させます...")
    threads = []
    for i, tello in enumerate(drones):
        thread = threading.Thread(target=lambda t=tello, idx=i: takeoff_drone(t, idx))
        threads.append(thread)
        thread.start()
    
    for thread in threads:
        thread.join()
    
    # 離陸後にRCコマンドをリセット
    for tello in drones:
        tello.send_rc_control(0, 0, 0, 0)
    
    time.sleep(2)  # 安定化のための待機
    is_flying = True
    print("全てのドローンが離陸しました")

def takeoff_drone(tello, index):
    """個別のドローンを離陸させる（スレッド用）"""
    try:
        print(f"ドローン {index+1} を離陸させています...")
        tello.takeoff()
    except Exception as e:
        print(f"ドローン {index+1} の離陸に失敗しました: {e}")

def land_all():
    """全てのドローンを同時に着陸させる"""
    global is_flying
    if not is_flying:
        print("飛行していません")
        return
        
    # 着陸前にRCコマンドをリセット
    for tello in drones:
        tello.send_rc_control(0, 0, 0, 0)
    
    print("全てのドローンを同時に着陸させます...")
    threads = []
    for i, tello in enumerate(drones):
        thread = threading.Thread(target=lambda t=tello, idx=i: land_drone(t, idx))
        threads.append(thread)
        thread.start()
    
    for thread in threads:
        thread.join()
    
    time.sleep(2)  # 安全のための待機
    is_flying = False
    print("全てのドローンが着陸しました")

def land_drone(tello, index):
    """個別のドローンを着陸させる（スレッド用）"""
    try:
        print(f"ドローン {index+1} を着陸させています...")
        tello.land()
    except Exception as e:
        print(f"ドローン {index+1} の着陸に失敗しました: {e}")

def emergency_stop():
    """全てのドローンを緊急停止させる"""
    print("緊急停止します！")
    
    for i, tello in enumerate(drones):
        try:
            tello.emergency()
            print(f"ドローン {i+1} に緊急停止コマンドを送信しました")
        except Exception as e:
            print(f"ドローン {i+1} への緊急停止コマンド送信に失敗しました: {e}")
    
    global should_stop, is_flying
    is_flying = False
    should_stop = True

def get_keyboard_input():
    """キーボード入力を取得して移動値を返す"""
    lr, fb, ud, yv = 0, 0, 0, 0  # 左右, 前後, 上下, ヨー回転
    speed = SPEED
    
    # キー入力をチェック
    pressed_keys = kp.get_pressed_keys()
    
    # 押されたキーがあればコンソールに表示
    if pressed_keys:
        print(f"キー入力検出: {', '.join(pressed_keys)}")
    
    # 左右移動 (左右キー)
    if "LEFT" in pressed_keys: lr = -speed
    elif "RIGHT" in pressed_keys: lr = speed
    
    # 前後移動 (上下キー)
    if "UP" in pressed_keys: fb = speed
    elif "DOWN" in pressed_keys: fb = -speed
    
    # 上下移動 (w/s)
    if "w" in pressed_keys: ud = speed
    elif "s" in pressed_keys: ud = -speed
    
    # 左右回転 (a/d)
    if "a" in pressed_keys: yv = -ROTATION_SPEED
    elif "d" in pressed_keys: yv = ROTATION_SPEED
    
    # 離陸と着陸
    if "q" in pressed_keys: takeoff_all()
    if "e" in pressed_keys: land_all()
    
    # 緊急停止
    if "ESCAPE" in pressed_keys: emergency_stop()
    
    # 通常終了
    if "SPACE" in pressed_keys:
        global should_stop
        should_stop = True
        print("SPACEキーが押されました。プログラムを正常終了します。")
    
    return [lr, fb, ud, yv]

def control_drones():
    """メインのドローン制御ループ"""
    print("ドローン制御を開始します...")
    print("Pygameウィンドウにフォーカスを当ててキーボード操作してください")
    
    last_key_press_time = time.time()
    last_values = [0, 0, 0, 0]
    
    while not should_stop:
        # キーボード入力を取得
        vals = get_keyboard_input()
        
        # 値が変化した場合のみ表示
        if vals != last_values:
            print(f"制御値: LR={vals[0]}, FB={vals[1]}, UD={vals[2]}, YV={vals[3]}")
            last_values = vals.copy()
        
        # 飛行中の場合のみコマンドを送信
        if is_flying:
            # 値が0でない場合のみコマンドを送信（バッテリー節約）
            if any(vals):
                for i, tello in enumerate(drones):
                    try:
                        tello.send_rc_control(vals[0], vals[1], vals[2], vals[3])
                    except Exception as e:
                        print(f"ドローン {i+1} へのコマンド送信に失敗しました: {e}")
            
            # 一定時間操作がない場合は、ホバリングコマンドを送信
            if time.time() - last_key_press_time > 10 and all(v == 0 for v in vals):
                for i, tello in enumerate(drones):
                    try:
                        # ホバリングコマンド
                        tello.send_rc_control(0, 0, 0, 0)
                    except Exception as e:
                        print(f"ドローン {i+1} へのホバリングコマンド送信に失敗しました: {e}")
        
        # キー入力があれば時間を更新
        if any(vals):
            last_key_press_time = time.time()
        
        # 一定間隔で実行
        time.sleep(INTERVAL)

def cleanup():
    """リソースをクリーンアップする"""
    global drones
    
    print("クリーンアップを実行しています...")
    
    # 飛行中の場合は着陸させる
    if is_flying:
        land_all()
    
    # ドローンとの接続を閉じる
    if tello_manager:
        tello_manager.shutdown()
    
    # Pygameを終了
    kp.quit()
    
    print("クリーンアップ完了")

def main():
    """メイン関数"""
    print("=== Tello Double Control Test ===")
    print("2機のTelloドローン接続確認テスト")
    print("")
    print("キー操作:")
    print("- Q: 離陸")
    print("- E: 着陸")
    print("- W/S: 上昇/下降")
    print("- A/D: 左右回転")
    print("- 矢印キー上/下: 前進/後退")
    print("- 矢印キー左/右: 左右移動")
    print("- ESC: 緊急停止")
    print("- SPACE: 正常終了")
    print("")
    
    # キーボードモジュールの初期化
    kp.init()
    
    try:
        # ドローンの初期化
        if initialize_drones():
            # ドローン制御ループを開始
            control_drones()
    except KeyboardInterrupt:
        print("\nプログラムが中断されました")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        # 終了処理
        cleanup()
        print("プログラムを終了します")

if __name__ == "__main__":
    main()
