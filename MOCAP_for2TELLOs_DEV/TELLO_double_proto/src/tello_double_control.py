# -*- coding: utf-8 -*-
"""
Tello Double Control - 2機のTelloドローンを同時にキーボード制御するプログラム

このプログラムは、ステーションモードで接続された2機のTelloドローンを
キーボード操作によって同時に制御します。

キー操作:
- Q: 離陸
- E: 着陸
- W/S: 上昇/下降
- A/D: 左右回転
- 矢印キー上/下: 前進/後退
- 矢印キー左/右: 左右移動
- ESC: 緊急停止

作成日: 2025-05-01
"""

import KeyPressModule as kp
import time
import cv2
import threading
import numpy as np
import os
# カスタムTelloクラスをインポート
from custom_tello import CustomTello, TelloManager

# グローバル変数
tello_manager = None  # TelloManagerオブジェクト
drones = []  # CustomTelloオブジェクトのリスト
is_flying = False  # 飛行状態
should_stop = False  # プログラム終了フラグ

# 制御パラメータ（デフォルト値）
SPEED = 30
ROTATION_SPEED = 50
INTERVAL = 0.05

# キー入力状態を保持する変数
last_key_release_time = 0  # 最後にキーが離された時間
key_pressed = False  # キーが押されているかどうか

def initialize_drones():
    """ドローンの初期化と接続"""
    global tello_manager, drones
    
    print("ドローンへの接続を開始します...")
    
    # TelloManagerを初期化
    print("TelloManagerを初期化しています...")
    tello_manager = TelloManager()
    
    # ドローンを検索（理想は2機だが、1機でも動作可能）
    target_drones = 2
    min_drones = 1  # 最低1機あれば動作可能
    print(f"Telloドローンを検索しています...（理想: {target_drones}機、最低: {min_drones}機）")
    tello_manager.find_available_tello(target_drones, timeout=15)  # 15秒のタイムアウトを設定
    
    # 検出されたドローンを取得
    drones = tello_manager.get_tello_list()
    
    if len(drones) < min_drones:
        print(f"エラー: 少なくとも{min_drones}機のドローンが必要ですが、{len(drones)}機しか見つかりませんでした")
        print("ドローンが見つかりません。プログラムを終了します。")
        return False
    elif len(drones) < target_drones:
        print(f"警告: {target_drones}機のドローンが理想ですが、{len(drones)}機しか見つかりませんでした")
        print(f"{len(drones)}機のドローンで続行します")
    
    # 検出されたドローンに接続
    print("ドローンに接続しています...")
    for i, tello in enumerate(drones):
        try:
            tello.connect()
            battery = tello.get_battery()
            print(f"ドローン {i+1} (IP: {tello.tello_ip}) に接続しました。バッテリー残量: {battery}%")
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
        except:
            pass
    
    global should_stop, is_flying
    is_flying = False
    should_stop = True

def get_keyboard_input():
    """キーボード入力を取得して移動値を返す"""
    global key_pressed, last_key_release_time, should_stop
    
    lr, fb, ud, yv = 0, 0, 0, 0  # 左右, 前後, 上下, ヨー回転
    speed = SPEED
    
    # キー入力をチェックして、押されたキーがあればコンソールに表示
    keys_to_check = ["a", "d", "w", "s", "UP", "DOWN", "LEFT", "RIGHT", "q", "e", "ESCAPE", "SPACE"]
    pressed_keys = []
    
    for key in keys_to_check:
        if kp.getKey(key):
            pressed_keys.append(key)
            print(f"キー入力検出: {key}")
    
    # スペースキーでプログラム終了
    if "SPACE" in pressed_keys:
        print("スペースキーが押されました - プログラムを終了します")
        should_stop = True
    
    # キーが押されているかどうかを記録
    if pressed_keys:
        key_pressed = True
    else:
        # キーが離された瞬間を検出
        if key_pressed:
            last_key_release_time = time.time()
            print("キーが離されました - ホバリングモードに移行")
        key_pressed = False
    
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
    
    return [lr, fb, ud, yv]

def control_drones():
    """メインのドローン制御ループ"""
    print("ドローン制御を開始します...")
    print("Pygameウィンドウにフォーカスを当ててキーボード操作してください")
    
    last_key_press_time = time.time()
    last_values = [0, 0, 0, 0]
    hover_sent = False  # ホバリングコマンドが送信されたかどうか
    
    while not should_stop:
        # キーボード入力を取得
        vals = get_keyboard_input()
        
        # 値が変化した場合のみ表示
        if vals != last_values:
            print(f"制御値: LR={vals[0]}, FB={vals[1]}, UD={vals[2]}, YV={vals[3]}")
            last_values = vals.copy()
        
        # 飛行中の場合のみコマンドを送信
        if is_flying:
            # キーが押されている場合
            if any(vals):
                hover_sent = False  # ホバリングフラグをリセット
                for i, tello in enumerate(drones):
                    try:
                        tello.send_rc_control(vals[0], vals[1], vals[2], vals[3])
                        print(f"ドローン {i+1} にコマンド送信: LR={vals[0]}, FB={vals[1]}, UD={vals[2]}, YV={vals[3]}")
                    except Exception as e:
                        print(f"ドローン {i+1} へのコマンド送信に失敗しました: {e}")
                last_key_press_time = time.time()
            
            # キーが離された直後にホバリングコマンドを送信
            elif not key_pressed and time.time() - last_key_release_time < 0.5 and not hover_sent:
                print("キーが離されたため、ホバリングコマンドを送信します")
                for i, tello in enumerate(drones):
                    try:
                        # ホバリングコマンド
                        tello.send_rc_control(0, 0, 0, 0)
                        print(f"ドローン {i+1} にホバリングコマンド送信")
                    except Exception as e:
                        print(f"ドローン {i+1} へのホバリングコマンド送信に失敗しました: {e}")
                hover_sent = True
            
            # 一定時間操作がない場合は、定期的にホバリングコマンドを送信（安全対策）
            elif time.time() - last_key_press_time > 3 and not hover_sent:
                for i, tello in enumerate(drones):
                    try:
                        # ホバリングコマンド
                        tello.send_rc_control(0, 0, 0, 0)
                        print(f"ドローン {i+1} に定期ホバリングコマンド送信")
                    except Exception as e:
                        print(f"ドローン {i+1} へのホバリングコマンド送信に失敗しました: {e}")
                hover_sent = True
        
        # 一定間隔で実行
        time.sleep(INTERVAL)

def cleanup():
    """リソースをクリーンアップする"""
    global drones, tello_manager
    
    print("クリーンアップを実行しています...")
    
    # 飛行中の場合は着陸させる
    if is_flying:
        try:
            print("安全のため、すべてのドローンを着陸させています...")
            land_all()
            # 着陸後に少し待機して安定させる
            time.sleep(2)
        except Exception as e:
            print(f"着陸中にエラーが発生しました: {e}")
            print("緊急停止を試みます...")
            try:
                emergency_stop()
            except:
                pass
    
    # すべてのドローンにホバリングコマンドを送信（安全対策）
    try:
        for i, tello in enumerate(drones):
            try:
                tello.send_rc_control(0, 0, 0, 0)
                print(f"ドローン {i+1} に最終ホバリングコマンドを送信しました")
            except:
                pass
    except:
        pass
    
    # ドローンとの接続を閉じる
    for i, tello in enumerate(drones):
        try:
            tello.end()
            print(f"ドローン {i+1} との接続を閉じました")
        except Exception as e:
            print(f"ドローン {i+1} との接続を閉じる際にエラーが発生しました: {e}")
    
    # TelloManagerのソケットを閉じる
    if tello_manager:
        try:
            tello_manager.socket.close()
            print("TelloManagerのソケットを閉じました")
        except Exception as e:
            print(f"ソケットを閉じる際にエラーが発生しました: {e}")
    
    # リストをクリア
    drones = []
    tello_manager = None
    print("クリーンアップ完了")

def main():
    """メイン関数"""
    global should_stop
    
    print("Tello Double Control - 2機のTelloドローン同時制御プログラム")
    print("キー操作:")
    print("- Q: 離陸")
    print("- E: 着陸")
    print("- W/S: 上昇/下降")
    print("- A/D: 左右回転")
    print("- 矢印キー上/下: 前進/後退")
    print("- 矢印キー左/右: 左右移動")
    print("- ESC: 緊急停止")
    print("- SPACE: プログラム終了")
    print("\n※ キーを離すと自動的にホバリングします")
    
    # キーボードモジュールの初期化
    kp.init()
    
    # ドローンの初期化
    if not initialize_drones():
        return
    
    # 終了時の処理を登録（Ctrl+Cなどで強制終了された場合にも対応）
    import atexit
    atexit.register(cleanup)
    
    try:
        # ドローン制御ループを開始
        control_drones()
    except KeyboardInterrupt:
        print("\nプログラムが中断されました")
        should_stop = True
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        should_stop = True
    finally:
        # 終了処理
        cleanup()
        print("プログラムを終了します")

if __name__ == "__main__":
    main()
