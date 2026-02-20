#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MOCAP Streaming Test - モーションキャプチャデータの受信テスト

このプログラムは、モーションキャプチャシステムからの位置データを受信して表示し、
正常にデータを取得できるかを確認するためのテストプログラムです。
実際のドローンには接続せず、MOCAPデータのみを扱います。

キー操作:
- ESC: 終了

作成日: 2025-06-26
"""

import time
import threading
import sys
import os
import json
import datetime
import keyboard_control as kp
import mocap_stream as ms

# グローバル変数
should_stop = False  # プログラム終了フラグ
rigid_body_ids = [1, 2]  # 追跡対象のリジッドボディID
data_refresh_rate = 0.1  # データ更新間隔（秒）
result_dir = "results"  # 結果保存ディレクトリ

# IDごとの位置データ保存用
position_logs = {rb_id: [] for rb_id in rigid_body_ids}
last_rb_positions = {rb_id: None for rb_id in rigid_body_ids}
last_save_time = time.time()
save_interval = 10.0  # データ保存間隔（秒）

def save_position_logs():
    """
    記録された位置データをファイルに保存
    """
    global position_logs
    
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # IDごとにデータを保存
    for rb_id, logs in position_logs.items():
        if not logs:  # データがない場合はスキップ
            continue
            
        filename = f"{result_dir}/mocap_data_rb{rb_id}_{timestamp}.json"
        
        try:
            with open(filename, 'w') as f:
                json.dump(logs, f, indent=2)
            print(f"RigidBody {rb_id} のデータを {filename} に保存しました ({len(logs)} レコード)")
            # 保存後はクリア
            position_logs[rb_id] = []
        except Exception as e:
            print(f"データ保存中にエラーが発生しました: {e}")


def display_mocap_data_thread():
    """
    モーションキャプチャデータを表示し、IDごとに記録するスレッド。
    各RigidBodyの受信回数をカウントし、1秒ごとにサマリを出力することで
    データ有無を一目で把握できるようにした。
    さらに、各リジッドボディの位置データをリアルタイムで表示し記録する。
    """
    global should_stop, position_logs, last_rb_positions, last_save_time
    packet_counts = {rb: 0 for rb in rigid_body_ids}
    last_summary_time = time.time()
    print("モーションキャプチャデータの表示と記録を開始します...")
    print(f"データは{save_interval}秒ごとに {result_dir} ディレクトリに保存されます")
    
    # 現在追跡中のRigidBody IDを取得
    tracked_ids = ms.get_tracked_rigid_body_ids()
    if tracked_ids:
        print(f"追跡中のRigidBody ID: {tracked_ids}")
    
    # 追跡するリジッドボディIDリスト
    print(f"ID {rigid_body_ids} のデータを同時に表示します")
    
    while not should_stop:
        current_time = time.time()
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # 各リジッドボディIDに対してデータを取得して表示する
        display_results = []
        
        # まず各IDごとにデータを取得して一時録
        for rb_id in rigid_body_ids:
            try:
                # 新しいAPIを使って指定したIDの位置データを取得
                pos = ms.get_current_position(rigid_body_id=rb_id)
                        
                if pos is not None:
                    # 取得結果を一時保存
                    display_results.append((rb_id, pos))
                    
                    # 受信カウント増加
                    packet_counts[rb_id] += 1
                    
                    # 位置データを記録
                    position_logs[rb_id].append({
                        "timestamp": timestamp,
                        "rigid_body_id": rb_id,
                        "position": {"x": pos['x'], "y": pos['y'], "z": pos['z']}
                    })
                    
                    # 最後の位置を更新
                    last_rb_positions[rb_id] = pos
            except Exception as e:
                # エラーが発生した場合はスキップ
                pass
        
        # データ表示をサマリーのみにする
        # 取得した全ての結果をまとめて表示
        # for rb_id, pos in display_results:
        #     print(f"Time: {timestamp} - RigidBody {rb_id} Data: X={pos['x']:.3f}, Y={pos['y']:.3f}, Z={pos['z']:.3f}")
        
        # 1 秒ごとにサマリー表示
        if current_time - last_summary_time >= 1.0:
            # 位置データのサマリー表示
            position_summary = ""
            for rb_id, pos in display_results:
                position_summary += f"Time: {timestamp} - RigidBody {rb_id} Data: X={pos['x']:.3f}, Y={pos['y']:.3f}, Z={pos['z']:.3f}\n"
            print(position_summary, end="")
            
            # パケットカウントサマリー
            # summary = " | ".join([f"RB{rb}: {packet_counts[rb]} pkt" for rb in rigid_body_ids])
            # print(f"[1s Summary] {summary}")
            
            # リセット
            packet_counts = {rb: 0 for rb in rigid_body_ids}
            last_summary_time = current_time
        
        # 一定間隔でデータを保存
        if current_time - last_save_time >= save_interval:
            save_position_logs()
            last_save_time = current_time
        
        time.sleep(data_refresh_rate)

def monitor_status_thread():
    """
    MOCAPシステムの接続状態を監視するスレッド
    """
    global should_stop
    while not should_stop:
        try:
            # 1秒ごとに接続状態をチェック
            time.sleep(1.0)
            
            # 接続状態を取得
            status = ms.get_connection_status()
            
            # パケット受信状況を出力
            if status["connected"]:
                current_packets = status["packets"]
                diff_packets = current_packets - last_stats["packets"]
                # print(f"MOCAP: パケット受信 (+{diff_packets}, 合計: {current_packets})")
                
                # 統計情報を更新
                last_stats["packets"] = current_packets
            else:
                # print("MOCAP: 接続状態が不安定です")
                pass
            
        except Exception as e:
            # print(f"Error in monitor thread: {e}")
            pass

def check_keyboard_input():
    """
    キーボード入力をチェックしてプログラム制御する
    """
    global should_stop
    
    while not should_stop:
        # キーボード入力を取得
        pressed_keys = kp.get_pressed_keys()
        
        # ESCキーで終了
        if "ESCAPE" in pressed_keys:
            print("\nESCキーが押されました。プログラムを終了します。")
            should_stop = True
            break
        
        # 一定間隔で実行
        time.sleep(0.1)

def main():
    """
    メイン関数
    """
    global should_stop
    
    # 結果保存ディレクトリの確認
    os.makedirs(result_dir, exist_ok=True)
    
    print("=== MOCAP Streaming Test ===")
    print(f"モーションキャプチャデータ受信・記録テスト (データ保存先: {result_dir})")
    print("")
    print("キー操作:")
    print("- ESC: 終了")
    print("")
    
    # キーボードモジュールの初期化
    kp.init()
    
    try:
        # MOCAPシステムに接続
        print("モーションキャプチャシステムに接続しています...")
        
        # 異なるリジッドボディIDでの接続を試みる
        # デフォルトIPアドレスとポート: 192.168.11.2:1511
        # NatNetクライアントを初期化
        print("モーションキャプチャシステムに接続しています...")
        # デバッグレベルを0にしてログ出力を最小化
        if not ms.initialize(rigid_body_id=None, debug_level=0):
            print("モーションキャプチャシステムへの接続に失敗しました")
            return 1
        
        # 接続状態を確認
        status = ms.get_connection_status()
        if not status["connected"]:
            print("モーションキャプチャシステムに接続できませんでした。")
            print("ネットワーク設定を確認してください。")
            return
        
        print("モーションキャプチャシステムに接続しました。")
        print(f"リジッドボディ {', '.join(map(str, rigid_body_ids))} のデータを監視します。")
        
        # データ表示スレッドを開始
        data_thread = threading.Thread(target=display_mocap_data_thread)
        data_thread.daemon = True
        data_thread.start()
        
        # 状態監視スレッドを開始
        status_thread = threading.Thread(target=monitor_status_thread)
        status_thread.daemon = True
        status_thread.start()
        
        # キーボード入力チェックを開始
        check_keyboard_input()
        
    except KeyboardInterrupt:
        print("\nプログラムが中断されました")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        # 終了処理
        should_stop = True
        
        # 最後のデータを保存
        save_position_logs()
        
        # MOCAPシステムとの接続を閉じる
        ms.shutdown()
        
        # Pygameを終了
        kp.quit()
        
        print("プログラムを終了します")

if __name__ == "__main__":
    main()
