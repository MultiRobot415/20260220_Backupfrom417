#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3機のTello EDUドローン手動制御プログラム

3機のTello EDUドローンをキーボードで手動制御するシンプルなプログラム。
MOCAPシステムは使用せず、完全手動操作のみを提供します。

作成日: 2025-09-30
"""

import time
import sys
import os
import threading

# カスタムモジュールをインポート
import keyboard_control as kc
from custom_tello import TelloManager

# プログラム設定
NUM_DRONES = 3  # ドローン数
SPEED = 20  # 移動速度（0-100）
ROTATION_SPEED = 50  # 回転速度（0-100）

class ManualControl3Tellos:
    """
    3機のTelloドローンを手動制御するクラス
    """
    def __init__(self):
        """
        初期化
        """
        print("=" * 60)
        print("3機のTello EDUドローン手動制御プログラム")
        print("=" * 60)
        
        # ドローンマネージャーの初期化
        self.manager = TelloManager()
        
        # ドローンの検出と接続
        print(f"\n{NUM_DRONES}機のドローンに接続しています...")
        self.manager.find_available_tello(NUM_DRONES)
        
        # ドローンリストの取得
        self.tellos = self.manager.get_tello_list()
        
        if len(self.tellos) == 0:
            print("エラー: ドローンが見つかりませんでした。")
            print("ドローンのIPアドレスを確認してください。")
            sys.exit(1)
        
        print(f"\n接続成功: {len(self.tellos)}機のドローンが利用可能です")
        
        # 選択されたドローンのインデックス（0=全て、1=1号機、2=2号機、3=3号機）
        self.selected_drone = 0  # デフォルトは全機
        
        # プログラム実行フラグ
        self.running = True
        
        # キーボード入力の初期化
        print("\nキーボード入力システムを初期化しています...")
        kc.init()
        time.sleep(1)  # 初期化待機
        
        print("\n初期化完了！")
        self._print_controls()
    
    def _print_controls(self):
        """
        操作方法を表示する
        """
        print("\n" + "=" * 60)
        print("操作方法")
        print("=" * 60)
        print("【基本操作】")
        print("  Q: 全ドローン同時離陸")
        print("  E: 全ドローン同時着陸")
        print("  ESC: 緊急停止")
        print("  SPACE: プログラム終了")
        print("\n【ドローン選択】")
        print("  1: ドローン1を選択")
        print("  2: ドローン2を選択")
        print("  3: ドローン3を選択")
        print("  0: 全ドローンを選択（デフォルト）")
        print("\n【手動操作】（選択されたドローンのみ）")
        print("  矢印キー（↑↓←→）: 前後左右移動")
        print("  W/S: 上昇/下降")
        print("  A/D: 左回転/右回転")
        print("=" * 60)
        print("\n※ Pygameウィンドウをクリックしてアクティブにしてください")
        print()
    
    def get_selected_drones(self):
        """
        選択されたドローンのリストを返す
        
        Returns:
            list: 選択されたドローンのリスト
        """
        if self.selected_drone == 0:
            # 全ドローンを返す
            return self.tellos
        else:
            # 特定のドローンを返す（1-indexed → 0-indexed）
            idx = self.selected_drone - 1
            if 0 <= idx < len(self.tellos):
                return [self.tellos[idx]]
            else:
                return []
    
    def handle_drone_selection(self):
        """
        ドローン選択キーを処理する
        """
        if kc.getKey('0'):
            if self.selected_drone != 0:
                self.selected_drone = 0
                print(f"✓ 全ドローンを選択しました")
                time.sleep(0.2)  # デバウンス
        elif kc.getKey('1'):
            if self.selected_drone != 1:
                self.selected_drone = 1
                print(f"✓ ドローン1を選択しました")
                time.sleep(0.2)
        elif kc.getKey('2'):
            if self.selected_drone != 2:
                self.selected_drone = 2
                print(f"✓ ドローン2を選択しました")
                time.sleep(0.2)
        elif kc.getKey('3'):
            if self.selected_drone != 3:
                self.selected_drone = 3
                print(f"✓ ドローン3を選択しました")
                time.sleep(0.2)
    
    def handle_takeoff_land(self):
        """
        離陸・着陸キーを処理する
        """
        # 離陸（全機同時）
        if kc.getKey('q'):
            print("\n全ドローンが離陸します...")
            
            # 並列処理用のスレッドリスト
            threads = []
            
            def takeoff_thread(tello, index):
                if not tello.is_flying:
                    print(f"  ドローン{index+1}: 離陸中...")
                    tello.takeoff()
            
            # 全ドローンの離陸を並列で実行
            for i, tello in enumerate(self.tellos):
                thread = threading.Thread(target=takeoff_thread, args=(tello, i))
                threads.append(thread)
                thread.start()
            
            # 全スレッドの完了を待つ
            for thread in threads:
                thread.join()
            
            print("離陸完了！")
            time.sleep(1)  # 離陸後の安定待機
        
        # 着陸（全機同時）
        if kc.getKey('e'):
            print("\n全ドローンが着陸します...")
            
            # 並列処理用のスレッドリスト
            threads = []
            
            def land_thread(tello):
                if tello.is_flying:
                    tello.land()
            
            # 全ドローンの着陸を並列で実行
            for tello in self.tellos:
                thread = threading.Thread(target=land_thread, args=(tello,))
                threads.append(thread)
                thread.start()
            
            # 全スレッドの完了を待つ
            for thread in threads:
                thread.join()
            
            print("着陸完了！")
            time.sleep(1)
    
    def handle_manual_control(self):
        """
        手動制御キーを処理する
        """
        # 選択されたドローンを取得
        selected_drones = self.get_selected_drones()
        
        if len(selected_drones) == 0:
            return
        
        # 制御値の初期化
        lr = 0  # 左右（left/right）
        fb = 0  # 前後（forward/backward）
        ud = 0  # 上下（up/down）
        yaw = 0  # ヨー回転
        
        # キー入力に応じて制御値を設定
        if kc.getKey('LEFT'):
            lr = -SPEED
        elif kc.getKey('RIGHT'):
            lr = SPEED
        
        if kc.getKey('UP'):
            fb = SPEED
        elif kc.getKey('DOWN'):
            fb = -SPEED
        
        if kc.getKey('w'):
            ud = SPEED
        elif kc.getKey('s'):
            ud = -SPEED
        
        if kc.getKey('a'):
            yaw = -ROTATION_SPEED
        elif kc.getKey('d'):
            yaw = ROTATION_SPEED
        
        # 選択されたドローンに制御値を送信
        for tello in selected_drones:
            if tello.is_flying:
                tello.send_rc_control(lr, fb, ud, yaw)
        
        # 選択されていないドローンにもホバリングコマンドを送信（15秒自動着陸防止）
        non_selected_drones = [tello for tello in self.tellos if tello not in selected_drones]
        for tello in non_selected_drones:
            if tello.is_flying:
                tello.send_rc_control(0, 0, 0, 0)
    
    def handle_emergency_stop(self):
        """
        緊急停止キーを処理する
        """
        if kc.getKey('ESCAPE'):
            print("\n!!! 緊急停止 !!!")
            for i, tello in enumerate(self.tellos):
                print(f"  ドローン{i+1}: 緊急停止中...")
                tello.emergency()
            self.running = False
            return True
        return False
    
    def handle_normal_exit(self):
        """
        通常終了キーを処理する
        """
        if kc.getKey('SPACE'):
            print("\nプログラムを終了します...")
            
            # 飛行中のドローンがあれば着陸
            flying_drones = [tello for tello in self.tellos if tello.is_flying]
            if len(flying_drones) > 0:
                print("飛行中のドローンを着陸させています...")
                for i, tello in enumerate(flying_drones):
                    print(f"  ドローン着陸中...")
                    tello.land()
                time.sleep(2)
            
            self.running = False
            return True
        return False
    
    def display_status(self):
        """
        定期的にドローンの状態を表示する
        """
        # 5秒ごとに表示
        if not hasattr(self, '_last_status_time'):
            self._last_status_time = time.time()
        
        if time.time() - self._last_status_time >= 5.0:
            print("\n--- ドローン状態 ---")
            for i, tello in enumerate(self.tellos):
                battery = tello.get_battery()
                status = "飛行中" if tello.is_flying else "待機中"
                selected = "★" if (self.selected_drone == 0 or self.selected_drone == i+1) else " "
                print(f"{selected} ドローン{i+1}: {status}, バッテリー: {battery}%")
            print(f"選択: {'全機' if self.selected_drone == 0 else f'ドローン{self.selected_drone}'}")
            print("-------------------\n")
            self._last_status_time = time.time()
    
    def run(self):
        """
        メインループ
        """
        print("\nメインループ開始！")
        print("Pygameウィンドウをクリックしてキーボード入力を有効にしてください。\n")
        
        try:
            while self.running:
                # ドローン選択処理
                self.handle_drone_selection()
                
                # 離陸・着陸処理
                self.handle_takeoff_land()
                
                # 手動制御処理
                self.handle_manual_control()
                
                # 緊急停止処理
                if self.handle_emergency_stop():
                    break
                
                # 通常終了処理
                if self.handle_normal_exit():
                    break
                
                # 状態表示
                self.display_status()
                
                # ループ間隔
                time.sleep(0.05)  # 20Hz
        
        except KeyboardInterrupt:
            print("\n\nキーボード割り込み検出")
            print("プログラムを終了します...")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """
        終了処理
        """
        print("\n終了処理を実行しています...")
        
        # 全ドローンを停止
        print("全ドローンにホバリングコマンドを送信...")
        for tello in self.tellos:
            if tello.is_flying:
                tello.send_rc_control(0, 0, 0, 0)
        
        time.sleep(0.5)
        
        # マネージャーのシャットダウン
        print("ドローンマネージャーをシャットダウン...")
        self.manager.shutdown()
        
        # キーボード入力システムの終了
        print("キーボード入力システムをシャットダウン...")
        kc.quit()
        
        print("\n終了処理完了")
        print("=" * 60)
        print("プログラムを終了しました")
        print("=" * 60)


def main():
    """
    メイン関数
    """
    try:
        # 制御システムの初期化
        control_system = ManualControl3Tellos()
        
        # メインループの実行
        control_system.run()
    
    except Exception as e:
        print(f"\nエラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
