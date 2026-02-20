"""
Tello Manager - Telloドローンの管理クラス

このモジュールは、Telloドローンを管理し、通信を行うためのクラスを提供します。
"""

import socket
import threading
import time
from djitellopy import Tello

class TelloManager:
    """
    Telloドローンを管理するクラス
    """
    def __init__(self):
        """
        TelloManagerの初期化
        """
        self.tello_list = []  # Telloオブジェクトのリスト
        
    def find_available_tello(self, num_drones=1):
        """
        利用可能なTelloドローンを検索する
        :param num_drones: 検索するTelloドローンの数
        """
        print(f"[開始] {num_drones}台のTelloドローンを検索しています...")
        
        # djitellopyのTelloクラスを使用
        tello = Tello()
        
        # 接続試行回数と間隔を設定
        max_retries = 3
        retry_interval = 2  # 秒
        
        for attempt in range(1, max_retries + 1):
            try:
                print(f"Telloドローンへの接続を試行しています (試行 {attempt}/{max_retries})...")
                # 接続テスト
                tello.connect()
                print(f"Telloドローンを検出しました")
                
                # 接続確認のためにコマンドモードを再確認
                try:
                    tello.send_command_with_return("command", timeout=5)
                    print("Telloドローンとの通信を確認しました")
                except Exception as cmd_err:
                    print(f"Telloドローンとの通信確認に失敗しました: {cmd_err}")
                    if attempt < max_retries:
                        print(f"{retry_interval}秒後に再試行します...")
                        time.sleep(retry_interval)
                        continue
                    raise
                
                self.tello_list.append(tello)
                break  # 接続成功した場合はループを抜ける
                
            except Exception as e:
                print(f"Telloドローンの検出に失敗しました: {e}")
                if attempt < max_retries:
                    print(f"{retry_interval}秒後に再試行します...")
                    time.sleep(retry_interval)
                else:
                    print(f"Telloドローンへの接続を{max_retries}回試行しましたが、失敗しました。")
                    print("ネットワーク接続を確認してください。")
                    print("TelloドローンのWi-Fiに接続されているか確認してください。")
        
        # 2機目のコメントアウト（将来的に使用する可能性あり）
        """
        # 2機目のTelloドローン
        if num_drones > 1:
            try:
                tello2 = Tello()
                tello2.connect()
                print(f"2機目のTelloドローンを検出しました")
                self.tello_list.append(tello2)
            except Exception as e:
                print(f"2機目のTelloドローンの検出に失敗しました: {e}")
        """
        
        print(f"検出されたTelloドローン: {len(self.tello_list)}台")
        
    def get_tello_list(self):
        """
        Telloドローンのリストを取得する
        :return: Telloオブジェクトのリスト
        """
        return self.tello_list
        
    def close_connections(self):
        """
        すべてのTelloドローンとの接続を閉じる
        """
        for tello in self.tello_list:
            try:
                tello.end()
            except:
                pass
        
        self.tello_list = []
        print("すべてのTelloドローンとの接続を閉じました")
