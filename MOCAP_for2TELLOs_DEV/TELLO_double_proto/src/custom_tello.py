# -*- coding: utf-8 -*-
"""
カスタムTelloクラス - 複数のTelloドローンを制御するためのクラス

このモジュールは、複数のTelloドローンを同時に制御するためのカスタムTelloクラスを提供します。
DJITelloPyをもとに独自開発を行った。複数制御を簡易に行える点がポイントである。
"""

import socket
import threading
import time
from collections import defaultdict

class CustomTello:
    """
    カスタムTelloクラス - 複数のTelloドローンを制御するためのクラス
    """
    def __init__(self, tello_ip, manager):
        """
        初期化
        :param tello_ip: TelloドローンのIPアドレス
        :param manager: TelloManagerオブジェクト
        """
        self.tello_ip = tello_ip
        self.manager = manager
        self.is_flying = False
        
    def connect(self):
        """
        ドローンに接続する
        """
        return self.send_command("command")
        
    def takeoff(self):
        """
        離陸する
        """
        result = self.send_command("takeoff")
        if result:
            self.is_flying = True
        return result
        
    def land(self):
        """
        着陸する
        """
        result = self.send_command("land")
        if result:
            self.is_flying = False
        return result
        
    def emergency(self):
        """
        緊急停止する
        """
        result = self.send_command("emergency")
        if result:
            self.is_flying = False
        return result
        
    def get_battery(self):
        """
        バッテリー残量を取得する
        :return: バッテリー残量（%）
        """
        response = self.send_command("battery?")
        try:
            return int(response)
        except:
            return 0
            
    def send_rc_control(self, left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity):
        """
        RCコントロールコマンドを送信する
        :param left_right_velocity: 左右速度（-100〜100）
        :param forward_backward_velocity: 前後速度（-100〜100）
        :param up_down_velocity: 上下速度（-100〜100）
        :param yaw_velocity: ヨー回転速度（-100〜100）
        """
        # 値を-100〜100の範囲に制限
        def clamp(value):
            return max(-100, min(100, value))
            
        left_right_velocity = clamp(left_right_velocity)
        forward_backward_velocity = clamp(forward_backward_velocity)
        up_down_velocity = clamp(up_down_velocity)
        yaw_velocity = clamp(yaw_velocity)
        
        # RCコマンドを送信（レスポンスを待たない）
        cmd = f"rc {left_right_velocity} {forward_backward_velocity} {up_down_velocity} {yaw_velocity}"
        print(f"RCコマンド送信: {cmd} -> {self.tello_ip}")
        self.manager.socket.sendto(cmd.encode('utf-8'), (self.tello_ip, 8889))
        return True
        
    def send_command(self, command):
        """
        コマンドを送信する
        :param command: 送信するコマンド
        :return: レスポンス
        """
        return self.manager.send_command(command, self.tello_ip)
        
    def end(self):
        """
        終了処理
        """
        if self.is_flying:
            self.land()


class TelloManager:
    """
    複数のTelloドローンを管理するクラス
    """
    def __init__(self):
        """
        初期化
        """
        # ソケット設定
        self.local_ip = ''
        self.local_port = 8889
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # コマンド送信用ソケット
        
        # ソケットオプションを設定して、ポートの再利用を許可
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            # Linux/Macでのみ使用可能なオプション
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            # Windowsではこのオプションは使用できない
            pass
        
        # ソケットのタイムアウトを設定
        self.socket.settimeout(5.0)
        
        # 既存のソケットを閉じる試み（ポートの解放）
        try:
            temp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            temp_socket.bind((self.local_ip, self.local_port))
            temp_socket.close()
            print(f"ポート{self.local_port}を解放しました")
        except:
            print(f"ポート{self.local_port}の解放に失敗しました（既に解放されている可能性があります）")
        
        try:
            print(f"ポート{self.local_port}にバインドしています...")
            self.socket.bind((self.local_ip, self.local_port))
            print(f"ポート{self.local_port}へのバインドに成功しました")
        except OSError as e:
            if hasattr(e, 'errno') and e.errno == 98:  # Address already in use
                print(f"警告: ポート{self.local_port}は既に使用されています。")
                print("別のポートを試します...")
                # 別のポートを試す
                self.local_port = 8890
                try:
                    self.socket.bind((self.local_ip, self.local_port))
                    print(f"ポート{self.local_port}へのバインドに成功しました")
                except Exception as e2:
                    print(f"エラー: {e2}")
                    import sys
                    sys.exit(1)
            else:
                print(f"エラー: {e}")
                raise

        # レスポンス受信用スレッド
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        # Tello関連の変数
        self.tello_ip_list = []
        self.tello_list = []
        self.log = defaultdict(list)
        self.responses = {}  # IPアドレスをキーとしたレスポンスの辞書

        # タイムアウト設定
        self.COMMAND_TIMEOUT = 9.0

    def __del__(self):
        """
        デストラクタ - ソケットを閉じる
        """
        self.socket.close()
        print("ソケットを閉じました")

    def find_available_tello(self, num, timeout=30):
        """
        ネットワーク内で利用可能なTelloドローンを検索する
        :param num: 検索するTelloドローンの理想数
        :param timeout: 検索のタイムアウト時間（秒）
        :return: None
        """
        print(f'[開始] 最大{num}台のTelloドローンを検索しています...')

        # サブネット情報を取得
        subnets, address = self._get_subnets()
        possible_addr = []

        # サブネット内の全IPアドレスをリストアップ
        for subnet, netmask in subnets:
            for ip in range(1, 255):
                possible_addr.append(f"{subnet}.{ip}")

        # 検索開始時間を記録
        start_time = time.time()
        search_count = 0
        
        # 指定された数のTelloドローンが見つかるか、タイムアウトするまで検索
        while len(self.tello_ip_list) < num and (time.time() - start_time) < timeout:
            search_count += 1
            print(f'[検索中] サブネット内のTelloドローンを検索しています... (試行 {search_count})')

            # 既に見つかったTelloドローンをリストから削除
            for tello_ip in self.tello_ip_list:
                if tello_ip in possible_addr:
                    possible_addr.remove(tello_ip)
            
            # 自分自身のIPアドレスをスキップ
            for ip in possible_addr:
                if ip in address:
                    continue

                # 'command'コマンドを送信
                try:
                    self.socket.sendto(b'command', (ip, 8889))
                except Exception as e:
                    print(f"[エラー] コマンド送信エラー ({ip}): {e}")
            
            # レスポンスを待つ（短い間隔で複数回試行）
            time.sleep(3)
            
            # 少なくとも1機見つかった場合は、もう少し待ってから次の試行へ
            if len(self.tello_ip_list) > 0:
                print(f"[情報] {len(self.tello_ip_list)}機のドローンが見つかりました。追加のドローンを{min(5, timeout-(time.time()-start_time))}秒間探します...")
                time.sleep(min(2, timeout-(time.time()-start_time)))

        # 検索結果を表示
        if len(self.tello_ip_list) == 0:
            print("[警告] Telloドローンが見つかりませんでした")
        else:
            print(f"[成功] {len(self.tello_ip_list)}機のTelloドローンが見つかりました")
            for i, ip in enumerate(self.tello_ip_list):
                print(f"  ドローン {i+1}: {ip}")

        # Telloドローン以外のアドレスをログから除外
        temp = defaultdict(list)
        for ip in self.tello_ip_list:
            temp[ip] = self.log[ip]
        self.log = temp

    def _get_subnets(self):
        """
        サーバーのネットワーク接続を調べ、サブネットアドレスとサーバーIPを返す
        :return: list[tuple]: サブネットとネットマスクのタプルのリスト
                 list[str]: IPアドレスのリスト
        """
        # 簡易的な実装 - 192.168.11.0/24 サブネットを仮定
        return [("192.168.11", "255.255.255.0")], ["192.168.11.1"]

    def get_tello_list(self):
        """
        検出されたTelloドローンのリストを返す
        :return: list[CustomTello]: CustomTelloオブジェクトのリスト
        """
        # 検出されたIPアドレスからCustomTelloオブジェクトを作成
        if not self.tello_list:
            for ip in self.tello_ip_list:
                self.tello_list.append(CustomTello(ip, self))
                
        return self.tello_list

    def send_command(self, command, ip):
        """
        コマンドを送信する
        :param command: 送信するコマンド
        :param ip: 送信先のIPアドレス
        :return: レスポンス
        """
        print(f"コマンド送信: {command} -> {ip}")
        
        # コマンドを送信
        self.socket.sendto(command.encode('utf-8'), (ip, 8889))
        
        # レスポンスを待つ
        start_time = time.time()
        while ip not in self.responses:
            if time.time() - start_time > self.COMMAND_TIMEOUT:
                print(f"タイムアウト: {command} -> {ip}")
                return None
            time.sleep(0.1)
            
        # レスポンスを取得して削除
        response = self.responses.pop(ip)
        print(f"レスポンス受信: {response} <- {ip}")
        
        return response

    def _receive_thread(self):
        """
        レスポンス受信スレッド
        """
        while True:
            try:
                # レスポンスを受信
                response, ip_port = self.socket.recvfrom(1024)
                ip = ip_port[0]
                response = response.decode('utf-8').strip()
                
                # 'ok'レスポンスを受信した場合、Telloドローンとして登録
                if response == 'ok' and ip not in self.tello_ip_list:
                    print(f"[Tello発見] IPアドレス: {ip}")
                    self.tello_ip_list.append(ip)
                
                # レスポンスを記録
                print(f"[単一レスポンス] IP:{ip} レスポンス: {response}")
                self.responses[ip] = response
                
            except socket.timeout:
                # タイムアウト
                print("[エラー] ソケットエラー: timed out")
            except Exception as e:
                # その他のエラー
                print(f"[エラー] 例外が発生しました: {e}")
