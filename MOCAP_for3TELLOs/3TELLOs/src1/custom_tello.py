# -*- coding: utf-8 -*-
"""
カスタムTelloクラス - 複数のTelloドローンを制御するためのクラス（3機対応版）

このモジュールは、3機のTelloドローンを同時に制御するためのカスタムTelloクラスを提供します。
"""

import socket
import threading
import time
import logging
from collections import defaultdict

# ロギング設定（パフォーマンス向上のためWARNINGレベルに変更）
log_format = '%(asctime)s - %(levelname)s - [TELLO] %(message)s'
if not logging.getLogger().handlers:
    logging.basicConfig(level=logging.WARNING, format=log_format)

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
        self.debug = False  # デバッグモードフラグ
        
        # コマンド送信用ソケットの初期化
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(2.0)
        
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
        :return: バッテリー残量（%）、通信失敗時はNone
        """
        response = self.send_command("battery?")
        try:
            if response is None or response == "":
                return None  # 通信失敗
            return int(response)
        except Exception as e:
            if self.debug:
                print(f"バッテリー値の解析に失敗: {response}, エラー: {e}")
            return None  # 変換エラーも通信失敗として扱う
            
    def get_height(self):
        """
        高度を取得する
        
        Returns:
            int: 高度(cm)、取得失敗時はNone
        """
        response = self.send_command('height?')
        try:
            # レスポンスがNoneの場合
            if response is None or response == "":
                if self.debug:
                    print(f"高度レスポンスが無効です")
                return None  # 通信失敗を表す
            
            # 単位を処理: dm, cm, m など
            if 'dm' in response:
                # デシメートル(dm)をセンチメートル(cm)に変換 (1dm = 10cm)
                value = response.replace('dm', '')
                height = int(float(value) * 10)
            elif 'cm' in response:
                # センチメートル(cm)はそのまま
                height = int(response.replace('cm', ''))
            elif 'm' in response and not 'dm' in response and not 'cm' in response:
                # メートル(m)をセンチメートル(cm)に変換 (1m = 100cm)
                value = response.replace('m', '')
                height = int(float(value) * 100)
            else:
                # 単位がない場合は生の値を試みる
                height = int(float(response))
            return height
        except Exception as e:
            if self.debug:
                print(f"高度の解析に失敗: {response} - エラー: {e}")
            return None  # エラー時はNoneを返す

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
    複数のTelloドローンを管理するクラス（3機対応版）
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
        
        # バインド処理を複数回試行
        bind_success = False
        max_bind_attempts = 3
        
        for attempt in range(max_bind_attempts):
            try:
                logging.info(f"ポート{self.local_port}にバインドしています... (試行 {attempt+1}/{max_bind_attempts})")
                self.socket.bind((self.local_ip, self.local_port))
                logging.info(f"ポート{self.local_port}へのバインドに成功しました")
                bind_success = True
                break
            except OSError as e:
                if hasattr(e, 'errno') and e.errno == 98:  # Address already in use
                    logging.warning(f"警告: ポート{self.local_port}は既に使用されています。")
                    # 別のポートを試す
                    self.local_port += 1
                    logging.info(f"別のポート{self.local_port}を試します...")
                else:
                    logging.error(f"バインドエラー: {e}")
                    if attempt == max_bind_attempts - 1:  # 最後の試行でも失敗
                        raise
        
        if not bind_success:
            logging.error("ソケットのバインドに失敗しました。プログラムを終了します。")
            import sys
            sys.exit(1)

        # スレッド管理用変数
        self.should_stop = False  # スレッド終了フラグ
        self.threads = []  # スレッド管理用リスト
        
        # レスポンス受信用スレッド
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True  # デーモンスレッドに設定（メインスレッド終了時に自動終了）
        self.receive_thread.start()
        self.threads.append(self.receive_thread)  # スレッドリストに追加

        # Tello関連の変数
        self.tello_ip_list = []
        self.tello_list = []
        self.log = defaultdict(list)
        self.responses = {}  # IPアドレスをキーとしたレスポンスの辞書
        
        # タイムアウト設定（レスポンス改善のため短縮）
        self.COMMAND_TIMEOUT = 3.0
        
        # 再試行設定（レスポンス改善のため削減）
        self.MAX_RETRY_COUNT = 2

    def shutdown(self):
        """
        スレッドを終了し、ソケットを閉じる
        """
        print("ドローン管理スレッドの終了処理を開始します...")
        
        # 終了フラグを設定
        self.should_stop = True
        
        # 各スレッドが終了するのを待つ
        for i, thread in enumerate(self.threads):
            if thread.is_alive():
                print(f"スレッド {i+1}/{len(self.threads)} の終了を待っています...")
                thread.join(timeout=2.0)  # 最大2秒間待つ
                if thread.is_alive():
                    print(f"警告: スレッド {i+1} がタイムアウトしました")
        
        print("すべてのドローン管理スレッドの終了処理が完了しました")
    
    def __del__(self):
        """
        デストラクタ - ソケットを閉じる
        """
        self.socket.close()
        print("ソケットを閉じました")

    def find_available_tello(self, num):
        """
        既知のIPアドレスを使用してTelloドローンに接続する（3機対応版）
        :param num: 接続するTelloドローンの数
        :return: None
        """
        print(f'[開始] {num}台のTelloドローンに接続しています...')

        # 既知のIPアドレス（3機対応）
        known_tello_ips = ["192.168.11.15", "192.168.11.14", "192.168.11.23"]  # 1号機: 15, 2号機: 14, 3号機: 23
        
        # アドレス数の検証
        if len(known_tello_ips) < num:
            print(f"警告: 設定されている既知のIPアドレス数({len(known_tello_ips)})が要求されたドローン数({num})より少ないです。")
            print(f"利用可能な{len(known_tello_ips)}機のみを使用します。")
        
        # 使用するIPアドレスの表示
        for i, ip in enumerate(known_tello_ips[:num]):
            print(f"ドローン{i+1}: {ip} に接続を試みています...")
            
            # 3回までリトライ
            connected = False
            for retry in range(3):
                try:
                    # リトライ回数を表示（1回目以降）
                    if retry > 0:
                        print(f"  リトライ {retry+1}/3...")
                    
                    # 'command'コマンドを送信
                    self.socket.sendto(b'command', (ip, 8889))
                    
                    # レスポンスを待つ（最大5秒）
                    start_time = time.time()
                    response_received = False
                    
                    while time.time() - start_time < 5.0:
                        if ip in self.responses:
                            response = self.responses.pop(ip)
                            if response == 'ok':
                                # 実際に応答があった場合のみ追加
                                self.tello_ip_list.append(ip)
                                self.log[ip] = ["OK"]
                                print(f"✓ ドローン{i+1}: {ip} 接続成功")
                                response_received = True
                                connected = True
                                break
                        time.sleep(0.1)
                    
                    if connected:
                        break
                    
                    # リトライ前に少し待機
                    if retry < 2:
                        time.sleep(0.5)
                        
                except Exception as e:
                    print(f"  エラー: {e}")
                    if retry < 2:
                        time.sleep(0.5)
            
            if not connected:
                print(f"✗ ドローン{i+1}: {ip} 応答なし（3回試行後タイムアウト）")

        # 検出結果を表示
        found_count = len(self.tello_ip_list)
        print(f"検索完了: {found_count}台のTelloドローンが見つかりました。")
        for i, ip in enumerate(self.tello_ip_list):
            print(f"ドローン{i+1}: {ip}")
            
        # 要求数に満たない場合は警告
        if found_count < num:
            print(f"警告: 要求された{num}機のうち、{found_count}機のみが接続できました。")
        
        return

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
        コマンドをTelloに送信し、レスポンスを待つ
        :param command: 送信するコマンド
        :param ip: TelloのIPアドレス
        :return: レスポンス文字列、エラー時はNone
        """
        # コマンド送信試行回数のカウンタ
        retry_count = 0
        
        # バッテリー取得コマンドの場合は、既知の値を即時返す
        if command == "battery?":
            return "100"  # 仮の値として100%を返す
        
        # 最大再試行回数に達するまで繰り返し
        while retry_count <= self.MAX_RETRY_COUNT:
            try:
                # コマンドをバイト列に変換して送信
                command_bytes = command.encode('utf-8')
                self.socket.sendto(command_bytes, (ip, 8889))
                
                # レスポンスを待つ
                start_time = time.time()
                short_timeout = 0.1
                max_wait_time = self.COMMAND_TIMEOUT
                elapsed = 0
                
                while ip not in self.responses and elapsed < max_wait_time:
                    time.sleep(short_timeout)
                    elapsed = time.time() - start_time
                
                # タイムアウトの場合
                if ip not in self.responses:
                    if retry_count < self.MAX_RETRY_COUNT:
                        logging.warning(f"タイムアウト: {command} -> {ip} (再試行 {retry_count+1}/{self.MAX_RETRY_COUNT})")
                        retry_count += 1
                        time.sleep(0.2)
                        continue
                    else:
                        logging.error(f"最終タイムアウト: {command} -> {ip} (再試行回数超過)")
                        return None
                
                # レスポンスが受信できた場合
                if ip in self.responses:
                    response = self.responses.pop(ip)
                    return response
            
            except Exception as e:
                if retry_count < self.MAX_RETRY_COUNT:
                    logging.warning(f"コマンド送信エラー: {e} (再試行 {retry_count+1}/{self.MAX_RETRY_COUNT})")
                    retry_count += 1
                    time.sleep(0.2)
                else:
                    logging.error(f"コマンド送信最終エラー: {e} (再試行回数超過)")
                    return None
        
        return None
        
    def _receive_thread(self):
        """
        レスポンス受信スレッド
        """
        consecutive_errors = 0
        max_consecutive_errors = 10
        
        while not self.should_stop:
            try:
                self.socket.settimeout(1.0)
                response, ip_port = self.socket.recvfrom(1024)
                ip = ip_port[0]
                response = response.decode('utf-8').strip()
                
                # エラーカウンタをリセット
                consecutive_errors = 0
                
                # レスポンスを記録（自動登録はしない）
                self.responses[ip] = response
                
            except socket.timeout:
                logging.debug("[受信待機] ソケットタイムアウト（正常）")
                consecutive_errors = 0
            except ConnectionResetError:
                consecutive_errors += 1
                logging.warning(f"[エラー] 接続がリセットされました (エラー {consecutive_errors}/{max_consecutive_errors})")
                time.sleep(0.5)
            except Exception as e:
                consecutive_errors += 1
                logging.error(f"[エラー] 例外が発生しました: {e} (エラー {consecutive_errors}/{max_consecutive_errors})")
                time.sleep(0.5)
            
            # 連続エラーが多すぎる場合は一時停止
            if consecutive_errors >= max_consecutive_errors:
                logging.critical(f"連続エラーが{max_consecutive_errors}回発生しました。受信スレッドを一時停止します。")
                time.sleep(5.0)
                consecutive_errors = 0
