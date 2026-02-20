# -*- coding: utf-8 -*-
"""
カスタムTelloクラス - 複数のTelloドローンを制御するためのクラス

このモジュールは、複数のTelloドローンを同時に制御するためのカスタムTelloクラスを提供します。
リファレンスコードのtello_manager_py3.pyを参考にしています。
"""

import socket
import threading
import time
import logging
from collections import defaultdict

# ロギング設定
log_format = '%(asctime)s - %(levelname)s - [TELLO] %(message)s'
if not logging.getLogger().handlers:
    logging.basicConfig(level=logging.INFO, format=log_format)

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
        
        # ソケットのタイムアウトを設定（タイムアウト値を増加）
        self.socket.settimeout(10.0)
        
        # 既存のソケットを閉じる試み（ポートの解放）
        try:
            temp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            temp_socket.bind((self.local_ip, self.local_port))
            temp_socket.close()
            logging.info(f"ポート{self.local_port}を解放しました")
        except:
            logging.warning(f"ポート{self.local_port}の解放に失敗しました（既に解放されている可能性があります）")
        
        try:
            logging.info(f"ポート{self.local_port}にバインドしています...")
            self.socket.bind((self.local_ip, self.local_port))
            logging.info(f"ポート{self.local_port}へのバインドに成功しました")
        except OSError as e:
            if hasattr(e, 'errno') and e.errno == 98:  # Address already in use
                logging.warning(f"警告: ポート{self.local_port}は既に使用されています。")
                logging.info("別のポートを試します...")
                # 別のポートを試す
                self.local_port = 8890
                try:
                    self.socket.bind((self.local_ip, self.local_port))
                    logging.info(f"ポート{self.local_port}へのバインドに成功しました")
                except Exception as e2:
                    logging.error(f"エラー: {e2}")
                    import sys
                    sys.exit(1)
            else:
                logging.error(f"エラー: {e}")
                raise

        # スレッド管理用変数
        self.should_stop = False  # スレッド終了フラグ
        self.threads = []  # スレッド管理用リスト
        
        # レスポンス受信用スレッド
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = False  # 非デーモンスレッドに変更
        self.receive_thread.start()
        self.threads.append(self.receive_thread)  # スレッドリストに追加

        # Tello関連の変数
        self.tello_ip_list = []
        self.tello_list = []
        self.log = defaultdict(list)
        self.responses = {}  # IPアドレスをキーとしたレスポンスの辞書
        
        # タイムアウト設定（タイムアウト値を増加）
        self.COMMAND_TIMEOUT = 15.0
        
        # 再試行設定
        self.MAX_RETRY_COUNT = 3

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
        ネットワーク内で利用可能なTelloドローンを検索する
        :param num: 検索するTelloドローンの数
        :return: None
        """
        print(f'[開始] {num}台のTelloドローンを検索しています...')

        # サブネット情報を取得
        subnets, address = self._get_subnets()
        possible_addr = []

        # サブネット内の全IPアドレスをリストアップ
        for subnet, netmask in subnets:
            for ip in range(1, 255):
                possible_addr.append(f"{subnet}.{ip}")

        # 指定された数のTelloドローンが見つかるまで検索
        while len(self.tello_ip_list) < num:
            print('[検索中] サブネット内のTelloドローンを検索しています...')

            # 既に見つかったTelloドローンをリストから削除
            for tello_ip in self.tello_ip_list:
                if tello_ip in possible_addr:
                    possible_addr.remove(tello_ip)
            
            # 自分自身のIPアドレスをスキップ
            for ip in possible_addr:
                if ip in address:
                    continue

                # 'command'コマンドを送信
                self.socket.sendto(b'command', (ip, 8889))
            
            # レスポンスを待つ
            time.sleep(5)

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
        logging.info(f"コマンド送信: {command} -> {ip}")
        
        # 再試行カウンタ
        retry_count = 0
        
        while retry_count <= self.MAX_RETRY_COUNT:
            try:
                # コマンドを送信
                self.socket.sendto(command.encode('utf-8'), (ip, 8889))
                
                # レスポンスを待つ
                start_time = time.time()
                while ip not in self.responses:
                    if time.time() - start_time > self.COMMAND_TIMEOUT:
                        if retry_count < self.MAX_RETRY_COUNT:
                            logging.warning(f"タイムアウト: {command} -> {ip} (再試行 {retry_count+1}/{self.MAX_RETRY_COUNT})")
                            retry_count += 1
                            break
                        else:
                            logging.error(f"最終タイムアウト: {command} -> {ip} (再試行回数超過)")
                            return None
                    time.sleep(0.1)
                
                # レスポンスが受信できた場合
                if ip in self.responses:
                    # レスポンスを取得して削除
                    response = self.responses.pop(ip)
                    logging.info(f"レスポンス受信: {response} <- {ip}")
                    return response
            
            except Exception as e:
                if retry_count < self.MAX_RETRY_COUNT:
                    logging.warning(f"コマンド送信エラー: {e} (再試行 {retry_count+1}/{self.MAX_RETRY_COUNT})")
                    retry_count += 1
                    time.sleep(1.0)  # エラー後の待機時間
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
        
        while not self.should_stop:  # 終了フラグをチェック
            try:
                # レスポンスを受信
                # タイムアウトを短く設定して、should_stopを適切にチェックできるようにする
                self.socket.settimeout(1.0)  # タイムアウトを1.0秒に延長
                response, ip_port = self.socket.recvfrom(1024)
                ip = ip_port[0]
                response = response.decode('utf-8').strip()
                
                # エラーカウンタをリセット（正常に受信できた）
                consecutive_errors = 0
                
                # 'ok'レスポンスを受信した場合、Telloドローンとして登録
                if response == 'ok' and ip not in self.tello_ip_list:
                    logging.info(f"[Tello発見] IPアドレス: {ip}")
                    self.tello_ip_list.append(ip)
                
                # レスポンスを記録
                logging.debug(f"[単一レスポンス] IP:{ip} レスポンス: {response}")
                self.responses[ip] = response
                
            except socket.timeout:
                # タイムアウト - 正常な動作の一部なので、DEBUGレベルでログ出力
                logging.debug("[受信待機] ソケットタイムアウト（正常）")
                consecutive_errors = 0  # タイムアウトは正常な動作なのでリセット
            except ConnectionResetError:
                # 接続リセットエラー
                consecutive_errors += 1
                logging.warning(f"[エラー] 接続がリセットされました (エラー {consecutive_errors}/{max_consecutive_errors})")
                time.sleep(0.5)  # 少し待機
            except Exception as e:
                # その他のエラー
                consecutive_errors += 1
                logging.error(f"[エラー] 例外が発生しました: {e} (エラー {consecutive_errors}/{max_consecutive_errors})")
                time.sleep(0.5)  # 少し待機
            
            # 連続エラーが多すぎる場合は一時停止して回復を試みる
            if consecutive_errors >= max_consecutive_errors:
                logging.critical(f"連続エラーが{max_consecutive_errors}回発生しました。受信スレッドを一時停止します。")
                time.sleep(5.0)  # 5秒間待機して回復を試みる
                consecutive_errors = 0  # カウンタをリセット
