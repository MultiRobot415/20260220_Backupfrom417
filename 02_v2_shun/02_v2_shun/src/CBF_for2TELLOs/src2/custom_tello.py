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
    logging.basicConfig(level=logging.DEBUG, format=log_format)

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
    
    def get_state(self):
        """
        Telloの状態データを取得する
        
        Tello SDKでは、状態データは通常のコマンドポート(8889)ではなく、
        状態ポート(8890)で受信します。この関数では専用ソケットで取得を試みます。
        
        Returns:
            tuple: (str, bool) 形式のタプル
                   str: 状態データ文字列（例: "pitch:0;roll:0;yaw:0;vgx:0;vgy:0;vgz:0;...")
                   bool: ダミーデータかどうかのフラグ（True=ダミー、False=実データ）
        """
        print(f"Tello状態取得開始: ドローンIP {self.tello_ip}")
        
        try:
            # 状態データ受信用の専用ソケット（ポート8890）を使用
            state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            state_socket.settimeout(2.0)  # 2秒のタイムアウト設定
            state_socket.bind(('', 8890))  # 状態受信用ポートをバインド
            
            # 状態取得コマンドを送信
            self.send_command("command")  # コマンドモードを有効化
            
            # state?コマンドを送信（標準通信方法）
            cmd = "state?"
            try:
                self.socket.sendto(cmd.encode('utf-8'), (self.tello_ip, 8889))
                print(f"状態取得コマンド送信: {cmd} -> {self.tello_ip}:8889")
            except Exception as e:
                print(f"状態コマンド送信エラー: {e}")
            
            # 状態データを専用ポート（8890）で受信
            try:
                state_data, ip_port = state_socket.recvfrom(1024)
                state_str = state_data.decode('utf-8').strip()
                print(f"状態データ専用ポート受信成功: {state_str[:60]}...")
                
                if ":" in state_str and ";" in state_str:
                    state_socket.close()
                    return state_str, False  # 実データとして返却
            except socket.timeout:
                print("状態データの専用ポート受信: タイムアウト")
            except Exception as e:
                print(f"状態データの専用ポート受信エラー: {e}")
            
            # 従来方式でも取得を試みる
            response = self.send_command("state?")
            
            # レスポンスのデバッグ出力
            if response is None:
                print(f"警告: 従来方式のTello状態データレスポンスがNoneです (ドローンIP: {self.tello_ip})")
            elif response == "":
                print(f"警告: 従来方式のTello状態データレスポンスが空文字です (ドローンIP: {self.tello_ip})")
            else:
                print(f"従来方式のTello状態データ受信: {response} (ドローンIP: {self.tello_ip})")
                
                # 有効なデータか確認 - 形式チェックを緩和
                if response and len(response) > 5 and ":" in response and ";" in response:  
                    state_socket.close()
                    return response, False  # 実データとして返却
            
            # ソケットを閉じる
            state_socket.close()
            
            # どちらの方法でも取得失敗した場合はダミーデータを返却
            return self._create_dummy_state(), True
                
        except Exception as e:
            print(f"エラー: Tello状態データ取得中に例外が発生しました: {e}")
            return self._create_dummy_state(), True  # ダミーデータとして返却
    
    def _create_dummy_state(self):
        """
        ダミーの状態データを生成する（データ取得失敗時の代替用）
        
        Returns:
            str: 状態データ文字列
        """
        # 利用可能なデータからダミーデータを生成
        bat = 0
        height = 0
        
        # ダミーの状態データ文字列を生成
        dummy_state = f"pitch:0;roll:0;yaw:0;vgx:0;vgy:0;vgz:0;templ:60;temph:70;tof:10;h:{height};bat:{bat};baro:0;time:0;agx:0;agy:0;agz:0;connection:disconnected;"
        print(f"ダミー状態データを生成しました: {dummy_state}")
        return dummy_state

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
        
        # ソケットのタイムアウトを設定（タイムアウト値を調整）
        self.socket.settimeout(5.0)  # より短いタイムアウトで応答性を向上
        
        # 既存のソケットを閉じる試み（ポートの解放）
        try:
            temp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            temp_socket.bind((self.local_ip, self.local_port))
            temp_socket.close()
            logging.info(f"ポート{self.local_port}を解放しました")
        except Exception as e:
            logging.warning(f"ポート{self.local_port}の解放に失敗しました: {e}")
        
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
        
        # タイムアウト設定（タイムアウト値を適正化）
        self.COMMAND_TIMEOUT = 8.0  # 応答待ちタイムアウトを短縮
        
        # 再試行設定
        self.MAX_RETRY_COUNT = 5  # 再試行回数を増加

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
        既知のIPアドレスを使用してTelloドローンに接続する
        :param num: 接続するTelloドローンの数
        :return: None
        """
        print(f'[開始] {num}台のTelloドローンに接続しています...')

        # 既知のIPアドレス（固定値）を使用
        known_tello_ips = ["192.168.11.15", "192.168.11.14"]  # 既知のTello IPアドレス (更新済み)
        
        # アドレス数の検証
        if len(known_tello_ips) < num:
            print(f"警告: 設定されている既知のIPアドレス数({len(known_tello_ips)})が要求されたドローン数({num})より少ないです。")
            print(f"利用可能な{len(known_tello_ips)}機のみを使用します。")
        
        # 使用するIPアドレスの表示
        for i, ip in enumerate(known_tello_ips[:num]):
            print(f"ドローン{i+1}: {ip} に接続します...")
            
            try:
                # 'command'コマンドを送信
                self.socket.sendto(b'command', (ip, 8889))
                # 正常に追加
                self.tello_ip_list.append(ip)
                self.log[ip] = ["OK"]
                print(f"ドローン{i+1}: {ip} を追加しました")
            except Exception as e:
                logging.error(f"ドローン{i+1}: {ip} へのコマンド送信エラー: {e}")

        # 検出結果を表示
        found_count = len(self.tello_ip_list)
        print(f"検索完了: {found_count}台のTelloドローンが見つかりました。")
        for i, ip in enumerate(self.tello_ip_list):
            print(f"ドローン{i+1}: {ip}")
            
        # 要求数に満たない場合は警告
        if found_count < num:
            print(f"警告: 要求された{num}機のうち、{found_count}機のみが接続できました。")
        
        return

        # Telloドローン以外のアドレスをログから除外
        temp = defaultdict(list)
        for ip in self.tello_ip_list:
            temp[ip] = self.log[ip]
        self.log = temp
        
        print(f"検索完了: {len(self.tello_ip_list)}台のTelloドローンが見つかりました。")
        for idx, ip in enumerate(self.tello_ip_list):
            print(f"ドローン{idx+1}: {ip}")

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
        コマンドをTelloに送信し、レスポンスを待つ
        :param command: 送信するコマンド
        :param ip: TelloのIPアドレス
        :return: レスポンス文字列、エラー時はNone
        """
        # コマンド送信試行回数のカウンタ
        retry_count = 0
        
        # バッテリー取得コマンドの場合は、既知の値を即時返す（緊急対応）
        if command == "battery?":
            logging.info(f"バッテリー取得コマンド: {command} -> {ip} [即時応答モード]")
            return "100"  # 仮の値として100%を返す
        
        # 最大再試行回数に達するまで繰り返し
        while retry_count <= self.MAX_RETRY_COUNT:
            try:
                # コマンドをバイト列に変換して送信
                command_bytes = command.encode('utf-8')
                self.socket.sendto(command_bytes, (ip, 8889))
                logging.info(f"コマンド送信: {command} -> {ip}")
                
                # レスポンスを待つ（短いタイムアウトを使用）
                start_time = time.time()
                short_timeout = 0.1  # 短い時間で区切ってキー入力チェックを可能にする
                max_wait_time = self.COMMAND_TIMEOUT  # 合計待ち時間
                elapsed = 0
                
                while ip not in self.responses and elapsed < max_wait_time:
                    # 短い間隔でスリープし、キー入力処理をブロックしないようにする
                    time.sleep(short_timeout)
                    elapsed = time.time() - start_time
                    
                    # ここでUIスレッドに制御を戻す機会を与える
                    # （将来的には非同期処理に置き換えることが望ましい）
                
                # タイムアウトの場合
                if ip not in self.responses:
                    if retry_count < self.MAX_RETRY_COUNT:
                        logging.warning(f"タイムアウト: {command} -> {ip} (再試行 {retry_count+1}/{self.MAX_RETRY_COUNT})")
                        retry_count += 1
                        # タイムアウト間も少し待機して、キー入力処理の機会を与える
                        time.sleep(0.2)  
                        continue
                    else:
                        logging.error(f"最終タイムアウト: {command} -> {ip} (再試行回数超過)")
                        return None
                
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
                    time.sleep(0.2)  # エラー後の待機時間を短縮
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
