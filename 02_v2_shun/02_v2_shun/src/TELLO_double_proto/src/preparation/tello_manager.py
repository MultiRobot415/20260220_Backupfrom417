"""
Tello Manager - Telloドローンの管理クラス

このモジュールは、複数のTelloドローンを管理し、通信を行うためのクラスを提供します。
ネットワーク内のTelloドローンを自動的に検出し、コマンドを送信します。

Python3環境で動作するように修正されています。
"""

import threading
import socket
import time
import netifaces
import netaddr
from netaddr import IPNetwork
from collections import defaultdict

class Stats:
    """
    Telloの統計情報を記録するクラス
    """
    def __init__(self, command, id):
        self.command = command
        self.response = None
        self.id = id
        self.start_time = time.time()
        self.end_time = None
        self.duration = None
        self.drone_ip = None

    def add_response(self, response, ip):
        """
        レスポンスを追加し、終了時間と所要時間を記録
        """
        self.response = response
        self.end_time = time.time()
        self.duration = self.end_time - self.start_time
        self.drone_ip = ip

    def got_response(self):
        """
        レスポンスが返ってきたかどうかを確認
        """
        return self.response is not None

    def return_stats(self):
        """
        統計情報を文字列として返す
        """
        return f'Command: {self.command}\nResponse: {self.response}\nStart time: {self.start_time}\nEnd time: {self.end_time}\nDuration: {self.duration}\nDrone IP: {self.drone_ip}\n'

class Tello:
    """
    Telloドローンとのインターフェースを提供するラッパークラス
    通信はTello_Managerによって処理される
    """
    def __init__(self, tello_ip, tello_manager):
        self.tello_ip = tello_ip
        self.tello_manager = tello_manager

    def send_command(self, command):
        """
        コマンドを送信する
        """
        return self.tello_manager.send_command(command, self.tello_ip)

class Tello_Manager:
    """
    複数のTelloドローンを管理し、通信を行うクラス
    """
    def __init__(self):
        # ソケット設定
        self.local_ip = ''
        self.local_port = 8889
        
        # 既存のソケットを閉じる試み（ポートの解放）
        try:
            temp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            temp_socket.bind((self.local_ip, self.local_port))
            temp_socket.close()
            print(f"ポート{self.local_port}を解放しました")
        except:
            print(f"ポート{self.local_port}の解放に失敗しました（既に解放されている可能性があります）")
        
        # 新しいソケットを作成
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

        # タイムアウト設定
        self.COMMAND_TIME_OUT = 9.0

        # マルチコマンド用の変数
        self.last_response_index = {}
        self.str_cmd_index = {}

    def __del__(self):
        """
        デストラクタ - ソケットを閉じる
        """
        self.socket.close()
        print("ソケットを閉じました")

    def find_avaliable_tello(self, num):
        """
        ネットワーク内で利用可能なTelloドローンを検索する
        :param num: 検索するTelloドローンの数
        :return: None
        """
        print(f'[開始] {num}台のTelloドローンを検索しています...')

        # サブネット情報を取得
        subnets, address = self.get_subnets()
        possible_addr = []

        # サブネット内の全IPアドレスをリストアップ
        for subnet, netmask in subnets:
            for ip in IPNetwork(f'{subnet}/{netmask}'):
                # ローカルアドレスとブロードキャストアドレスをスキップ
                if str(ip).split('.')[3] == '0' or str(ip).split('.')[3] == '255':
                    continue
                possible_addr.append(str(ip))

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

                # コマンドを記録
                self.log[ip].append(Stats('command', len(self.log[ip])))
                # 'command'コマンドを送信
                self.socket.sendto(b'command', (ip, 8889))
            
            # レスポンスを待つ
            time.sleep(5)

        # Telloドローン以外のアドレスをログから除外
        temp = defaultdict(list)
        for ip in self.tello_ip_list:
            temp[ip] = self.log[ip]
        self.log = temp

    def get_subnets(self):
        """
        サーバーのネットワーク接続を調べ、サブネットアドレスとサーバーIPを返す
        :return: list[tuple]: サブネットとネットマスクのタプルのリスト
                 list[str]: IPアドレスのリスト
        """
        subnets = []
        ifaces = netifaces.interfaces()
        addr_list = []
        
        for myiface in ifaces:
            addrs = netifaces.ifaddresses(myiface)

            if socket.AF_INET not in addrs:
                continue
                
            # IPv4情報を取得
            ipinfo = addrs[socket.AF_INET][0]
            address = ipinfo['addr']
            netmask = ipinfo['netmask']

            # 検索範囲を制限（ルーターサブネット用）
            if netmask != '255.255.255.0':
                continue

            # IPオブジェクトを作成
            cidr = netaddr.IPNetwork(f'{address}/{netmask}')
            network = cidr.network
            
            subnets.append((network, netmask))
            addr_list.append(address)
            
        return subnets, addr_list

    def get_tello_list(self):
        """
        Telloオブジェクトのリストを返す
        """
        return self.tello_list

    def send_command(self, command, ip):
        """
        コマンドをIPアドレスに送信する
        最後のコマンドが'OK'を受信するまでブロックされる
        コマンドが失敗した場合（タイムアウトまたはエラー）、再送信を試みる
        :param command: 送信するコマンド
        :param ip: TelloドローンのIPアドレス
        :return: 最新のコマンドレスポンス
        """
        # マルチコマンドかどうかを判断
        if isinstance(command, bytes) and len(command) >= 2:
            command_sof_1 = command[0]
            command_sof_2 = command[1]
        elif isinstance(command, str) and len(command) >= 2:
            command_sof_1 = ord(command[0])
            command_sof_2 = ord(command[1])
        else:
            command_sof_1 = 0
            command_sof_2 = 0
            
        multi_cmd_send_flag = (command_sof_1 == 0x52 and command_sof_2 == 0x65)

        # マルチコマンドの場合
        if multi_cmd_send_flag:
            self.str_cmd_index[ip] = self.str_cmd_index.get(ip, 0) + 1
            for num in range(1, 5):
                str_cmd_index_h = self.str_cmd_index[ip] // 128 + 1
                str_cmd_index_l = self.str_cmd_index[ip] % 128
                if str_cmd_index_l == 0:
                    str_cmd_index_l = str_cmd_index_l + 2
                cmd_sof = [0x52, 0x65, str_cmd_index_h, str_cmd_index_l, 0x01, num + 1, 0x20]
                cmd_sof_str = bytes(cmd_sof)
                
                if isinstance(command, str):
                    cmd = cmd_sof_str + command[3:].encode('utf-8')
                else:
                    cmd = cmd_sof_str + command[3:]
                    
                self.socket.sendto(cmd, (ip, 8889))

            print(f'[マルチコマンド] IP:{ip} コマンド: {command[3:]}')
            real_command = command[3:] if isinstance(command, str) else command[3:].decode('utf-8')
        else:
            # 単一コマンドの場合
            if isinstance(command, str):
                self.socket.sendto(command.encode('utf-8'), (ip, 8889))
                print(f'[単一コマンド] IP:{ip} コマンド: {command}')
                real_command = command
            else:
                self.socket.sendto(command, (ip, 8889))
                print(f'[単一コマンド] IP:{ip} コマンド: {command.decode("utf-8")}')
                real_command = command.decode('utf-8')

        # コマンドをログに記録
        self.log[ip].append(Stats(real_command, len(self.log[ip])))
        
        # レスポンスを待つ
        start = time.time()
        while not self.log[ip][-1].got_response():
            now = time.time()
            diff = now - start
            if diff > self.COMMAND_TIME_OUT:
                print(f'[タイムアウト] コマンド: {real_command}')
                return

    def _receive_thread(self):
        """
        Telloからのレスポンスを受信するスレッド
        """
        while True:
            try:
                response, ip_port = self.socket.recvfrom(1024)
                ip = ip_port[0]
                
                # 新しいTelloドローンを発見した場合
                if response.upper() == b'OK' and ip not in self.tello_ip_list:
                    print(f'[Tello発見] IPアドレス: {ip}')
                    self.tello_ip_list.append(ip)
                    self.last_response_index[ip] = 100
                    self.tello_list.append(Tello(ip, self))
                    self.str_cmd_index[ip] = 1
                
                # レスポンスの処理
                try:
                    response_str = response.decode('utf-8')
                except UnicodeDecodeError:
                    response_str = str(response)
                
                # マルチコマンドのレスポンスかどうかを判断
                response_sof_part1 = response[0] if len(response) > 0 else 0
                response_sof_part2 = response[1] if len(response) > 1 else 0
                
                if response_sof_part1 == 0x52 and response_sof_part2 == 0x65:
                    response_index = response[3] if len(response) > 3 else 0
                    
                    if ip in self.last_response_index and response_index != self.last_response_index[ip]:
                        print(f'[マルチレスポンス] IP:{ip} レスポンス: {response_str[7:]}')
                        if ip in self.log and len(self.log[ip]) > 0:
                            self.log[ip][-1].add_response(response_str[7:], ip)
                    
                    if ip in self.last_response_index:
                        self.last_response_index[ip] = response_index
                else:
                    print(f'[単一レスポンス] IP:{ip} レスポンス: {response_str}')
                    if ip in self.log and len(self.log[ip]) > 0:
                        self.log[ip][-1].add_response(response_str, ip)
                
            except Exception as e:
                print(f'[エラー] ソケットエラー: {e}')

    def get_log(self):
        """
        ログを取得する
        """
        return self.log
