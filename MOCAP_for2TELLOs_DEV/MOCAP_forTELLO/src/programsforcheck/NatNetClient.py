"""
OptiTrack NatNet direct depacketization library for Python 3.x
"""

import socket
import struct
import time
import logging
from threading import Thread

def trace(*args):
    # デバッグ用のトレース関数
    logging.debug("".join(map(str, args)))

# ロギング設定がまだ行われていない場合のためのデフォルト設定
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# 各種データ型の構造体定義
Vector3 = struct.Struct('<fff')
Quaternion = struct.Struct('<ffff')
FloatValue = struct.Struct('<f')
DoubleValue = struct.Struct('<d')

class NatNetClient:
    def __init__(self):
        # NatNetサーバーのIPアドレス（OptiTrackのPCのIPアドレスに変更する必要がある）
        self.serverIPAddress = "127.0.0.1"

        # ローカルネットワークインターフェースのIPアドレス
        self.localIPAddress = "127.0.0.1"

        # Motiveのストリーミング設定に記載されているマルチキャストアドレスと一致する必要がある
        self.multicastAddress = "239.255.42.99"

        # NatNetコマンドチャンネル
        self.commandPort = 1510

        # NatNetデータチャンネル
        self.dataPort = 1511

        # フレームごとに剛体データを受信するためのコールバックメソッド
        self.rigidBodyListener = None
        
        # 新しいフレームを受信するためのコールバックメソッド
        self.newFrameListener = None

        # NatNetストリームバージョン（初期化中に実際のバージョンに更新される）
        self.__natNetStreamVersion = (3, 0, 0, 0)

    # クライアント/サーバーメッセージID
    NAT_PING = 0
    NAT_PINGRESPONSE = 1
    NAT_REQUEST = 2
    NAT_RESPONSE = 3
    NAT_REQUEST_MODELDEF = 4
    NAT_MODELDEF = 5
    NAT_REQUEST_FRAMEOFDATA = 6
    NAT_FRAMEOFDATA = 7
    NAT_MESSAGESTRING = 8
    NAT_DISCONNECT = 9
    NAT_UNRECOGNIZED_REQUEST = 100

    # NatNetストリームに接続するためのデータソケットを作成
    def __createDataSocket(self, port):
        logging.debug(f"__createDataSocket: ポート {port} のデータソケットを作成します")
        logging.debug(f"__createDataSocket: マルチキャストアドレス={self.multicastAddress}, ローカルアドレス={self.localIPAddress}")
        
        try:
            # ソケットの作成
            result = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            logging.debug("__createDataSocket: ソケットを作成しました")

            # ソケットオプションの設定
            result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            logging.debug("__createDataSocket: SO_REUSEADDRオプションを設定しました")
            
            # ブロードキャスト受信を有効化
            result.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            logging.debug("__createDataSocket: SO_BROADCASTオプションを設定しました")
            
            # ソケットのバインド - 重要: 空文字列を使用して任意のアドレスからのパケットを受信
            result.bind(('', port))  # 空文字列を使用することが重要
            logging.debug(f"__createDataSocket: ソケットをポート {port} にバインドしました (任意のアドレスからの受信)")
            
            # マルチキャスト設定 - バインド後に行うことが重要
            try:
                # マルチキャストグループに参加
                mreq = socket.inet_aton(self.multicastAddress) + socket.inet_aton(self.localIPAddress)
                result.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                logging.debug("__createDataSocket: IP_ADD_MEMBERSHIPオプションを設定しました")
                
                # マルチキャストTTLの設定
                result.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32) 
                logging.debug("__createDataSocket: IP_MULTICAST_TTLを32に設定しました")
                
                # マルチキャストループバックを有効化
                result.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
                logging.debug("__createDataSocket: IP_MULTICAST_LOOPを有効化しました")
                
                # タイムアウトの設定（デバッグ用）
                # タイムアウトは長めに設定して、データ受信を待つ時間を確保
                result.settimeout(30.0)  # 30秒のタイムアウト
                logging.debug("__createDataSocket: ソケットタイムアウトを30秒に設定しました")
            except Exception as e:
                logging.error(f"__createDataSocket: マルチキャスト設定中にエラーが発生しました: {e}")

            return result
        except Exception as e:
            logging.error(f"__createDataSocket: エラーが発生しました: {e}")
            import traceback
            logging.error("Stack trace:", exc_info=True)
            return None

    # NatNetストリームに接続するためのコマンドソケットを作成
    def __createCommandSocket(self):
        result = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        result.bind(('', 0))
        result.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        return result

    # リジッドボディデータのアンパック
    def __unpackRigidBody(self, data):
        try:
            logging.debug("__unpackRigidBody: リジッドボディデータのアンパックを開始します")
            
            # データのサイズを確認
            logging.debug(f"__unpackRigidBody: データサイズ: {len(data)} バイト")
            
            # ヘッダーバイトを表示
            hex_header = ' '.join([f'{b:02x}' for b in data[:16]])
            logging.debug(f"__unpackRigidBody: 先頭16バイト: {hex_header}")
            
            offset = 0

            # リジッドボディID
            id = int.from_bytes(data[offset:offset+4], byteorder='little')
            offset += 4
            logging.debug(f"__unpackRigidBody: リジッドボディID: {id}")
            trace("ID:", id)

            # 位置と回転
            pos = struct.unpack('fff', data[offset:offset+12])
            offset += 12
            trace("\tPosition:", pos[0], ",", pos[1], ",", pos[2])
            
            rot = struct.unpack('ffff', data[offset:offset+16])
            offset += 16
            trace("\tOrientation:", rot[0], ",", rot[1], ",", rot[2], ",", rot[3])

            logging.debug(f"__unpackRigidBody: 位置: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
            logging.debug(f"__unpackRigidBody: 回転: x={rot[0]:.3f}, y={rot[1]:.3f}, z={rot[2]:.3f}, w={rot[3]:.3f}")

            # コールバック関数が設定されていれば呼び出す
            if self.rigidBodyListener is not None:
                try:
                    self.rigidBodyListener(id, pos, rot)
                    logging.debug(f"__unpackRigidBody: リジッドボディリスナーを呼び出しました: ID={id}")
                except Exception as e:
                    logging.error(f"__unpackRigidBody: リスナー呼び出し中にエラーが発生しました: {e}")

            # マーカー数
            markerCount = int.from_bytes(data[offset:offset+4], byteorder='little')
            offset += 4
            logging.debug(f"__unpackRigidBody: マーカー数: {markerCount}")
            trace("Marker Count:", markerCount)

            # マーカーデータをスキップ
            marker_data_size = markerCount * 3 * 4
            offset += marker_data_size
            logging.debug(f"__unpackRigidBody: マーカーデータをスキップしました ({marker_data_size} バイト)")

            # マーカーIDをスキップ
            marker_id_size = markerCount * 4
            offset += marker_id_size
            logging.debug(f"__unpackRigidBody: マーカーIDをスキップしました ({marker_id_size} バイト)")

            # マーカーサイズをスキップ
            marker_size_size = markerCount * 4
            offset += marker_size_size
            logging.debug(f"__unpackRigidBody: マーカーサイズをスキップしました ({marker_size_size} バイト)")

            # メーンマーカーエラーとトラッキングフラグ
            meanError = struct.unpack('f', data[offset:offset+4])
            offset += 4
            logging.debug(f"__unpackRigidBody: 平均マーカーエラー: {meanError[0]:.6f}")
            trace("Mean marker error:", meanError[0])

            # トラッキングフラグ
            param = int.from_bytes(data[offset:offset+2], byteorder='little')
            offset += 2
            trackingValid = (param & 0x01) != 0
            logging.debug(f"__unpackRigidBody: トラッキング有効: {trackingValid}")
            trace("Tracking Valid:", trackingValid)

            logging.debug("__unpackRigidBody: リジッドボディデータのアンパックが完了しました")
            return offset
        except Exception as e:
            logging.error(f"__unpackRigidBody: エラーが発生しました: {e}")
            logging.error("Stack trace:", exc_info=True)
            return offset

    # データパケットからスケルトンオブジェクトを解析
    def __unpackSkeleton(self, data):
        offset = 0
        
        id = int.from_bytes(data[offset:offset+4], byteorder='little')
        offset += 4
        trace("ID:", id)
        
        rigidBodyCount = int.from_bytes(data[offset:offset+4], byteorder='little')
        offset += 4
        trace("Rigid Body Count:", rigidBodyCount)
        for j in range(0, rigidBodyCount):
            offset += self.__unpackRigidBody(data[offset:])

        return offset

    # モーションキャプチャフレームメッセージからデータを解析
    def __unpackMocapData(self, data):
        try:
            logging.debug("__unpackMocapData: モーションキャプチャデータのアンパックを開始します")
            trace("Begin MoCap Frame\n-----------------\n")

            # データのサイズを確認
            logging.debug(f"__unpackMocapData: データサイズ: {len(data)} バイト")
            
            # ヘッダーバイトを表示
            hex_header = ' '.join([f'{b:02x}' for b in data[:16]])
            logging.debug(f"__unpackMocapData: 先頭16バイト: {hex_header}")
            
            data = memoryview(data)
            offset = 0
            
            # フレーム番号
            frameNumber = int.from_bytes(data[offset:offset+4], byteorder='little')
            offset += 4
            logging.debug(f"__unpackMocapData: フレーム番号: {frameNumber}")
            trace("Frame #:", frameNumber)

            # マーカーセット数
            markerSetCount = int.from_bytes(data[offset:offset+4], byteorder='little')
            offset += 4
            logging.debug(f"__unpackMocapData: マーカーセット数: {markerSetCount}")
            trace("Marker Set Count:", markerSetCount)

            for i in range(markerSetCount):
                # マーカーセット名
                modelName, separator, remainder = bytes(data[offset:]).partition(b'\0')
                offset += len(modelName) + 1
                logging.debug(f"__unpackMocapData: モデル名: {modelName.decode('utf-8')}")
                trace("Model Name:", modelName.decode('utf-8'))
                
                # マーカー数
                markerCount = int.from_bytes(data[offset:offset+4], byteorder='little')
                offset += 4
                logging.debug(f"__unpackMocapData: マーカー数: {markerCount}")
                trace("Marker Count:", markerCount)

                for j in range(markerCount):
                    pos = struct.unpack('fff', data[offset:offset+12])
                    offset += 12
                    #trace("\tMarker", j, ":", pos[0], ",", pos[1], ",", pos[2])
                             
            # 未ラベルマーカー数
            unlabeledMarkersCount = int.from_bytes(data[offset:offset+4], byteorder='little')
            offset += 4
            logging.debug(f"__unpackMocapData: 未ラベルマーカー数: {unlabeledMarkersCount}")
            trace("Unlabeled Markers Count:", unlabeledMarkersCount)

            for i in range(unlabeledMarkersCount):
                pos = struct.unpack('fff', data[offset:offset+12])
                offset += 12
                #trace("\tMarker", i, ":", pos[0], ",", pos[1], ",", pos[2])

            # リジッドボディ数
            rigidBodyCount = int.from_bytes(data[offset:offset+4], byteorder='little')
            offset += 4
            logging.debug(f"__unpackMocapData: リジッドボディ数: {rigidBodyCount}")
            trace("Rigid Body Count:", rigidBodyCount)

            for i in range(rigidBodyCount):
                logging.debug(f"__unpackMocapData: リジッドボディ {i+1}/{rigidBodyCount} のアンパック")
                offset += self.__unpackRigidBody(data[offset:])

            # バージョン 2.1 以降のスケルトンデータ
            # スキップします
            logging.debug("__unpackMocapData: スケルトンデータをスキップします")

            # ラベル付きマーカーセット
            # スキップします
            logging.debug("__unpackMocapData: ラベル付きマーカーセットをスキップします")

            # フォースプレートデータ
            # スキップします
            logging.debug("__unpackMocapData: フォースプレートデータをスキップします")

            # デバイスデータ
            # スキップします
            logging.debug("__unpackMocapData: デバイスデータをスキップします")

            logging.debug("__unpackMocapData: モーションキャプチャデータのアンパックが完了しました")
            trace("End MoCap Frame\n---------------\n")
        except Exception as e:
            logging.error(f"__unpackMocapData: エラーが発生しました: {e}")
            logging.error("Stack trace:", exc_info=True)

        # バージョン2.1以降
        skeletonCount = 0
        if ((self.__natNetStreamVersion[0] == 2 and self.__natNetStreamVersion[1] > 0) or 
            self.__natNetStreamVersion[0] > 2):
            skeletonCount = int.from_bytes(data[offset:offset+4], byteorder='little')
            offset += 4
            trace("Skeleton Count:", skeletonCount)
            for i in range(0, skeletonCount):
                offset += self.__unpackSkeleton(data[offset:])

        # タイムコード
        timecode = int.from_bytes(data[offset:offset+4], byteorder='little')
        offset += 4
        timecodeSub = int.from_bytes(data[offset:offset+4], byteorder='little')
        offset += 4

        # タイムスタンプ
        if ((self.__natNetStreamVersion[0] == 2 and self.__natNetStreamVersion[1] >= 7) or 
            self.__natNetStreamVersion[0] > 2):
            timestamp, = DoubleValue.unpack(data[offset:offset+8])
            offset += 8
        else:
            timestamp, = FloatValue.unpack(data[offset:offset+4])
            offset += 4

        # フレームパラメータ
        param, = struct.unpack('h', data[offset:offset+2])
        isRecording = (param & 0x01) != 0
        trackedModelsChanged = (param & 0x02) != 0
        offset += 2

        # リスナーに情報を送信
        if self.newFrameListener is not None:
            self.newFrameListener(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                                 0, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged)

    # データスレッド関数
    def __dataThreadFunction(self, socket):
        logging.debug(f"__dataThreadFunction: データ受信スレッドを開始しました")
        logging.debug(f"__dataThreadFunction: マルチキャストアドレス={self.multicastAddress}, ローカルアドレス={self.localIPAddress}")
        
        # データ受信カウンター
        packet_count = 0
        last_report_time = time.time()
        wait_count = 0
        
        try:
            logging.debug("__dataThreadFunction: データを待機中...")
            
            while True:
                try:
                    # データ受信を試行
                    data, addr = socket.recvfrom(32768)  # 32kバイトのバッファサイズ
                    
                    # データを受信した場合の処理
                    packet_count += 1
                    wait_count = 0  # 受信成功なら待機カウンターをリセット
                    
                    # 定期的なレポート
                    current_time = time.time()
                    if current_time - last_report_time > 10.0:
                        logging.info(f"__dataThreadFunction: 最後の10秒間に{packet_count}個のパケットを受信しました")
                        packet_count = 0
                        last_report_time = current_time
                    
                    # 各パケットの詳細情報を表示
                    # 最初のパケットまたはレポート直後のパケットは詳細表示
                    if packet_count <= 1 or packet_count % 100 == 0:
                        logging.debug(f"__dataThreadFunction: {addr}から{len(data)}バイトのデータを受信しました (パケット#{packet_count})")
                        # 最初の数バイトをヘキサ表示してデバッグ
                        hex_data = ' '.join([f'{b:02x}' for b in data[:16]])
                        logging.debug(f"__dataThreadFunction: 先頭16バイト: {hex_data}")
                    
                    # データ処理
                    if len(data) > 0:
                        try:
                            self.__processMessage(data)
                        except Exception as e:
                            logging.error(f"__dataThreadFunction: データ処理中にエラーが発生しました: {e}")
                            logging.error("Stack trace:", exc_info=True)
                            
                except socket.timeout:
                    wait_count += 1
                    # 5回のタイムアウトごとにメッセージを表示
                    if wait_count % 5 == 0:
                        logging.debug(f"__dataThreadFunction: ソケットタイムアウトが発生しました ({wait_count}回目)")
                        logging.warning(f"__dataThreadFunction: データが受信されない場合は、以下を確認してください:")
                        logging.warning(f"  - OptiTrack Motiveでデータストリーミングが有効になっているか")
                        logging.warning(f"  - マルチキャストアドレスとポートが正しく設定されているか")
                        logging.warning(f"  - ファイアウォールがUDP通信をブロックしていないか")
                        
                except Exception as e:
                    logging.error(f"__dataThreadFunction: データ受信中にエラーが発生しました: {e}")
                    logging.error("Stack trace:", exc_info=True)
                    time.sleep(1)  # エラー発生時は少し待機
                    
        except Exception as e:
            logging.error(f"__dataThreadFunction: メインループでエラーが発生しました: {e}")
            logging.error("Stack trace:", exc_info=True)

    # メッセージ処理
    def __processMessage(self, data):
        try:
            trace("Begin __processMessage")
            messageID = int.from_bytes(data[0:2], byteorder='little')
            trace("messageID: {0}", messageID)
            packetSize = int.from_bytes(data[2:4], byteorder='little')
            logging.debug(f"__processMessage: メッセージID={messageID}, パケットサイズ={packetSize}")
            
            # メッセージタイプの名前を取得
            message_type_name = "UNKNOWN"
            if messageID == self.NAT_FRAMEOFDATA:
                message_type_name = "NAT_FRAMEOFDATA"
            elif messageID == self.NAT_MODELDEF:
                message_type_name = "NAT_MODELDEF"
            elif messageID == self.NAT_PINGRESPONSE:
                message_type_name = "NAT_PINGRESPONSE"
            elif messageID == self.NAT_RESPONSE:
                message_type_name = "NAT_RESPONSE"
            elif messageID == self.NAT_UNRECOGNIZED_REQUEST:
                message_type_name = "NAT_UNRECOGNIZED_REQUEST"
            elif messageID == self.NAT_MESSAGESTRING:
                message_type_name = "NAT_MESSAGESTRING"
            
            logging.debug(f"__processMessage: メッセージタイプ={message_type_name}")
            
            offset = 4
            if messageID == self.NAT_FRAMEOFDATA:
                logging.debug("__processMessage: フレームデータを受信しました")
                self.__unpackMocapData(data[offset:])            
            elif messageID == self.NAT_MODELDEF:
                trace("Received NAT_MODELDEF")
                self.__unpackModelDef(data[offset:])           
            elif messageID == self.NAT_PINGRESPONSE:
                trace("Received NAT_PINGRESPONSE")
                offset += 256   # Skip the sending app's Name field
                offset += 4     # Skip the sending app's Version info
                self.serverVersion = struct.unpack('BBBB', data[offset:offset+4])
                logging.info(f"__processMessage: サーバーバージョン={self.serverVersion}")
            elif messageID == self.NAT_RESPONSE:
                trace("Received NAT_RESPONSE")
                if packetSize > 4:
                    message, separator, remainder = bytes(data[offset:]).partition(b'\0')
                    logging.info(f"__processMessage: サーバーからのレスポンス: {message.decode('utf-8')}")
            elif messageID == self.NAT_UNRECOGNIZED_REQUEST:
                trace("Received NAT_UNRECOGNIZED_REQUEST")
            elif messageID == self.NAT_MESSAGESTRING:
                trace("Received NAT_MESSAGESTRING")
                message, separator, remainder = bytes(data[offset:]).partition(b'\0')
                logging.info(f"__processMessage: サーバーからのメッセージ: {message.decode('utf-8')}")
                offset += len(message) + 1
                logging.debug(f"__processMessage: サーバーからメッセージを受信しました: {message.decode('utf-8')}")
                trace("Received message from server:", message.decode('utf-8'))
            else:
                logging.warning(f"__processMessage: エラー: 認識されないパケットタイプ {messageID}")
                trace("ERROR: Unrecognized packet type")
                
            trace("End Packet\n----------\n")
        except Exception as e:
            logging.error(f"__processMessage: エラーが発生しました: {e}")
            logging.error("Stack trace:", exc_info=True)
            
    # コマンド送信
    def sendCommand(self, command, commandStr, socket, address):
        # 既知のメッセージ形式でメッセージを作成
        if command == self.NAT_REQUEST_MODELDEF or command == self.NAT_REQUEST_FRAMEOFDATA:
            packetSize = 0
            commandStr = ""
        elif command == self.NAT_REQUEST:
            packetSize = len(commandStr) + 1
        elif command == self.NAT_PING:
            commandStr = "Ping"
            packetSize = len(commandStr) + 1

        data = command.to_bytes(2, byteorder='little')
        data += packetSize.to_bytes(2, byteorder='little')
        
        data += commandStr.encode('utf-8')
        data += b'\0'

        socket.sendto(data, address)
        
    # 実行
    def run(self):
        logging.info(f"NatNetClient: サーバーアドレス={self.serverIPAddress}, ローカルアドレス={self.localIPAddress}")
        logging.info(f"NatNetClient: マルチキャストアドレス={self.multicastAddress}, コマンドポート={self.commandPort}, データポート={self.dataPort}")
        
        # データソケットを作成
        try:
            self.dataSocket = self.__createDataSocket(self.dataPort)
            if self.dataSocket is None:
                logging.error("NatNetClient: データチャネルを開けませんでした")
                return False
            logging.info("NatNetClient: データソケットを作成しました")
        except Exception as e:
            logging.error(f"NatNetClient: データソケットの作成中にエラーが発生しました: {e}")
            return False

        # コマンドソケットを作成
        try:
            self.commandSocket = self.__createCommandSocket()
            if self.commandSocket is None:
                logging.error("NatNetClient: コマンドチャネルを開けませんでした")
                return False
            logging.info("NatNetClient: コマンドソケットを作成しました")
        except Exception as e:
            logging.error(f"NatNetClient: コマンドソケットの作成中にエラーが発生しました: {e}")
            return False

        # データパケット受信用の別スレッドを作成
        logging.info("NatNetClient: データスレッドを作成します")
        dataThread = Thread(target=self.__dataThreadFunction, args=(self.dataSocket,))
        dataThread.daemon = True
        dataThread.start()

        # コマンドパケット受信用の別スレッドを作成
        logging.info("NatNetClient: コマンドスレッドを作成します")
        commandThread = Thread(target=self.__dataThreadFunction, args=(self.commandSocket,))
        commandThread.daemon = True
        commandThread.start()

        # サーバーにPINGを送信
        logging.info(f"NatNetClient: サーバー {self.serverIPAddress}:{self.commandPort} にPINGを送信します")
        self.sendCommand(self.NAT_PING, "", self.commandSocket, (self.serverIPAddress, self.commandPort))
        logging.info("NatNetClient: PINGを送信しました")
        
        return True
        
    # サーバーIPアドレスを設定
    def set_server_address(self, server_address):
        self.serverIPAddress = server_address
        
    # ローカルIPアドレスを設定
    def set_local_address(self, local_address):
        self.localIPAddress = local_address
        
    # クライアントをシャットダウン
    def shutdown(self):
        """クライアントをシャットダウンし、ソケットを閉じる"""
        try:
            if hasattr(self, 'dataSocket') and self.dataSocket:
                self.dataSocket.close()
            if hasattr(self, 'commandSocket') and self.commandSocket:
                self.commandSocket.close()
            logging.info("NatNetClient: ソケットを閉じました")
        except Exception as e:
            logging.error(f"NatNetClient: シャットダウン中にエラーが発生しました: {e}")
