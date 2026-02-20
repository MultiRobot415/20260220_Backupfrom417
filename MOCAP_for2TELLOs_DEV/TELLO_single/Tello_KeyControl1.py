# -*- coding: utf-8 -*-
#telloモジュールのTelloクラスをインポートしている．
from djitellopy import Tello
import KeyPressModule as kp
import cv2
from time import sleep
kp.init()
#Telloクラスをインスタンス化．サンプルコードではtello.Tello()としていて，これはimportの仕方による違い
me = Tello()
me.connect()
#me.streamon()
print("接続OKです")
print(me.get_battery())

def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 100
    if kp.getKey("LEFT"): lr = -speed
    elif kp.getKey("RIGHT"): lr = speed

    if kp.getKey("UP"): fb = speed
    elif kp.getKey("DOWN"): fb = -speed

    if kp.getKey("w"): ud = speed
    elif kp.getKey("s"): ud = -speed

    if kp.getKey("a"): yv = -speed
    elif kp.getKey("d"): yv = speed

    if kp.getKey("e"): me.land()
    if kp.getKey("q"): me.takeoff()
    if kp.getKey("escape"): me.emergency()

    return [lr, fb, ud, yv]
while True:
    #制御信号
    vals = getKeyboardInput()

    #上が本命（実際に飛ばすとき），下はデバッグ用にすべて0出力    
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    #me.send_rc_control(0, 0, 0, 0)

    print(vals)
    sleep(0.05)
    
    # 映像表示部分をコメントアウト
    #img = me.get_frame_read().frame # 映像を取得
    #img = cv2.resize(img, (960, 720)) # 画像サイズを調整
    #cv2.imshow("Tello", img) # 画像を表示
    #cv2.waitKey(1) # 1ms待つ
