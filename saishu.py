
import cv2
from dynamixel_sdk import *
from dynamixel_sdk import Protocol2PacketHandler as P2PH                    # Uses Dynamixel SDK library
from dynamixel_sdk import PortHandler
from dynamixel_sdk import PacketHandler
import time
import threading

import numpy as np
import sys, select, os
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

import colorsys


 

# Control table address
AX_TORQUE_ENABLE       = 24                          # Control table address is different in Dynamixel model
AX_GOAL_POSITION       = 30
AX_PRESENT_POSITION    = 36
AX_MOVING_SPEED        = 32
AX_MOVING              = 46
# Protocol version
PROTOCOL_VERSION            = 1                             # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 15                            #アーム
DXL_ID2                     = 8                             #アーム
DXL_ID3                     = 17                            #アーム
DXL_ID4                     = 10                            #左右
DXL_ID5                     = 3                             #上下
DXL_ID6                     = 13                            #上下
BAUDRATE                    = 1000000
DEVICENAME                  = "COM4"                # Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed


DXL_GOAL                    = [720,1000,513,350,512,512]            # モータの角度
DXL_MOVING                  = [400,400,400,100,100,100]         # モータの速度
RELOAD = 0                                                  # リロードの段階

start_time                  = 0                             # 測り始め
stop_time                   = 0                             # 測り終わり
delta                       = 0                             #stop_timeとstart_timeの差

circle                      = 0                             # 的を検出したか
all_target                  = 0                             # 的の初期個数

TAS                         = 0
LEFT_TARGET_XY              = [0,0]
TAS_RELOAD                  = 0
size                        = (0,0)
COPY_circle                 = 0

START_TIME                  = 0                             # タイマーの測り始め
TIME_EATER                  = 0                             # タイマーを減らす
TIME_LIMIT                  = 120                            # 制限時間
TIMER                       = TIME_LIMIT                    # 残り何秒か
MIN                         = 0                             # 分
SEC                         = 0                             # 秒

h                           = 0
s                           = 0
v                           = 0

GAME_STATE                  = 0                             # ゲームの段階

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHan662lerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)
cap = cv2.VideoCapture(0) #cameraの設定

def onMouse(event, x, y, flags, param):
    global h, s, v
    if GAME_STATE == 0.5:
        if event == cv2.EVENT_LBUTTONDOWN:
            color = img[y,x]
            r = color[2]
            g = color[1]
            b = color[0]
            hsv = colorsys.rgb_to_hsv(r/255.0,g/255.0,b/255.0)
            h = int(hsv[0]*180)
            s = int(hsv[1]*255)
            v = int(hsv[2]*255)
            #print('HSV: %d,%d,%d' % (h,s,v))
        
def yellow(img):
    global circle, GAME_STATE, h, s, v, size, LEFT_TARGET_XY, COPY_circle
    # 色基準で2値化する。
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, np.array([145, 70, 0]), np.array([180, 255, 255]))
    yellow = cv2.inRange(hsv, np.array([h-15,s-40, v-100]), np.array([h+15,s+40, v+100]))
    blue = cv2.inRange(hsv, np.array([108, 121, 0]), np.array([120, 255, 255]))

    # 色の範囲を指定する
    lower_color = np.array([h-15, s-35, v-35])
    upper_color = np.array([h+15, s+35, v+35])

    # 指定した色に基づいたマスク画像の生成
    mask = cv2.inRange(hsv, lower_color, upper_color)
    output = cv2.bitwise_and(hsv, hsv, mask = mask)


    img_origin=output
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, img_binary = cv2.threshold(img_gray, 180, 255,cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    """
    for i in contours:
        if cv2.contourArea(i) < 50:
            continue
        if GAME_STATE == 1:
            cv2.polylines(img, i, True, (0,255,0),5)           # 検出した物の輪郭
            """

    area_num = 950
    contours_filtered = list(filter(lambda x: cv2.contourArea(x) > area_num, contours))

    circle = 0                # いくつ的があるか
    x=0
    y=0
    for i in contours_filtered:
        circle +=1
        x,y,width,height = cv2.boundingRect(i)
        if GAME_STATE == 1 or GAME_STATE == 0.5:
            cv2.rectangle(img, (x,y),(x+width, y+height), color = (0,255,0), thickness =2 )
        if TAS == 1 and GAME_STATE == 3:
            if LEFT_TARGET_XY[0] > x+width/2:   # 左端の的なら、
                LEFT_TARGET_XY[0] = x+width/2   # 左端の的のX座標を記録
                LEFT_TARGET_XY[1] = y+height/2  # 左端の的のＹ座標を記録
    #cv2.putText(img_origin,'target',(x+10,y+10), cv2.FONT_HERSHEY_PLAIN, 1,(0,0,0),1,cv2.LINE_AA)


    #print("あなたの点数は"+str(circle)+"点です")
    imgyellow = np.asarray(mask)
    return img
            

class DXL():

    def __init__(self):

        # Open port
        if portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()


        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, AX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")



    def moveDXL(self):
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_GOAL_POSITION, DXL_GOAL[0])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_GOAL_POSITION, DXL_GOAL[1])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_GOAL_POSITION, DXL_GOAL[2])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_GOAL_POSITION, DXL_GOAL[3])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID5, AX_GOAL_POSITION, DXL_GOAL[4])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID6, AX_GOAL_POSITION, DXL_GOAL[5])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, DXL_MOVING[0])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, DXL_MOVING[1])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, DXL_MOVING[2])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, DXL_MOVING[3])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID5, AX_MOVING_SPEED, DXL_MOVING[4])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID6, AX_MOVING_SPEED, DXL_MOVING[5])

    
################## Main Loop ################################################
"""
                DXL_GOAL[0] = 700
                DXL_GOAL[1] = 819
                DXL_GOAL[2] = 205
                307で90度
"""


if __name__ == "__main__":
    
    dx = DXL()
#205,512,819
    try:
        while True:
            KEY = cv2.waitKey(1) & 0xff     # 何を入力したか
            if KEY == 27:             # Escで強制終了
                break
            
            #ret, frame = cap.read()#　カメラの読み込み
            #img = frame
            _, img = cap.read()
            size = (img.shape[1]//2, img.shape[0]//2)
            #img = cv2.resize(img, size)
            #print("press ESC to quit!")
            cv2.namedWindow('Camera', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback("Camera", onMouse)
            
            
            if GAME_STATE == 0:                                 # ゲーム開始画面
                DXL_GOAL                    = [720,1000,513,350,512,512]            # モータの角度
                DXL_MOVING                  = [400,400,400,100,100,100]         # モータの速度
                RELOAD                      = 0                                 # リロードの段階
                TAS                         = 0
                TAS_RELOAD                  = 0
                LEFT_TARGET_XY = [3*size[0], 3*size[1]]
                cv2.putText(img,
                            text="SHATEKI",
                            org=(size[0]-197, 3+2*size[1]//3),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=3.0,
                            color=(0, 0, 90),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                cv2.putText(img,
                            text="SHATEKI",
                            org=(size[0]-200, 2*size[1]//3),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=3.0,
                            color=(0, 152, 243),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                cv2.putText(img,
                            text="-Defeat All Targets!-",
                            org=(size[0]-277, 3+size[1]),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=1.5,
                            color=(0, 0, 90),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                cv2.putText(img,
                            text="-Defeat All Targets!-",
                            org=(size[0]-280, size[1]),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=1.5,
                            color=(0, 152, 243),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                cv2.putText(img,
                            text='Press Space Bar',
                            org=(size[0]-128, 2+3*size[1]//2),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=1.0,
                            color=(255, 255, 255),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                cv2.putText(img,
                            text='Press Space Key',
                            org=(size[0]-130, 3*size[1]//2),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=1.0,
                            color=(0, 0, 0),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                if KEY == 32:                           # スペースで的がいくつあるか検出
                    #img = yellow(img)
                    #all_target = circle
                    GAME_STATE = 0.5
            elif GAME_STATE == 0.5:
                cv2.putText(img,
                            text=("Click on one of the targets."),
                            org=(size[0]-258, 2+2*size[1]-60),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=1.0,
                            color=(255, 255, 255),
                            thickness=2,
                            lineType=cv2.LINE_AA)                 
                cv2.putText(img,
                            text=("Click on one of the targets."),
                            org=(size[0]-260, 2*size[1]-60),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=1.0,
                            color=(0, 0, 0),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                cv2.putText(img,
                            text="Press Space Bar after selecting the color.",
                            org=(size[0]-288, 2+2*size[1]-20),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=0.8,
                            color=(255, 255, 255),
                            thickness=2,
                            
                            lineType=cv2.LINE_AA)
                cv2.putText(img,
                            text="Press Space Bar after selecting the color.",
                            org=(size[0]-290, 2*size[1]-20),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=0.8,
                            color=(0, 0, 0),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                img = yellow(img)
                if KEY == 32:                           # スペースで的がいくつあるか検出
                    all_target = circle
                    GAME_STATE = 1
            elif GAME_STATE == 1:                               # 的の数の検出
                img = yellow(img)
                if(all_target == 0):            # 的が見つからなかった場合
                    GAME_STATE = -1         # エラー処理1
                else:   
                    if(all_target == 1):          # 的を1つ検出した場合
                        cv2.putText(img,
                                    text=(str(all_target) + " target was detected."),
                                    org=(2+size[0]-178, 2+size[1]//2),
                                    fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                    fontScale=1.0,
                                    color=(255, 255, 255),
                                    thickness=2,
                                    lineType=cv2.LINE_AA)
                        cv2.putText(img,
                                    text=(str(all_target) + " target was detected."),
                                    org=(size[0]-180, size[1]//2),
                                    fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                    fontScale=1.0,
                                    color=(0, 0, 0),
                                    thickness=2,
                                    lineType=cv2.LINE_AA)
                    elif(all_target >= 2):          # 的を複数検出した場合
                        cv2.putText(img,
                                    text=(str(all_target) + " targets were detected."),
                                    org=(size[0]-178, 2+size[1]//2),
                                    fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                    fontScale=1.0,
                                    color=(255, 255, 255),
                                    thickness=2,
                                    lineType=cv2.LINE_AA)
                        cv2.putText(img,
                                    text=(str(all_target) + " targets were detected."),
                                    org=(size[0]-180, size[1]//2),
                                    fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                    fontScale=1.0,
                                    color=(0, 0, 0),
                                    thickness=2,
                                    lineType=cv2.LINE_AA)
                    cv2.putText(img,
                                text="Are You Ready? (Y/N)",
                                org=(size[0]-168, 2+size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=1.0,
                                color=(255, 255, 255),
                                thickness=2,
                                
                                lineType=cv2.LINE_AA)
                    cv2.putText(img,
                                text="Are You Ready? (Y/N)",
                                org=(size[0]-170, size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=1.0,
                                color=(255, 0, 0),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                    if KEY == int(ord('y')):                # Yes
                        GAME_STATE = 2
                        TIME_LIMIT = all_target * 45
                        start_time = time.time()    # 時間を測り始める
                        stop_time = start_time      # stop_timeとstart_timeの差を0にする
                    elif KEY == int(ord('n')):              # No
                        GAME_STATE = 0
                    elif KEY == int(ord('t')):                # TAS
                        TAS = 1
                        GAME_STATE = 2
                        TIME_LIMIT = all_target * 45
                        start_time = time.time()    # 時間を測り始める
                        stop_time = start_time      # stop_timeとstart_timeの差を0にする
            elif GAME_STATE == -1:                  # エラー処理1（的を検出しなかった)
                    cv2.putText(img,
                                text=("No target was detected. (Space)"),
                                org=(size[0]-258, 2+size[1]//2),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=1.0,
                                color=(0, 0, 0),
                                thickness=2,
                                lineType=cv2.LINE_AA)                 
                    cv2.putText(img,
                                text=("No target was detected. (Space)"),
                                org=(size[0]-260, size[1]//2),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=1.0,
                                color=(0, 0, 255),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                    if KEY == 32:
                        GAME_STATE = 0
            elif GAME_STATE == 2:                               # カウントダウン
                stop_time = time.time()
                if delta < 1:
                    cv2.putText(img,
                                text="3",
                                org=(size[0]-28, 2+size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=3.0,
                                color=(0, 0, 90),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                    cv2.putText(img,
                                text="3",
                                org=(size[0]-30, size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=3.0,
                                color=(0, 152, 243),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                elif 1 <= delta < 2:
                    cv2.putText(img,
                                text="2",
                                org=(size[0]-28, 2+size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=3.0,
                                color=(0, 0, 90),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                    cv2.putText(img,
                                text="2",
                                org=(size[0]-30, size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=3.0,
                                color=(0, 152, 243),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                elif 2 <= delta < 3:
                    cv2.putText(img,
                                text="1",
                                org=(size[0]-28, 2+size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=3.0,
                                color=(0, 0, 90),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                    cv2.putText(img,
                                text="1",
                                org=(size[0]-30, size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=3.0,
                                color=(0, 152, 243),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                elif 3 <= delta < 4:
                    cv2.putText(img,
                                text="START",
                                org=(size[0]-197, 3+size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=4.0,
                                color=(0, 0, 90),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                    cv2.putText(img,
                                text="START",
                                org=(size[0]-200, size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=4.0,
                                color=(0, 152, 243),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                elif 4 <= delta:
                    START_TIME = time.time()                        # タイマーを測り始める
                    TIME_EATER = START_TIME                         # 揃える
                    GAME_STATE = 3
            elif GAME_STATE == 3:
                img = yellow(img)
                TIME_EATER = time.time()
                TIMER = int(TIME_LIMIT - TIME_EATER + START_TIME +0.999)    # 残り時間の計算
                MIN = int(TIMER/60) # 分
                SEC = TIMER-MIN*60  # 秒
                if MIN == 0 and SEC <= 0:                   # 間に合わなかったらゲームオーバー
                    GAME_STATE = 4
                else:
                    if MIN < 10:
                        MIN = "0" + str(MIN)    # 1桁なら0を付ける
                    if SEC < 10:
                        SEC = "0" + str(SEC)    # 1桁なら0を付ける
                    cv2.putText(img,
                                text= str(MIN)+":"+str(SEC),
                                org=(27, 52),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=1.1,
                                color=(255, 255, 255),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                    cv2.putText(img,
                                text= str(MIN)+":"+str(SEC),
                                org=(25, 50),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=1.1,
                                color=(0, 0, 0),
                                thickness=2,
                                lineType=cv2.LINE_AA)

                cv2.putText(img,
                            text= "Remaining",
                            org=(2*size[0]-168,37),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=0.8,
                            color=(255, 255, 255),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                cv2.putText(img,
                            text= "Remaining",
                            org=(2*size[0]-170,35),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=0.8,
                            color=(0, 0, 0),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                    
                cv2.putText(img,
                            text= "targets:",
                            org=(2*size[0]-168,67),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=0.8,
                            color=(255, 255, 255),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                cv2.putText(img,
                            text= "targets:",
                            org=(2*size[0]-170,65),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=0.8,
                            color=(0, 0, 0),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                cv2.putText(img,
                            text= str(circle),
                            org=(2*size[0]-58,77),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=1.2,
                            color=(0, 0, 0),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                cv2.putText(img,
                            text= str(circle),
                            org=(2*size[0]-60,75),
                            fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                            fontScale=1.2,
                            color=(0, 150, 255),
                            thickness=2,
                            lineType=cv2.LINE_AA)
                
                if circle == 0:                             # 全て倒したらゲームクリア
                    GAME_STATE = 5
                
                if TAS == 1:
                    if TAS_RELOAD == -1:
                        LEFT_TARGET_XY = [3*size[0], 3*size[1]]
                        TAS_RELOAD = 0
                    elif TAS_RELOAD == 0 and TAS == 1:
                        if   0<= LEFT_TARGET_XY[0]              < 2*size[0]//13:    # X座標
                            DXL_GOAL[3] = 415
                        elif 2*size[0]//13 <= LEFT_TARGET_XY[0] < 4*size[0]//13:
                            DXL_GOAL[3] = 405
                        elif 4*size[0]//13 <= LEFT_TARGET_XY[0] < 6*size[0]//13:
                            DXL_GOAL[3] = 395
                        elif 6*size[0]//13 <= LEFT_TARGET_XY[0] < 8*size[0]//13:
                            DXL_GOAL[3] = 385
                        elif 8*size[0]//13 <= LEFT_TARGET_XY[0] < 10*size[0]//13:
                            DXL_GOAL[3] = 375
                        elif 10*size[0]//13 <= LEFT_TARGET_XY[0] < 12*size[0]//13:
                            DXL_GOAL[3] = 365
                        elif 12*size[0]//13 <= LEFT_TARGET_XY[0] < 14*size[0]//13:
                            DXL_GOAL[3] = 355
                        elif 14*size[0]//13 <= LEFT_TARGET_XY[0] < 16*size[0]//13:
                            DXL_GOAL[3] = 345
                        elif 16*size[0]//13 <= LEFT_TARGET_XY[0] < 18*size[0]//13:
                            DXL_GOAL[3] = 335
                        elif 18*size[0]//13 <= LEFT_TARGET_XY[0] < 20*size[0]//13:
                            DXL_GOAL[3] = 325
                        elif 20*size[0]//13 <= LEFT_TARGET_XY[0] < 22*size[0]//13:
                            DXL_GOAL[3] = 315
                        elif 22*size[0]//13 <= LEFT_TARGET_XY[0] < 24*size[0]//13:
                            DXL_GOAL[3] = 305
                        elif 24*size[0]//13 <= LEFT_TARGET_XY[0] <= 26*size[0]//13:
                            DXL_GOAL[3] = 295
                            
                        if   0<= LEFT_TARGET_XY[1]              < 2*size[1]//7:    # Y座標
                            DXL_GOAL[4] = 512
                            DXL_GOAL[5] = 512
                        elif 2*size[1]//7 <= LEFT_TARGET_XY[1] < 4*size[1]//7:
                            DXL_GOAL[4] = 502
                            DXL_GOAL[5] = 522
                        elif 4*size[1]//7 <= LEFT_TARGET_XY[1] < 6*size[1]//7:
                            DXL_GOAL[4] = 492
                            DXL_GOAL[5] = 532
                        elif 6*size[1]//7 <= LEFT_TARGET_XY[1] < 8*size[1]//7:
                            DXL_GOAL[4] = 482
                            DXL_GOAL[5] = 542
                        elif 8*size[1]//7 <= LEFT_TARGET_XY[1] < 10*size[1]//7:
                            DXL_GOAL[4] = 472
                            DXL_GOAL[5] = 552
                        elif 10*size[1]//7 <= LEFT_TARGET_XY[1] < 12*size[1]//7:
                            DXL_GOAL[4] = 462
                            DXL_GOAL[5] = 562
                        elif 12*size[1]//7 <= LEFT_TARGET_XY[1] < 14*size[1]//7:
                            DXL_GOAL[4] = 452
                            DXL_GOAL[5] = 572
                        
                        TAS_RELOAD = 1
                            
                        
                if ((TAS == 0 and RELOAD == 0 and KEY == int(ord('r'))) or TAS_RELOAD == 1):          #rで前に倒れる
                    start_time = time.time()    # 時間を測り始める
                    stop_time = start_time      # stop_timeとstart_timeの差を0にする
                    delta = 0                   # deltaをリセット
                    DXL_MOVING[0] = 300
                    DXL_MOVING[1] = 300
                    DXL_MOVING[2] = 300
                    DXL_GOAL[0] = 712
                    DXL_GOAL[1] = 698
                    DXL_GOAL[2] = 736
                    RELOAD = 1 
                    if TAS == 1:
                        TAS_RELOAD = 2
                elif ((RELOAD == 1 or TAS_RELOAD == 2) and delta <= 0.5):                     # 倒れるのを待つ
                    stop_time = time.time()     # 時間を測る
                elif ((RELOAD == 1 or TAS_RELOAD == 2) and delta > 0.5):                      # フックを掛ける
                    start_time = time.time()    # 時間を測り始める
                    stop_time = start_time      # stop_timeとstart_timeの差を0にする
                    delta = 0                   # deltaをリセット
                    DXL_MOVING[0] = 500
                    DXL_GOAL[0] = 312
                    RELOAD = 2
                    if TAS == 1:
                        TAS_RELOAD = 3
                elif ((RELOAD == 2 or TAS_RELOAD == 3) and delta <= 0.5):                     # 掛かるのを待つ
                    stop_time = time.time()     # 時間を測る
                elif ((RELOAD == 2 or TAS_RELOAD == 3) and delta > 0.5):                      # 引っ張る
                    start_time = time.time()    # 時間を測り始める
                    stop_time = start_time      # stop_timeとstart_timeの差を0にする
                    delta = 0                   # deltaをリセット
                    DXL_MOVING[0] = 500
                    DXL_MOVING[1] = 600
                    DXL_MOVING[2] = 600
                    DXL_GOAL[0] = 307
                    DXL_GOAL[1] = 1000
                    DXL_GOAL[2] = 513
                    RELOAD = 3
                    if TAS == 1:
                        TAS_RELOAD = 4
                elif ((RELOAD == 3 or TAS_RELOAD == 4) and delta <= 0.3):                      # 引っ張るのを待つ
                    stop_time = time.time()     # 時間を測る
                elif ((RELOAD == 3 or TAS_RELOAD == 4) and delta > 0.3):                     # 発射準備完了
                    start_time = time.time()    # 時間を測り始める
                    stop_time = start_time      # stop_timeとstart_timeの差を0にする
                    delta = 0                   # deltaをリセット
                    RELOAD = 4
                    if TAS == 1:
                        TAS_RELOAD = 5
                elif ((RELOAD == 4 and KEY == 32) or TAS_RELOAD == 5):                       # リロードしてる状態ならスペースで発射
                    start_time = 0              # 時間を測り始める
                    stop_time = start_time      # stop_timeとstart_timeの差を0にする
                    delta = 0                   # deltaをリセット
                    DXL_MOVING[0] = 2047
                    DXL_GOAL[0] = 720
                    RELOAD = 5
                    if TAS == 1:
                        TAS_RELOAD = 6
                elif ((RELOAD == 5 or TAS_RELOAD == 6) and delta <= 0.05):                     # 発射を待つ
                    stop_time = time.time()     # 時間を測る
                elif ((RELOAD == 5 or TAS_RELOAD == 6) and delta > 0.05):                    # 発射終了
                    start_time = time.time()    # start_timeをリセット
                    stop_time = start_time      # stop_timeをリセット
                    delta = 0                   # deltaをリセット
                    RELOAD = 0
                    if TAS == 1:
                        TAS_RELOAD = -1
                
                if TAS == 0:
                    if(KEY== int(ord('a')) and DXL_GOAL[3]<500):     # 左(15回まで)
                                DXL_MOVING[3] = 50
                                DXL_GOAL[3] += 10
                    elif(KEY == int(ord('d')) and DXL_GOAL[3]>200):   # 右(15回まで)
                            DXL_MOVING[3] = 50
                            DXL_GOAL[3] -= 10
                    
                    if(KEY== int(ord('w')) and DXL_GOAL[4]<562 and DXL_GOAL[5]>462):     # 上(5回まで)
                                DXL_MOVING[4] = 50
                                DXL_MOVING[5] = 50
                                DXL_GOAL[4] += 10
                                DXL_GOAL[5] -= 10
                    elif(KEY == int(ord('s')) and DXL_GOAL[4]>482 and DXL_GOAL[5]<542):   # 下(3回まで)
                            DXL_MOVING[4] = 50
                            DXL_MOVING[5] = 50
                            DXL_GOAL[4] -= 10
                            DXL_GOAL[5] += 10
                    if(KEY== int(ord('z'))):                                                #初期位置に戻す
                        DXL_GOAL[3] = 350
                        DXL_GOAL[4] = 512
                        DXL_GOAL[5] = 512
            elif GAME_STATE == 4:                           # ゲームオーバー
                    cv2.putText(img,
                                text="GAME OVER",
                                org=(size[0]-247, 3+size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=2.7,
                                color=(0, 0, 100),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                    cv2.putText(img,
                                text="GAME OVER",
                                org=(size[0]-250, size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=2.7,
                                color=(0, 0, 255),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                    if KEY == 32:                           # スペースでタイトル画面に戻る
                        GAME_STATE = 0
            elif GAME_STATE == 5:                           # ゲームクリア
                    cv2.putText(img,
                                text="Complete!",
                                org=(size[0]-217, 3+size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=2.7,
                                color=(0, 100, 0),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                    cv2.putText(img,
                                text="Complete!",
                                org=(size[0]-220, size[1]),
                                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                                fontScale=2.7,
                                color=(0, 255, 0),
                                thickness=2,
                                lineType=cv2.LINE_AA)
                    if KEY == 32:                           # スペースでタイトル画面に戻る
                        GAME_STATE = 0
            delta = stop_time - start_time                          # 差を計算
            dx.moveDXL()# dynamixelを動かすメソッド
            
            
            cv2.imshow('Camera',img)
            
            
##############################################################################

    finally:
        # Close port
        cap.release()
        cv2.destroyAllWindows()
        portHandler.closePort()