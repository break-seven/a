#!/usr/bin/python3
# coding=utf8
import sys
#sys.path.append('C:\\Users\\13231\\Desktop\\sourcecode\\ArmPi')
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
#检查python版本
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
#初始化一个python类
AK = ArmIK()
#创建颜色字典
range_rgb = {
    'red':   (0, 0, 255),
    'blue':  (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red')
# 设置检测颜色，输入颜色，更改__target_color，返回true
def setTargetColor(target_color):
    global __target_color

    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

#找出面积最大的轮廓
#参数为要比较的轮廓的列表
def getAreaMaxContour(contours) :
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours : #历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  #计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  #只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c

        return area_max_contour, contour_area_max  #返回最大的轮廓

# 夹持器夹取时闭合的角度
servo1 = 500

# 初始位置
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)#驱动转到指定位置，舵机id，位置何转动需要时间
    Board.setBusServoPulse(2, 500, 500)#
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)#转过去或者返回false，俯仰角范围可以调整

def setBuzzer(timer):#我觉得这个函数有点大病
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

#设置扩展板的RGB灯颜色使其跟要追踪的颜色一致
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()
#初始化一大堆
count = 0#
_stop = False #用来确定是否停止
color_list = []
get_roi = False
get_layer = False
__isRunning = False #用来查看是否运行
detect_color = 'None'
detect_layer = 'None'
detect_direct = 'None'
start_pick_up = False
start_count_t1 = True
#格式化
def reset():
    global _stop
    global count
    global get_roi
    global get_layer
    global color_list
    global detect_color
    global detect_layer
    global detect_direct
    global start_pick_up
    global __target_color
    global start_count_t1

    count = 0
    _stop = False
    color_list = []
    get_roi = False
    get_layer = False
    __target_color = ()
    detect_color = 'None'
    detect_layer = 'None'
    detect_direct = 'None'
    start_pick_up = False
    start_count_t1 = True
#初始化
def init():
    print("test Init")
    initMove()#转到初始位置
#start
def start():
    global __isRunning
    reset()#重置
    __isRunning = True
    print("test Start")
#停止
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("test Stop")
#并没有释放什么东西
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("test Exit")
#一些参数
rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False #用来判断能不能到达某个点
world_X, world_Y = 0, 0
#move函数
def move():
    global rect
    global _stop
    global get_roi
    global get_layer
    global unreachable
    global __isRunning
    global detect_color
    global detect_layer
    global detect_direct
    global start_pick_up
    global rotation_angle
    global world_X, world_Y
    
    #放置坐标,等会测试一下
    '''coordinate = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
    }'''
    coordinate = {
    'one': {'left':(-28 + 0.5,15 - 0.5,5), 'mid':(-28 + 0.5,15 - 0.5,5), 'right':(-28 + 0.5,15 - 0.5,5)},
    'two': {'left':(-28 + 0.5,10 - 0.5,10), 'mid':(-28 + 0.5,10 - 0.5,10), 'right':(-28 + 0.5,10 - 0.5,10)},
    'three': {'left':(-28 + 0.5,5 - 0.5,15), 'mid':(-28 + 0.5,5 - 0.5,15), 'right':(-28 + 0.5,5 - 0.5,15)},
    }
    while True:
        if __isRunning: 
                   
            if detect_color != 'None' and start_pick_up and detect_layer != "None":  #如果检测到方块没有移动一段时间后，开始夹取
                print('1')
                #移到目标位置，高度6cm, 通过返回的结果判断是否能到达指定位置
                #如果不给出运行时间参数，则自动计算，并通过结果返回
                set_rgb(detect_color)#设置显示灯
                setBuzzer(0.1)#有点病的程序
                result = AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0)  #移到目标位置返回角度俯仰角等
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                    time.sleep(result[2]/1000) #如果可以到达指定位置，则获取运行时间

                    if not __isRunning:
                        continue
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle) #计算夹持器需要旋转的角度
                    Board.setBusServoPulse(1, servo1 - 500, 500)  #1号动，角度固定
                    Board.setBusServoPulse(2, servo2_angle, 500)#2号动，角度计算
                    time.sleep(0.5)#休息
                    
                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((world_X, world_Y, 1.5), -90, -90, 0, 1000)#转到A点(world_X, world_Y, 1.5)
                    time.sleep(1.5)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1-100, 500)  #1号动，夹持器闭合，角度固定
                    time.sleep(0.8)
                    Board.setBusServoPulse(2, 500, 500)#2号动，角度固定腕部转动
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  #机械臂抬起，转到B点(world_X, world_Y, 12)
                    time.sleep(1)

                    if not __isRunning:
                        continue
                    result = AK.setPitchRangeMoving((coordinate[detect_layer][detect_direct][0], coordinate[detect_layer][detect_direct][1], 12), -90, -90, 0) #计算并转到目标1点(coordinate[detect_color][0], coordinate[detect_color][1], 12) 
                    time.sleep(result[2]/1000)#如果可以到达指定位置，则获取运行时间
                    
                    if not __isRunning:
                        continue                   
                    servo2_angle = getAngle(coordinate[detect_layer][detect_direct][0], coordinate[detect_layer][detect_direct][1], -90)#计算夹持器需要旋转的角度
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_layer][detect_direct][0]+6, coordinate[detect_layer][detect_direct][1], coordinate[detect_layer][detect_direct][2] + 3), 0, -10, 10, 1000)#计算并转到目标2点上方(coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        continue                    
                    AK.setPitchRangeMoving((coordinate[detect_layer][detect_direct][0]+6, coordinate[detect_layer][detect_direct][1], coordinate[detect_layer][detect_direct][2]), 0, -10, 10, 1000)#转到目标2点前方(coordinate[detect_color])
                    time.sleep(0.8)

                    if not __isRunning:
                        continue                    
                    AK.setPitchRangeMoving((coordinate[detect_layer][detect_direct]), 0, -10, 10, 1000)#转到目标2点(coordinate[detect_color])
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1 - 500, 500)  # 1号动，固定角度,打开
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_layer][detect_direct][0], coordinate[detect_layer][detect_direct][1], 12), -90, -90, 0, 800)#到了3点有点奇怪，(coordinate[detect_color][0], coordinate[detect_color][1], 12)
                    time.sleep(0.8)

                    initMove()  # 回到初始位置
                    time.sleep(1.5)
                    #全部初始化
                    detect_color = 'None'
                    detect_layer = 'None'
                    detect_direct = 'None'
                    get_roi = False
                    get_layer = False
                    start_pick_up = False
                    set_rgb(detect_color)
        else:
            if _stop:#停止了
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)#1号动，固定角度
                time.sleep(0.5)#休息
                Board.setBusServoPulse(2, 500, 500)#2号动，固定角度
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)#夹持角度
                time.sleep(1.5)
            time.sleep(0.01)
          
#运行子线程
th = threading.Thread(target=move)#运行move
th.setDaemon(True)#
th.start()#线程开始运行了  

t1 = 0
roi = ()
codeinfo = ' '
points = np.zeros((2,3))
straight_qrcode =np.zeros((2,3))
center_list = []
last_x, last_y = 0, 0
draw_color = range_rgb["black"]
qrcoder = cv2.QRCodeDetector()
def run(img):
    global roi
    global codeinfo
    global points
    global straight_qrcode
    global rect
    global count
    global get_roi
    global get_layer
    global center_list
    global unreachable
    global __isRunning
    global start_pick_up
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global start_count_t1, t1
    global detect_color, draw_color, color_list
    global detect_layer
    global detect_direct
    print('3')
    #图像初操作
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    #画线操作
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

    if not __isRunning:
        return img#没跑了
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)#高斯滤波
    #如果检测到某个区域有识别到的物体，则一直检测该区域直到没有为止
    if get_roi and not start_pick_up and get_layer:
        get_roi = False
        get_layer = False
        frame_gb = getMaskROI(frame_gb, roi, size)#目标区域外完全变黑      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0
    
    if not start_pick_up:#一个比较关键的参数？
        for i in color_range:#有一个神奇的参数
            if i in __target_color:
                frame_mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])  #对原图像和掩模进行位运算
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6,6),np.uint8))  #开运算
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6,6),np.uint8)) #闭运算
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓
                areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓
                if areaMaxContour is not None:
                    if area_max > max_area:#找最大面积
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 2500:  # 有找到最大面积
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.int0(cv2.boxPoints(rect))
            
            roi = getROI(box) #获取roi区域
            get_roi = True
            #QRcode
            src = getMaskROI(frame_resize, roi, size)
            codeinfo, points, straight_qrcode = qrcoder.detectAndDecode(src)
            placelist = codeinfo.split(" ")
            if len(placelist) == 2:
                placelist = codeinfo.split(" ")
                print("qrcode information is : \n%s"% codeinfo)
                detect_layer = placelist[0]
                detect_direct = placelist[1]
                get_layer = True
            img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # 获取木块中心坐标
             
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size) #转换为现实世界坐标
            
            cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[color_area_max], 1) #绘制中心点
            
            distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) #对比上次坐标来判断是否移动
            last_x, last_y = world_x, world_y
            if not start_pick_up:
                if color_area_max == 'red':  #红色最大
                    color = 1
                elif color_area_max == 'green':  #绿色最大
                    color = 2
                elif color_area_max == 'blue':  #蓝色最大
                    color = 3
                else:
                    color = 0
                color_list.append(color)
                # 累计判断
                if distance < 0.5:
                    count += 1
                    center_list.extend((world_x, world_y))
                    if start_count_t1:
                        start_count_t1 = False
                        t1 = time.time()
                    if time.time() - t1 > 1:
                        rotation_angle = rect[2] 
                        start_count_t1 = True
                        world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                        center_list = []
                        count = 0
                        start_pick_up = True
                else:
                    t1 = time.time()
                    start_count_t1 = True
                    center_list = []
                    count = 0
#设定draw_color
                if len(color_list) == 3:  #多次判断
                    # 取平均值
                    color = int(round(np.mean(np.array(color_list))))
                    color_list = []
                    if color == 1:
                        detect_color = 'red'
                        draw_color = range_rgb["red"]
                    elif color == 2:
                        detect_color = 'green'
                        draw_color = range_rgb["green"]
                    elif color == 3:
                        detect_color = 'blue'
                        draw_color = range_rgb["blue"]
                    else:
                        detect_color = 'None'
                        draw_color = range_rgb["black"]
        else:
            if not start_pick_up:
                draw_color = (0, 0, 0)
                detect_color = "None"

    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    return img

if __name__ == '__main__':
    init()#初始化
    start()#开始
    __target_color = ('red', 'green', 'blue')#元组
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
    stop()
    exit()
