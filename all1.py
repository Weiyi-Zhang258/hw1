"""This controller uses built-in motion manager modules to get the OP2 to walk.."""

import os
import sys
import cv2
import numpy as np
import time
import math
import threading
from controller import Robot
from controller import Camera


# Path operations to correctly locate the managers.
libraryPath = os.path.join(os.environ.get("WEBOTS_HOME"), 'projects', 'robots', 'robotis', 'darwin-op', 'libraries',
                           'python37')
libraryPath = libraryPath.replace('/', os.sep)
sys.path.append(libraryPath)

from managers import RobotisOp2GaitManager, RobotisOp2MotionManager

# Names of position sensors needed to get the corresponding device and read the measurements.
positionSensorNames = ('ShoulderR', 'ShoulderL', 'ArmUpperR', 'ArmUpperL',
                       'ArmLowerR', 'ArmLowerL', 'PelvYR', 'PelvYL',
                       'PelvR', 'PelvL', 'LegUpperR', 'LegUpperL',
                       'LegLowerR', 'LegLowerL', 'AnkleR', 'AnkleL',
                       'FootR', 'FootL', 'Neck', 'Head')
# List of position sensor devices.
positionSensors = []

# Create the Robot instance.
robot = Robot()
robot.getSupervisor()


basicTimeStep = int(robot.getBasicTimeStep())
# print(robot.getDevice("camera"))
camera1=robot.getCamera("Camera")
print(camera1)
# camera= Camera(camera1)
camera= Camera('Camera')
# print(robot.getCamera('Camera'))
# camera.wb_camera_enable()
mTimeStep=basicTimeStep
camera.enable(int(mTimeStep))
camera.getSamplingPeriod()
# width=camera.getWidth()
# height=camera.getHeight()
firstimage=camera.getImage()
ori_width = int(4 * 160)  # 原始图像640x480
ori_height = int(3 * 160)
r_width = int(4 * 20)  # 处理图像时缩小为80x60,加快处理速度，谨慎修改！
r_height = int(3 * 20)
color_range = {'yellow_door': [(10, 43, 46), (34, 255, 255)],
               'red_floor1': [(0, 43, 46), (10, 255, 255)],
               'red_floor2': [(156, 43, 46), (180, 255, 255)],
               'green_bridge': [(35, 43, 20), (100, 255, 255)],
               'yellow_hole': [(10, 70, 46), (34, 255, 255)],
               'black_hole': [(0, 0, 0), (180, 255, 80)],
               'black_gap': [(0, 0, 0), (180, 255, 100)],
               'black_dir': [(0, 0, 0), (180, 255, 46)],
               'blue': [(110, 43, 46), (124, 255, 255)],
               'black_door': [(0, 0, 0), (180, 255, 46)],
               }

def get_img():
    global org_img
    global ret
    global Running
    global camera
    while True:
        if camera.enable(int(mTimeStep))!=0:
            org_img = camera.getImage()
            if org_img==None:
                ret=False
            else:
                ret=True
        else:
            time.sleep(0.01)

# 读取图像线程
th1 = threading.Thread(target=get_img)
th1.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
th1.start()

# print(image)
if firstimage==None:
    print("none")

motionManager = RobotisOp2MotionManager(robot)

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Retrieve devices.
headLed = robot.getLED('HeadLed')
eyeLed = robot.getLED('EyeLed')
gyro = robot.getGyro('Gyro')

# Enable all the position sensors and populate the 'positionSensor' list.
for i in range(0, len(positionSensorNames)):
    positionSensors.append(robot.getPositionSensor(positionSensorNames[i] + 'S'))
    positionSensors[i].enable(basicTimeStep)

# Initialize the LED devices.
headLed.set(0xff0000)
eyeLed.set(0xa0a0ff)
# Enable gyro device.
gyro.enable(basicTimeStep)

# Perform one simulation step to get sensors working properly.
robot.step(timestep)

# Page 1: stand up.
# Page 9: assume walking position.

motionManager.playPage(1, True)
motionManager.playPage(9, True)

# Initialize OP2 gait manager.
gaitManager = None
gaitManager = RobotisOp2GaitManager(robot, "")
gaitManager.start()
gaitManager.setXAmplitude(0.0)
gaitManager.setYAmplitude(0.0)
gaitManager.setBalanceEnable(True)
gaitAmplitude = -1
looptimes = 0

motorNames = ['PelvYR', 'PelvYL', 'PelvR', 'PelvL', 'LegUpperR', 'LegUpperL', 'LegLowerR', 'LegLowerL',
              'AnkleR', 'AnkleL', 'FootR', 'FootL']
motors = []
positionSensors = []
old_motor_position = None
for i in range(0, len(motorNames)):
    positionSensors.append(robot.getPositionSensor(motorNames[i] + 'S'))
    positionSensors[i].enable(timestep)

for j in range(0, len(motorNames)):
    motors.append(robot.getMotor(motorNames[j]))

# Main loop: perform a simulation step until the simulation is over.
gaitManager.setXAmplitude(1)
flag=True
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    #area_max_contour = np.array([[[0, 0]], [[0, 1]], [[1, 1]], [[1, 0]]])
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 25:  #只有在面积大于25时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓

#################################################第二关：台阶##########################################
def floor(r_w, r_h):
    #原始图像，关卡数，关卡名-未确定关卡时为空，移动参数，重置信号，跳关信号，debug信号
    # global org_img, state, state_sel, step, reset, skip, debug
    global org_img, step
    step=0
    while flag:
        #返回开机时间
        t1 = cv2.getTickCount()
        # 扩展hei边，防止边界无法识别
        #填充边界为固定值
        border = cv2.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(0, 0, 0))  # 扩展hei边，防止边界无法识别
        #为颜色字典掩码与原图像进行初始或者运算
        org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        # 对原图像和掩模(颜色的字典)进行位运算
        frame_red1 = cv2.inRange(frame_hsv, color_range['red_floor1'][0], color_range['red_floor1'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        # 对原图像和掩模(颜色的字典)进行位运算
        frame_red2 = cv2.inRange(frame_hsv, color_range['red_floor2'][0],
                                 color_range['red_floor2'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        frame_red = cv2.add(frame_red1, frame_red2)
        # 开运算 去噪点
        opened = cv2.morphologyEx(frame_red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        # 闭运算 封闭连接
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        # print(closed)

        # 找出轮廓
        (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        # 找出最大轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        # 最大轮廓百分比
        percent = round(area_max * 100 / (r_w * r_h), 2)  # 最大轮廓百分比
        # 最大轮廓的输出及标注
        # if debug:
        #     cv2.imshow('closed', closed)  # 显示图像
        #     cv2.drawContours(org_img_copy, contours, -1, (255, 0, 255), 1)  # 画出轮廓
        #     cv2.putText(org_img_copy, 'area:' + str(percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
        
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            # center, w_h, angle = rect  # 中心点 宽高 旋转角度
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标
            top_right = areaMaxContour[0][0]  # 右上角
            top_left = areaMaxContour[0][0]  # 左上角
            for c in areaMaxContour:
                if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 1.5 * c[0][1] < (r_w - top_right[0]) + 1.5 * top_right[1]:
                    top_right = c[0]
            angle = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi  # 得到角度
            center_x = (top_right[0] + top_left[0]) / 2  # 得到中心坐标
            center_y = (top_right[1] + top_left[1]) / 2
        else:
            angle = 0
            center_x = 0.5 * r_w
            center_y = 0

        if step == 0:
            SSR.running_action_group('239', 1)  # 到达台阶前

            if percent > 70:  # 接近台阶
                #print('percent > 70')
                SSR.running_action_group('157', 1)  # 93立正
                SSR.running_action_group('232', 2)  # 小步贴近边沿
                SSR.running_action_group('237', 1)  # 更小步贴近边沿
                SSR.running_action_group('226', 1)  # 上台阶
                PWMServo.setServo(1, 1650, 500)  # 抬头
                PWMServo.setServo(2, 1510, 500)  # NO3
                step = 1
                SSR.running_action_group('00', 1)  # 短立正
                time.sleep(0.2)  # 等待摄像头稳定
        elif step == 1:  # 调整方向和位置
            # print('angle: %f' %(angle))
            # print('center_x: %f' %(center_x))
            # print('center_y: %f' %(center_y))
            if angle > 5:  # 需要左转
                SSR.running_action_group('202', 1)
            elif angle < -1:  # 需要右转
                SSR.running_action_group('203', 1)
            elif -1 <= angle <= 5:  # 角度正确
                if center_x > 0.52 * r_w:  # 需要右移
                    SSR.running_action_group('161', 1)
                elif center_x < 0.48 * r_w:  # 需要左移
                    SSR.running_action_group('160', 1)
                elif 0.48 * r_w <= center_x <= 0.52 * r_w:  # 位置正确
                    SSR.running_action_group('239', 10)  # 直行十步
                    SSR.running_action_group('157', 1)
                    PWMServo.setServo(1, 2200, 500)  # 低头，准备看边沿
                    PWMServo.setServo(2, 1510, 500)
                    step = 2
                    time.sleep(0.6)
        elif step == 2:  # 接近下台阶边沿，调整方向
            # print('angle: %f' %(angle))
            # print('center_x: %f' %(center_x))
            # print('center_y: %f' %(center_y))
            if center_y < 0.5 * r_h: #or percent < 10:  # 台阶边沿还远 or  percent<10(由于全红时的识别bug)
                SSR.running_action_group('232', 1)
            elif 0.5 * r_h <= center_y:  # 已经很接近边沿，调整一次方向
                if angle > 7:  # 需要左转
                    SSR.running_action_group('202', 1)
                elif angle < -3:  # 需要右转
                    SSR.running_action_group('203', 1)
                elif -3 <= angle <= 7:
                    step = 3
        elif step == 3:
            if center_y < 0.73 * r_h:
                SSR.running_action_group('237', 1)  # 更小步挪
            else:  # 到达下台阶边沿
                SSR.running_action_group('225', 1)  # 下台阶
                # state += 1
                # state_sel = None
                step = -1  # 雷阵调方向
                cv2.destroyAllWindows()
                flag=False
                # print('state=3')
                break


while robot.step(timestep) != -1:
    Value = []
    for i in range(0, len(motorNames)):
        Value.append((positionSensors[i].getValue()))
    # print(Value)
    # print('before')
    gaitManager.step(basicTimeStep)
    # print('after')
    looptimes += 1
    if looptimes >= 3:
        break

