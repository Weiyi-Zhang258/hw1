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
# from managers import RobotisOp2GaitManager, RobotisOp2MotionManager


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
print(camera.enable(int(mTimeStep)))
print(camera.getSamplingPeriod())
print(camera.getWidth())
print(camera.getHeight())
image=camera.getImage()
# print(image)
if image==None:
    print("none")
# print(image.size())
# cameradata = cv2.VideoCapture('Camera')
camera.saveImage('/home/luyi/webots.png',100)
# print(len(cap))
# cv2.imshow("cap",cap)
# print(image[2][3][0])
# for x in range(0,camera.getWidth()):
    # for y in range(0,camera.getHeight()):
        # print(camera.getSamplingPeriod())
        # red   = image[x][y][0]
        # green = image[x][y][1]
        # blue  = image[x][y][2]
        # gray  = (red + green + blue) / 3
        # print ('r='+str(red)+' g='+str(green)+' b='+str(blue))
# Initialize motion manager.
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
gaitManager.setXAmplitude(0.1)
nip=50
while robot.step(timestep) != -1:
    # print(camera.getSamplingPeriod())
    # print(camera.getWidth())
    # print(camera.getHeight())
    looptimes += 1
    if looptimes<=nip:
        Value = []
        for i in range(0, len(motorNames)):
            Value.append((positionSensors[i].getValue()))
    # print(Value)
    # print('before')
        gaitManager.step(basicTimeStep)
    if looptimes==nip:
        gaitManager.setXAmplitude(0)
        gaitManager.setAAmplitude(1)
    if looptimes<=2*nip:
        Value = []
        for i in range(0, len(motorNames)):
            Value.append((positionSensors[i].getValue()))
    # print(Value)
    # print('before')
        gaitManager.step(basicTimeStep)
    if looptimes ==2*nip:
        gaitManager.setAAmplitude(0)
        gaitManager.setXAmplitude(0.1)
    if looptimes<=3*nip:
        Value = []
        for i in range(0, len(motorNames)):
            Value.append((positionSensors[i].getValue()))
        gaitManager.step(basicTimeStep)
    if looptimes ==3*nip:
        gaitManager.setXAmplitude(0)
        gaitManager.setAAmplitude(-1)
        
    if looptimes<=4*nip:
        Value = []
        for i in range(0, len(motorNames)):
            Value.append((positionSensors[i].getValue()))
        gaitManager.step(basicTimeStep)
        
    if looptimes >=5*nip:
        break
        


# while robot.step(timestep) != -1:
    # print("wwww")
    # motionManager.playPage(1, True)
    # motionManager.playPage(2, True)
    # motionManager.playPage(3, True)
    # motionManager.playPage(4, True)
    # motionManager.playPage(6, True)
    # motionManager.playPage(9, True)
    # motionManager.playPage(12, True)
    # motionManager.playPage(13, True)
    # motionManager.playPage(15, True)
    # motionManager.playPage(16, True)
    # motionManager.playPage(17, True)
    # motionManager.playPage(23, True)
    # motionManager.playPage(24, True)
    # motionManager.playPage(27, True)
    # motionManager.playPage(29, True)
    # motionManager.playPage(31, True)
    # motionManager.playPage(38, True)
    # motionManager.playPage(41, True)
    # motionManager.playPage(54, True)
    # motionManager.playPage(57, True)
    # motionManager.playPage(70, True)
    # motionManager.playPage(71, True)
    # motionManager.playPage(90, True)
    # motionManager.playPage(91, True)
