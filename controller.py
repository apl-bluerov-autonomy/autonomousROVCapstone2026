import math
from pymavlink import mavutil
import time
import commands
import PID
import random
import numpy as np
import pylab as plt
import camera
import multiprocessing as mp
import cv2
from collections import deque
 

RUN_MODE = {
    'IDLE',
    'VERTICAL',
    'FORWARD',
    'STRAFE',
}

mtx = 0
dist = 0


STATUS = {'INIT', 'SEARCH', 'APPROACH', 'ALIGN', 'ATTACH', 'DONE'}

# message_types = {'ATTITUDE', 'SCALED_IMU2', 'NAMED_VALUE_FLOAT', 'VFR_HUD'}


## Initialize robot , then do a small descend
rov = commands.Robot()
cam = camera.camera()
cam.loadCameraSettings()
rov.calibrateDepth()

# # while(1):
#     rov.grabDepth()
#     time.sleep(0.5)


#PWM limits 1100-1900
xtarg = 0
yTarg = 0
zTarg = 0
vertOut = 0

yawTarget = 0

fwdPWM = 0


x= 0; y= 0; z= 0
rot = 0; rvec = 0
headingTarg = 0
rollTarg = 0
tol = 2
tf = 100
t0 = time.monotonic()

# lastPositions = deque()
# lastPositions.maxlen = 1000

# knownTag = (0 , 0 , 0)


# xPID = PID.PID(kp=-3e-1, ki=0, kd= -1e-5, target=xtarg, pwm_min=1400, pwm_max=1600)
# yPID = PID.PID(kp=5e-1, ki=0, kd= 0, target=yTarg, pwm_min= 1400, pwm_max=1600)
zPID = PID.PID(kp=5, ki= 0, kd= 0, target=zTarg, name='zPid')
# yawPID = PID.PID(kp=1.2, ki= 0, kd= 1.2, target=yawTarget, pwm_min=1400, pwm_max=1600)


STATUS = 'INIT'
cam.startStream()

while(1):
    cam.stream.show()
    pos = cam.getPos()
    time.sleep(1e-4)
    match(STATUS):
        case 'INIT':
            rov.armRobot()
            rov.setGain(0.2)
            rov.setMode('MANUAL')
            pos = cam.getPos()
            STATUS = 'SEARCH'
        case 'SEARCH':
            if(pos is not None):
                x, y, z, rot, rvec = pos
                STATUS = 'APPROACH'
            
        case 'APPROACH':
            if(pos is not None):
                tagTime = time.monotonic()
                x, y, z, rot, rvec = pos #this will be in the same units as the marker size in camera class

                # strafeOut = xPID.update(x)
                # vertOut = yPID.update(y)
                print("z val:", z)
                vertOut = zPID.update(z)

                # angle = np.rad2deg(np.atan2(z, x)) 
                # headingOut = yawPID.update(angle)
                rov.goVertical(vertOut)
            else:
                if(time.monotonic()-tagTime >= 2):
                    rov.goVertical(0)
           

            STATUS = STATUS
        case 'ALIGN':
            STATUS = 'ATTACH'
        case 'ATTACH':
            STATUS = 'DONE'
        case 'DONE':
            print('done')
            rov.disarmRobot()
# print("DisarmingRobot")
rov.disarmRobot()
cam.release()
