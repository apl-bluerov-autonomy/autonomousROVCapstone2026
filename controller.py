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
zTarg = 1500

yawTarget = 90

fwdPWM = 0


x= 0; y= 0; z= 50
rot = 0; rvec = 0
headingTarg = 90
rollTarg = 0
tol = 2
tf = 100
t0 = time.monotonic()
# rov.disarmRobot()

# lastPositions = deque()
# lastPositions.maxlen = 1000

# knownTag = (0 , 0 , 0)


xPID = PID.PID(kp=-3e-1, ki=0, kd= -1e-5, target=xtarg, pwm_min=1400, pwm_max=1600)
yPID = PID.PID(kp=5e-1, ki=0, kd= 0, target=yTarg, pwm_min= 1400, pwm_max=1600)
zPID = PID.PID(kp=-1e-1, ki= 0, kd= 0, target=zTarg, pwm_min=1400, pwm_max=1600)
yawPID = PID.PID(kp=1.2, ki= 0, kd= 1.2, target=yawTarget, pwm_min=1400, pwm_max=1600)

# u = np.empty(shape=1)
# ref = np.empty(shape=1)
# times = np.empty(shape=1)
# pos = np.empty(shape=1)

# def updateArrs(r, p, utx ,t):
#     ref.append(r)
#     pos.append(p)
#     u.append(utx)
#     times.append(t)

STATUS = 'INIT'

while(1):
    time.sleep(1)
    match(STATUS):
        case 'INIT':
            cam.startStream()
            rov.armRobot()
            rov.setGain(0.2)
            rov.setMode('MANUAL')
            rov.lightsOff()
            pos = cam.getPos()
            if(pos is not None):
                STATUS = 'APPROACH'
            STATUS = 'SEARCH'
        case 'SEARCH':
            pos = cam.getPos()
            if(pos is not None):
                x, y, z, rot, rvec = pos
                # lastPositions.appendleft(pos[0:2])
                # knownTag = x, y ,z
            STATUS = 'APPROACH'
        case 'APPROACH':

            pos = cam.getPos()
            if(pos is not None):
                x, y, z, rot, rvec = pos #this will be in the same units as the marker size in camera class
                knownTag = x, y ,z
                # lastPositions.appendleft(pos[0:2])
            compass = rov.grabCompass()
            # print("compass:", compass)
            print("rot", rot)
            print("rvec", np.linalg.norm(rvec))
            print("rvec", rvec)
            
            # angle = np.rad2deg(np.atan2(z, x)) 
            # headingOut = yawPID.update(angle)
            # vertOut = yPID.update(y)
            # fwdOut = zPID.update(z)
            # strafeOut = xPID.update(x)
            # fwdOut = zPID.update(z)
            # updateArrs(headingTarg, angle, b, time.monotonic()-t0)
            # rov.turn(headingOut); rov.goVertical(vertOut); rov.goForward(fwdOut)
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





# plt.plot(times, ref)
# plt.plot(u)
# plt.plot(pos)
# plt.show()

