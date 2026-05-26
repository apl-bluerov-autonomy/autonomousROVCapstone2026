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
import tagTracking

print("CV2 version:", cv2.__version__)
 

RUN_MODE = {
    'IDLE',
    'VERTICAL',
    'FORWARD',
    'STRAFE',
}

mtx = 0
dist = 0


STATUS = {'INIT', 'SEARCH', 'APPROACH', 'ALIGN', 'ATTACH', 'DONE', 'TEST'}

# message_types = {'ATTITUDE', 'SCALED_IMU2', 'NAMED_VALUE_FLOAT', 'VFR_HUD'}


## Initialize robot , then do a small descend
rov = commands.Robot()
cam = camera.camera()
cam.loadCameraSettings()
rov.calibrateDepth()

tags = []


#PWM limits 1100-1900
xtarg = 0
yTarg = 0
zTarg = 0
depthTarget = 0.5 #meter
tagTime = 0

vertOut = 0

yawTarget = 0

fwdPWM = 0


x= 0; y= 0; # tolerance 50mm
rot = 0; rvec = 0
headingTarg = 0
turnTarg = 0
tol = 2
tf = 100
timeOut = 120

cameraToHookOffset= 65 #mm





xPID = PID.PID(kp=0.1, ki=0, kd= 0.01, target=xtarg, min=-30, max=30, name='xPid', tol=50)
yPID = PID.PID(kp=0.1, ki=0, kd= 0.01, target=yTarg, min=-30, max=30, name='yPid', tol=50)
zPID = PID.PID(kp=40, ki=1, kd=10, target=zTarg, min=-100, max=100, name='zPid', tol=50)
zPID.updateTarget(depthTarget)
turnPID = PID.PID(kp=0.01, ki= 0, kd= 0.1, target=turnTarg, min=-30, max = 30, name='turn', tol=10)

lastKnownTagInfo = [0 ,0 ,0]
STATUS = 'INIT'
cam.startStream()
t0 = time.monotonic()
while(time.monotonic()-t0 <= timeOut):
    cam.stream.show()
    pos = cam.getPos()
    depth = rov.grabDepth()
    print(depth, "Status:", STATUS, vertOut)
    rov.disableThrust()
    match(STATUS):
        case 'INIT':
            
            rov.armRobot()
            rov.lightsOn()
            rov.setGain(0.2)
            rov.setMode('MANUAL')
            pos = cam.getPos()
            STATUS = 'SEARCH'
        case 'SEARCH':
            depthTarget = 0.5
            if(depth is not None):
                vertOut = zPID.update(depth)
            rov.goVertical(vertOut)
            if(pos is not None):
                print('TAG FOUND')
                x, y, z, rot, rvec = pos
                angle = np.rad2deg(np.atan2(y,x))
                tagTracking.captureTag(pos, time.monotonic())
                fwdOut = xPID.update(x)
                strafeOut = yPID.update(y)

                STATUS = 'APPROACH'
            else:
                STATUS = 'SEARCH'

        case 'APPROACH':
            depthTarget = 1
            if(pos is not None):
                x, y, z, rot, rvec = pos #this will be in the same units as the marker size in camera class
                tagTracking.captureTag(pos, time.monotonic())
                angle = np.rad2deg(np.atan2(y,x))
                # print("angle:", angle)
                print(x, y, z)
                fwdOut = xPID.update(x)
                strafeOut = yPID.update(y)
            # elif(len(tagTracking.tags) > 2):
            #     pTag = tagTracking.predictTag(time.monotonic())
            #     fwdOut = xPID.update(pTag[0])
            #     strafeOut = yPID.update(pTag[1])

            vertOut = zPID.update(depth)
            # rov.turn(turnOut)

            rov.goVertical(vertOut)
            rov.goForward(fwdOut)
            rov.strafe(strafeOut)

            if(abs(x) <= 200 and abs(y) <= 200):
                STATUS = 'APPROACH'
            STATUS = STATUS
        case 'ALIGN':
            if(pos is not None):
                x, y, z, rot, rvec = pos #this will be in the same units as the marker size in camera class
                angle = np.rad2deg(np.atan2(y,x))
                print("angle:", angle)
                print(x, y, z)

            vertOut = zPID.update(depth)
            fwdOut = xPID.update(x/10)
            strafeOut= yPID.update(y/10)
            turnOut = turnPID.update(np.rad2deg(np.atan2(y, x)))

            rov.goVertical(vertOut)
            rov.turn(turnOut)
            rov.goForwardBack(fwdOut)
            rov.strafe(strafeOut)

            STATUS = 'ALIGN'
        case 'ATTACH':
            vertOut = zPID.update(depth)
            rov.goVertical(vertOut)
            depthTarget = 3
            STATUS = 'ATTACH'
        case 'DONE':
            print('done')
            rov.disarmRobot()
        
        case 'TEST':
            print('test case')
            zPID.update(depth)
            rov.goVertical(vertOut)
            rov.goForward(50)
    rov.updateThrusts()
# print("DisarmingRobot")

cam.release()
rov.lightsOff()
rov.disarmRobot()

#test