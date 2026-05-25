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

print("CV2 version:", cv2.__version__)
 

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
timeOut = 60

cameraToHookOffset= 65 #mm


# lastPositions = deque()
# lastPositions.maxlen = 1000

# knownTag = (0 , 0 , 0)



xPID = PID.PID(kp=1, ki=0, kd= 0.01, target=xtarg, min=-60, max=60, name='xPid', tol=10)
yPID = PID.PID(kp=1, ki=0, kd= 0.01, target=yTarg, min=-60, max=60, name='yPid', tol=10)
zPID = PID.PID(kp=50, ki=1, kd=10, target=zTarg, min=-150, max=150, name='zPid', tol=10)
zPID.updateTarget(depthTarget)
turnPID = PID.PID(kp=0.1, ki= 0, kd= 0.1, target=turnTarg, min=-30, max = 30, name='turn', tol=2)

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

                lastKnownTagInfo = pos[0:3]
                tagTime = time.monotonic()
                STATUS = 'APPROACH'
            else:
                STATUS = 'SEARCH'

        case 'APPROACH':
            if(pos is not None):
                x, y, z, rot, rvec = pos #this will be in the same units as the marker size in camera class
                angle = np.rad2deg(np.atan2(y,x))
                print("angle:", angle)
                print(x, y, z)
                lastKnownTagInfo = pos[0:2]
                tagTime = time.monotonic()
            elif(tagTime - time.monotonic() >= 1.5):
                strafeOut = 0
                fwdOut = 0
                STATUS = 'SEARCH'
            fwdOut = xPID.update(lastKnownTagInfo[0])
            strafeOut = yPID.update(lastKnownTagInfo[1])
            # turnOut = turnPID.update(np.rad2deg(np.atan2(y, x)))
            vertOut = zPID.update(depth)

            # print("fwd:", fwdOut)
            # print("turnOut", turnOut)

            rov.goVertical(vertOut)
            rov.strafe(strafeOut)

            # if(angle > 2 or angle < -2):
            #     print('Turning....')
            #     rov.turn(turnOut)
            #     rov.goForward(0)
            if(fwdOut > 0):
                rov.goForwardFront(fwdOut)
            else:
                rov.goForwardBack(fwdOut)
            
            STATUS = STATUS
        case 'ALIGN':
            if(pos is not None):
                x, y, z, rot, rvec = pos
                angle = np.atan2(y,x)
                print("angle", np.rad2deg(angle))

            STATUS = 'ALIGN'
        case 'ATTACH':
            STATUS = 'ATTACH'
        case 'DONE':
            print('done')
            rov.disarmRobot()
    rov.updateThrusts()
# print("DisarmingRobot")

cam.release()
rov.lightsOff()
rov.disarmRobot()

