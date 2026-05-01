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


x= 0; y= 0; z= 0
rot = 0; rvec = 0
headingTarg = 0
rollTarg = 0
tol = 2
tf = 100
timeOut = 60

# lastPositions = deque()
# lastPositions.maxlen = 1000

# knownTag = (0 , 0 , 0)


xPID = PID.PID(kp=1, ki=0, kd= 0, target=xtarg, min=-30, max=30, name='xPid')
yPID = PID.PID(kp=1, ki=0, kd= 0, target=yTarg, min=-30, max=30, name='yPid')
zPID = PID.PID(kp=75, ki=0, kd=50, target=zTarg, min=-150, max=150, name='zPid')
zPID.updateTarget(depthTarget)
# yawPID = PID.PID(kp=1.2, ki= 0, kd= 1.2, target=yawTarget, )

lastKnownTagInfo = [0 ,0 ,0]
STATUS = 'INIT'
cam.startStream()
t0 = time.monotonic()
while(time.monotonic()-t0 <= timeOut):
    cam.stream.show()
    pos = cam.getPos()
    depth = rov.grabDepth()
    print("Depth:", depth, "Status", STATUS)
    match(STATUS):
        case 'INIT':
            rov.armRobot()
            rov.setGain(0.2)
            rov.setMode('MANUAL')
            pos = cam.getPos()
            STATUS = 'SEARCH'
        case 'SEARCH':
            depthTarget = 0.5
            if(depth is not None):
                # print(depth)
                vertOut = zPID.update(depth)
                # print(vertOut)
            rov.goVertical(vertOut)
            # rov.turn(35)
            if(pos is not None):
                print('TAG FOUND')
                depthTarget = 1.5

                x, y, z, rot, rvec = pos
                lastKnownTagInfo = pos[0:3]
                tagTime = time.monotonic()
                STATUS = 'APPROACH'
            else:
                STATUS = 'SEARCH'

        case 'APPROACH':
            if(pos is not None):
                x, y, z, rot, rvec = pos #this will be in the same units as the marker size in camera class
                print(x, y, z)
                lastKnownTagInfo = pos[0:2]
                tagTime = time.monotonic()
            elif(tagTime - time.monotonic() >= 0.5):
                strafeOut = 0
                fwdOut = 0
                STATUS = 'SEARCH'
            # print(depth)
            fwdOut = xPID.update(lastKnownTagInfo[0])
            strafeOut = yPID.update(lastKnownTagInfo[1])
            vertOut = zPID.update(depth)
            print("strafe:", strafeOut)
            print("fwd:", fwdOut)

            rov.goVertical(vertOut)
            # rov.strafe(strafeOut)
            rov.goForward(fwdOut)

            # angle = np.rad2deg(np.atan2(z, x)) 
            # headingOut = yawPID.update(angle)

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
