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
yTarg = 200
zTarg = 0
depthTarget = 0.5 #meter
tagTime = 0

vertOut = 0

yawTarget = 0

fwdPWM = 0


x= 0; y= 0; # tolerance 50mm
rot = 0; rvec = 0
headingTarg = 0
turnTarg = 90
tol = 2
tf = 100
timeOut = 100

cameraToHookOffset= 65 #mm





xPID = PID.PID(kp=0.1, ki=0, kd= 0.001, target=xtarg, min=-30, max=30, name='xPid', tol=100)
yPID = PID.PID(kp=0.1, ki=0, kd= 0.001, target=yTarg, min=-30, max=30, name='yPid', tol=100)
zPID = PID.PID(kp=100, ki=0.15, kd=0.01, target=zTarg, min=-200, max=60, name='zPid', tol=0.1)
zPID.updateTarget(depthTarget)
turnPID = PID.PID(kp=5, ki= 0, kd= 0, target=turnTarg, min=-30, max=30, name='turn', tol=30)


def printPIDS():
    xPID.print_PID()
    yPID.print_PID()
    zPID.print_PID()
    turnPID.print_PID()
    

lastKnownTagInfo = [0 ,0 ,0]
STATUS = 'INIT'
cam.startStream()
t0 = time.monotonic()
while(time.monotonic()-t0 <= timeOut):
    cam.stream.show()
    pos = cam.getPos()
    depth = rov.grabDepth()
    print(depth, "Status:", STATUS)
    # print(pos)
    printPIDS()
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
            zPID.updateTarget(1.5)
            if(pos is not None):
                x, y, z, rot, rvec = pos #this will be in the same units as the marker size in camera class
                tagTracking.captureTag(pos, time.monotonic())
                angle = np.rad2deg(np.atan2(y,x))

            elif(len(tagTracking.tags) > 2):
                pTag = tagTracking.predictTag(time.monotonic())
                x = pTag[0]
                y = pTag[1]

            fwdOut = xPID.update(x)
            strafeOut = yPID.update(y)
            vertOut = zPID.update(depth)
            # rov.turn(turnOut)

            rov.goVertical(vertOut)
            rov.goForward(fwdOut)
            rov.strafe(strafeOut)

            if(xPID.atTarget(x)):
                STATUS = 'ALIGN'

            
            STATUS = STATUS
        case 'ALIGN':
            if(pos is not None):
                x, y, z, rot, rvec = pos #this will be in the same units as the marker size in camera class
                angle = min(abs(np.rad2deg(np.atan2(y, x))), abs(360 - np.rad2deg(np.atan2(y,x))))

            elif(len(tagTracking.tags) > 2):
                pTag = tagTracking.predictTag(time.monotonic())
                x = pTag[0]
                y= pTag[1]
                angle = min(abs(np.rad2deg(np.atan2(y, x))), abs(180 - np.rad2deg(np.atan2(y,x))))
            

            vertOut = zPID.update(depth)
            fwdOut = xPID.update(x)
            strafeOut= yPID.update(y)
            turnOut = turnPID.update(angle)

            rov.goVertical(vertOut)

            rov.goForward(fwdOut)
            # rov.turn(turnOut)

            if(yPID.atTarget(y) and xPID.atTarget(x)):
                STATUS = 'ATTACH'
            STATUS = STATUS
        case 'ATTACH':
            if(pos is not None):
                x, y, z, rot, rvec = pos #this will be in the same units as the marker size in camera class
                tagTracking.captureTag(pos, time.monotonic())
                angle = np.rad2deg(np.atan2(y,x))
            else:
                pTag = tagTracking.predictTag(time.monotonic())
                x = pTag[0]
                y= pTag[1]
            vertOut = zPID.update(depth)
            fwdOut = xPID.update(x)
            strafeOut= yPID.update(y)

            zPID.updateTarget(3)
            rov.goVertical(vertOut)
            rov.goForward(fwdOut)
            rov.strafe(strafeOut)

            if(zPID.atTarget(depth)):
                STATUS = 'DONE'
                rov.stopThruster()
            
            STATUS = STATUS
        case 'DONE':
            print('done')
            rov.disarmRobot()
        
        case 'TEST':
            print('test case')
            vertOut = zPID.update(depth)
            if(-25 <= vertOut < 25):
                vertOut = -25
            rov.goVertical(vertOut)
            if(time.monotonic()-t0 > 30):
                zPID.updateTarget(2)


    rov.updateThrusts()
commands.plot_measurements(turnPID)
commands.plot_measurements(xPID)

# print("DisarmingRobot")
cam.release()
rov.lightsOff()
rov.disarmRobot()



