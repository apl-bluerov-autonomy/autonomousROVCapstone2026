import math
from pymavlink import mavutil
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt

PULSE_DEADBAND = 40 #based on t200 spec sheet
PULSE_PERIOD = 0.25 #need to experiment/tweak probably
def initConnection():
    connection = "udp:0.0.0.0:14550"    #Create connection from topside
    connection = mavutil.mavlink_connection(connection)

    #verify connection
    connection.wait_heartbeat()
    return connection

class Robot:
    xVel = 0
    yVel = 0
    zVel = 0

    thrust1, thrust2, thrust3, thrust4 = 1500, 1500, 1500, 1500


    def __init__(self):
        self.robot = initConnection()
        pass


#BASIC/NECESSARY COMMANDS

    def armRobot(self):
        print("Arming robot")
        self.robot.arducopter_arm()
        self.robot.motors_armed_wait()
        print("Robot armed")
    

    #STABILIZE, MANUAL, DEPTH HOLD

    def setMode(self, mode):
        if mode not in self.robot.mode_mapping():
            print('Error, unknown mode')
            self.robot.disarmRobot()
        else:
            mode_id = self.robot.mode_mapping()[mode]
            self.robot.set_mode(mode_id)

    def disarmRobot(self):
        print("Disarming Robot")
        self.robot.arducopter_disarm()

    def setGain(self, gain=0.3):
        param = b'PILOT_GAIN'
        self.robot.mav.param_set_send(
            self.robot.target_system,
            self.robot.target_component,
            param,
            gain,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def calibrateDepth(self):
        self.robot.mav.command_long_send(
        self.robot.target_system,
        self.robot.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 0, 1, 0, 0, 0, 0, 0
    )


    def grabDepth(self):
        self.robot.mav.command_long_send(
        self.robot.target_system,
        self.robot.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        137,
        0, 0, 0, 0, 0, 0
    )
        msg = self.robot.recv_match(type='SCALED_PRESSURE2', blocking=True)
        if msg:
            pressure = msg.press_abs
            # print(pressure)
            depth = (pressure - 1013.25) / 98.0665
            # print(f"Depth: {depth:.3f} m")
            return depth
        else:
            print("No response received")
            return None
    
    def grabIMU(self):
        self.robot.mav.command_long_send(
        self.robot.target_system,
        self.robot.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        26,
        0, 0, 0, 0, 0, 0
    )
        
        msg = self.robot.recv_match(type='SCALED_IMU', blocking=True)
        if msg:
            xacc = msg.xacc; yacc= msg.yacc; zacc = msg.zacc
            xgryo = msg.xgyro; ygryo = msg.ygyro; zgyro = msg.zgyro
            # xmag = msg.xmag; ymag = msg.ymag; zmag= msg.zmag
            return xacc, yacc, zacc, xgryo, ygryo, zgyro
        else:
            print("No response received")
            return None
    
    def grabCompass(self):
        self.robot.mav.command_long_send(
        self.robot.target_system,
        self.robot.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        74,
        0, 0, 0, 0, 0, 0
    )
        msg = self.robot.recv_match( type = "VFR_HUD", blocking = True)
        if msg is None:
            return None
        else:
            msg.heading
        
    
    
    def updateVelocities(self):
        xacc, yacc, zacc = self.grabIMU()
        self.xVel += xacc; self.yVel += yacc; self.zVel += zacc
        return self.xVel, self.yVel, self.zVel

    ##-----------------MOTION CONTROL----------------------
    ##-----------------------------------------------------

    def disableThrust(self):
        self.thrust1 = 1500
        self.thrust2 = 1500
        self.thrust3 = 1500
        self.thrust4 = 1500
    
    def apply_deadband_pulse(channel_id, pwm):
        offset = pwm - 1500
        if(channel_id not in {1,2,3,4,5,6}): #lights or something
            return pwm
        if abs(offset) < 5:
            return 1500
        if abs(offset) > PULSE_DEADBAND:
            return pwm
        duty_cycle = abs(offset) / PULSE_DEADBAND
        phase = (time.monotonic() % PULSE_PERIOD) / PULSE_PERIOD

        if phase < duty_cycle:
            pulsed_offset = PULSE_DEADBAND if offset > 0 else -PULSE_DEADBAND
        else:
            pulsed_offset = 0
        return 1500 + pulsed_offset

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        # print("Running at", pwm)
        if(pwm == 0):
            pwm = 1500
        pwm = self.apply_deadband_pulse(channel_id, pwm)
        """ Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900
        """
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self.robot.mav.rc_channels_override_send(
            self.robot.target_system,                # target_system
            self.robot.target_component,             # target_component
            *rc_channel_values)  

    def stopThruster(self):
        for i in range(1, 6):
            self.set_rc_channel_pwm(i, 1500)

    def goForward(self, offset):
        if(offset is None):
            offset = 0
        else:
            offset = offset        
        self.thrust1 = self.thrust1-offset
        self.thrust2 = self.thrust2-offset
        self.thrust3 = self.thrust3+offset
        self.thrust4 = self.thrust4+offset
        # self.set_rc_channel_pwm(1, 1500-offset)
        # self.set_rc_channel_pwm(2, 1500-offset)
        # self.set_rc_channel_pwm(3, 1500+offset)
        # self.set_rc_channel_pwm(4, 1500+offset)

    def goForwardFront(self, offset):
        if(offset is None):
            offset = 0
        else:
            offset = offset        
        self.thrust1 = self.thrust1+offset
        self.thrust2 = self.thrust2+offset
        # self.set_rc_channel_pwm(1, 1500-offset)
        # self.set_rc_channel_pwm(2, 1500-offset)

        # self.set_rc_channel_pwm(3, 1500)
        # self.set_rc_channel_pwm(4, 1500)
    
    def goForwardBack(self, offset):
        if(offset is None):
            offset = 0
        else:
            offset = offset
        self.thrust3 = self.thrust3+offset
        self.thrust4 = self.thrust4+offset


        # self.set_rc_channel_pwm(1, 1500)
        # self.set_rc_channel_pwm(2, 1500)
        # self.set_rc_channel_pwm(3, 1500+offset)
        # self.set_rc_channel_pwm(4, 1500+offset)

    def goVertical(self, offset):
        if(offset is None):
            offset = 0
        else:
            offset = offset
        self.set_rc_channel_pwm(5, 1500+offset)
        self.set_rc_channel_pwm(6, 1500+offset)

    def strafe(self, offset):
        if(offset is None):
            offset = 0
        else:
            offset = offset

        self.thrust1 = self.thrust1-offset
        self.thrust2 = self.thrust2+offset
        self.thrust3 = self.thrust3-offset
        self.thrust4 = self.thrust4+offset

        # self.set_rc_channel_pwm(1, 1500+offset)
        # self.set_rc_channel_pwm(2, 1500-offset)
        # self.set_rc_channel_pwm(3, 1500+offset)
        # self.set_rc_channel_pwm(4, 1500-offset)
    
    # def strafeLeft(self, offset):
    #     self.set_rc_channel_pwm(1, 1500+offset)
    #     self.set_rc_channel_pwm(3, 1500+offset)
    #     self.set_rc_channel_pwm(2, 1500)
    #     self.set_rc_channel_pwm(4, 1500)

    # def strafeRight(self, offset):
    #     self.set_rc_channel_pwm(1, 1500)
    #     self.set_rc_channel_pwm(3, 1500)
    #     self.set_rc_channel_pwm(2, 1500-offset)
    #     self.set_rc_channel_pwm(4, 1500-offset)

    def turn(self, offset):
        if(offset is None):
            offset = 0
        else:
            offset = offset
        
        self.thrust1 = self.thrust1+offset
        # self.thrust2 = self.thrust2-offset
        # self.thrust3 = self.thrust3-offset
        self.thrust4 = self.thrust4+offset

        # self.set_rc_channel_pwm(1, 1500+offset)
        # self.set_rc_channel_pwm(2, 1500-offset)
        # self.set_rc_channel_pwm(3, 1500-offset)
        # self.set_rc_channel_pwm(4, 1500+offset)
    
    def updateThrusts(self):
        self.set_rc_channel_pwm(1, self.thrust1)
        self.set_rc_channel_pwm(2, self.thrust2)
        self.set_rc_channel_pwm(3, self.thrust3)
        self.set_rc_channel_pwm(4, self.thrust4)
        


    ##-----------------LIGHTS CONTROL----------------------
    ##-----------------------------------------------------


    def lights(self, pwm):
        self.set_rc_channel_pwm(9, pwm)

    def lightsOn(self):
        print("Lights on")
        self.lights(1900)

    def lightsOff(self):
        self.lights(1100)
    

##Plotting PIDs: To be implemented in controller.py
  

def plot_measurements(PID):
        plt.plot(PID.times, PID.measurements)
        plt.xlabel("Time")
        plt.ylabel("Measurement")
        plt.title(PID.name + " Measurements")
        plt.grid(True)
        plt.show()
    


