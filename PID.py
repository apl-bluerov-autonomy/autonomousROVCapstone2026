import time
import numpy as np

class PID:
    def __init__(self, kp=0, ki=0, kd=0, target=0.0, tol = 0.1,
                 pwm_min=1100, pwm_max = 1900, pwm_trim = 100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.tol = tol
        self.pwm = 1500
        
        self.integral = 0.0
        self.prev_error = 0.0
        
        self.prev_time = time.monotonic()
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.pwm_trim = pwm_trim

        self.integral_lim = 500

    def updateTarget(self, target):
        self.target = target
    
    def getPWM(self):
        return int(self.pwm)
    
    def scaleDown(self):
        if(self.pwm > 1500):
            self.pwm = self.pwm + 10
        elif(self.pwm < 1500):
            self.pwm = self.pwm - 10 #def a better way to do this (like dif between self.pwm and 1500 when close to 1500)
        else:
            self.pwm = self.pwm



    def updateTol(self, tol):
        self.tol = tol

    def atTarget(self, measurement):
        if(measurement  == self.target + self.tol or measurement == self.target - self.tol):
            return True
        else:
            return False

    def update(self, measurement):
        # Time step
        current_time = time.monotonic()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt <= 0.0:
            return 0.0

        # Error
        error = self.target - measurement

        # Integral
        self.integral += error * dt

        # Derivative
        derivative = (error - self.prev_error) / dt

        # PID output
        b = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )
        
        self.pwm = 1500 + b
        delta = 1500 - self.pwm

        # if(np.abs(delta) < 10):
        #     self.pwm = 1500
        # if(np.abs(delta) > 10):
        #     if(delta > 0):
        #         self.pwm = self.pwm - 20
        #     else:
        #         self.pwm = self.pwm + 20

        #Keep it between 1100, 1900

        self.pwm = max(min(self.pwm, self.pwm_max), self.pwm_min)

        # Store for next step
        self.prev_error = error
        # return output
        print(int(self.pwm))
        return int(self.pwm)