import time
import numpy as np

class PID:
    def __init__(self, kp=0, ki=0, kd=0, target=0.0, tol = 0.1,
                 min=-200, max = 200, trim = 100, name='noName'):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.tol = tol
        self.min = min
        self.max = max
        self.trim = trim
        self.offset = 0
        self.name = name
        self.measurement = None
        
        self.integral = 0.0
        self.prev_error = 0.0
        
        self.prev_time = time.monotonic()
        self.integral_lim = 50

    def updateTarget(self, target):
        self.target = target
    
    def getOffset(self):
        return int(self.offset)
    
    def getError(self):
        return self.target - self.measurement
    
    def resetInt(self):
        self.integral = 0

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
        self.measurement = measurement

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

        self.offset = b

        self.offset = max(min(self.offset, self.max), self.min)

        # Store for next step
        self.prev_error = error
        # return output
        #print(self.name, int(self.offset))
        return int(self.offset)
    
    def print_PID(self):
        print(self.name, int(self.offset))