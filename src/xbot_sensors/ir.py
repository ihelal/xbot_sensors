#!/usr/bin/python
import RPi.GPIO as GPIO
import time
from mybot_sdk.robot_setup import get_robot_cfg

class IR():
    def __init__(self):
        LineFollower = get_robot_cfg()["Sensors"]["LineFollower"]
        self.LINE_LEFT = LineFollower["L"]
        self.LINE_CENTER = LineFollower["C"]
        self.LINE_RIGHT = LineFollower["R"]
        self.init_ir()

    def init_ir(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LINE_LEFT, GPIO.IN)
        GPIO.setup(self.LINE_CENTER, GPIO.IN)
        GPIO.setup(self.LINE_RIGHT, GPIO.IN)

    def read_line(self):
        L_VAL = (GPIO.input(self.LINE_LEFT) == True)
        C_VAL = (GPIO.input(self.LINE_CENTER) == True)
        R_VAL = (GPIO.input(self.LINE_RIGHT) == True) 
        return L_VAL,C_VAL,R_VAL

# if __name__ == '__main__':
#     run = IR()
#     while 1:
#        print(run.read_line())