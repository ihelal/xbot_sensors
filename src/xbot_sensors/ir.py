#!/usr/bin/python3
import time
from mybot_sdk.robot_setup import Brain

class IR():
    def __init__(self):
        self.brain = Brain()
        self.LineFollower = self.brain.initIR()
        self.L = self.LineFollower[0]
        self.C = self.LineFollower[1]
        self.R = self.LineFollower[2]

    def read_line(self):
        L_VAL = (self.brain.get_input_status(self.L) == True)
        C_VAL = (self.brain.get_input_status(self.C) == True)
        R_VAL = (self.brain.get_input_status(self.R) == True) 
        return L_VAL,C_VAL,R_VAL

# if __name__ == '__main__':
#     run = IR()
#     while True:
#        print(run.read_line())
#        time.sleep(0.03333)