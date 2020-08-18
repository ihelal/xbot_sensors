#!/usr/bin/python
import RPi.GPIO as GPIO
import time
from mybot_sdk.robot_setup import get_robot_cfg

class Ultrasonic_Sensors():
    def __init__(self,side="ALL"):
        Ultrasonic = get_robot_cfg()["Sensors"]["Ultrasonic"]
        self.GPIO_TRIGGER = Ultrasonic["Trigger"]
        self.GPIO_ECHO_F = Ultrasonic["Front"]
        self.GPIO_ECHO_B = Ultrasonic["Back"]
        self.GPIO_ECHO_R = Ultrasonic["Right"]
        self.GPIO_ECHO_L = Ultrasonic["Left"]
        self.MAX_RANGE = Ultrasonic["MaxRange"]

        self.f_us = 0
        self.b_us = 0
        self.r_us = 0
        self.l_us = 0

        self.ACCURACY = 2 #Deciaml Places
        self.init_us()

        self.req_side = side
        self.FRONT = "FRONT"
        self.BACK  = "BACK"
        self.RIGHT = "RIGHT"
        self.LEFT  = "LEFT"
        self.ALL   = "ALL"

    def init_us(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO_F, GPIO.IN)
        GPIO.setup(self.GPIO_ECHO_B, GPIO.IN)
        GPIO.setup(self.GPIO_ECHO_R, GPIO.IN)
        GPIO.setup(self.GPIO_ECHO_L, GPIO.IN)
        

    def distance(self, side):
        # set Trigger to HIGH
        GPIO.output(self.GPIO_TRIGGER, True)
        # # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
    
        StartTime = time.time()
        StopTime = time.time()
    
        # save StartTime
        while GPIO.input(side) == 0:
            StartTime = time.time()
    
        # save time of arrival
        while GPIO.input(side) == 1:
            StopTime = time.time()
    
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = TimeElapsed * 17150

        if(distance > self.MAX_RANGE):
            return self.MAX_RANGE 
        else:
            return distance
        
    
    def measure_dist(self):  
        # self.f_us = self.distance(self.GPIO_ECHO_F)
        # self.b_us = self.distance(self.GPIO_ECHO_B)
        # self.l_us = self.distance(self.GPIO_ECHO_L)
        self.r_us = self.distance(self.GPIO_ECHO_R)  

    def read_us(self):
        self.measure_dist()
        front_us = round(self.f_us,self.ACCURACY)
        back_us  = round(self.b_us,self.ACCURACY)
        right_us = round(self.r_us,self.ACCURACY)
        left_us  = round(self.l_us,self.ACCURACY)

        if(self.req_side == self.FRONT):
            return front_us
        elif(self.req_side == self.BACK):
            return back_us
        elif(self.req_side == self.RIGHT):
            return right_us 
        elif(self.req_side == self.LEFT):
            return left_us 
        elif(self.req_side == self.ALL):
            return front_us,back_us,left_us,right_us

# if __name__ == '__main__':
#     run = Ultrasonic_Sensors()
#     while 1:
#         print(run.read_us())
    
