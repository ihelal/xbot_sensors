#!/usr/bin/python3
import RPi.GPIO as GPIO
import time
from mybot_sdk.robot_setup import Motor,Brain
from xbot_drivers.base import Base
import matplotlib.pyplot as plt
import numpy as np

import math

# class Encoder():
#     def __init__(self):
#         self.encoder_val = 0    
#     def callback(self,channel):
#         self.encoder_val += 1
    
#     return self.encoder_val


class Encoders():
    def __init__(self):
        self.lf_en_val = 0
        self.rf_en_val = 0
        self.lb_en_val = 0
        self.rb_en_val = 0

        
        self.brain = Brain()
        self.brain.initEncoders()
        self.init_encoders()
        self.m = Motor()
        self.mybot_base = Base()
        self.d = 0

        self.REV = math.pi * 60
        self.TICK = self.REV/384
        
    def init_encoders(self):
        GPIO.add_event_detect(7,GPIO.FALLING,callback=self.lf_en_cb)
        # self.brain.setPull(7,GPIO.FALLING,self.lf_en_cb(1,1))
        # self.brain.setPull(7,GPIO.FALLING,self.lf_en_cb)
        # self.brain.setPull(7,GPIO.FALLING,self.lf_en_cb)
        # self.brain.setPull(7,GPIO.FALLING,self.lf_en_cb)

    def setPull(self,pin,stat,function_name):
        GPIO.add_event_detect(pin,stat,callback=function_name)
        # return 1

    def lf_en_cb(self,channel):
        print("")
        self.lf_en_val += 1
        print("LeftFront: " + str(self.lf_en_val*self.TICK) + "mm")
    
    def rf_en_cb(self,channel):
        print("")
        self.rf_en_val += 1
        print(self.rf_en_val)
    
    def lb_en_cb(self,channel):
        print("")
        self.lb_en_val += 1
        print(self.lb_en_val)
    
    def rb_en_cb(self,channel):
        print("")
        self.rb_en_val += 1
        print(self.rb_en_val)
    
    # def spin(self):
    #     self.rate = rospy.Rate(10) # 10hz
    #     while not rospy.is_shutdown():
    #         self.rate.sleep()
    #     GPIO.cleanup()
    
    # def pub(self,value):
    #     msg = Int64()
    #     msg.data = value
    #     self.pub_switch.publish(msg)
        
    def getEncoderValue(self,pin):
        pass

    def gg(self):
        d = self.lf_en_val * 0.49
        return d

    def dist(self):
        while self.d < 10:
            self.mybot_base.spassetMotors(70,1,0,0,0)



if __name__ == '__main__':
    run = Encoders()
    while True:
        # Plot the data
        plt.plot(x, x, label='linear')

        # Add a legend
        plt.legend()

        # Show the plot
        plt.show()





    
