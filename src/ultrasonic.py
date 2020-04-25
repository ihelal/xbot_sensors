#!/usr/bin/python
import rospy
import RPi.GPIO as GPIO
import time

from xbot_sensors.msg import Ultrasonic

class ultrasonic():
    def __init__(self):
        self.GPIO_TRIGGER = 12
        self.GPIO_ECHO_F = 8
        self.GPIO_ECHO_B = 7
        self.GPIO_ECHO_R = 25
        self.GPIO_ECHO_L = 1

        self.f_us = 0
        self.b_us = 0
        self.r_us = 0
        self.l_us = 0

        self.init_us()

        rospy.init_node('ultrasonic_sensors', anonymous=False)
        # pub = rospy.Publisher('topic_name', String, queue_size=10)
        # rospy.Publisher("/cmd_vel", Twist, self.cb_Twist)
        
    def init_us(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO_F, GPIO.IN)
        GPIO.setup(self.GPIO_ECHO_B, GPIO.IN)
        GPIO.setup(self.GPIO_ECHO_R, GPIO.IN)
        GPIO.setup(self.GPIO_ECHO_L, GPIO.IN)
        GPIO.cleanup()
        
        GPIO.setmode(GPIO.BCM)
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
        distance = (TimeElapsed * 34300) / 2
    
        return distance

    def read_us(self):
        self.f_us = self.distance(self.GPIO_ECHO_F)
        self.b_us = self.distance(self.GPIO_ECHO_B)
        self.l_us = self.distance(self.GPIO_ECHO_L)
        self.r_us = self.distance(self.GPIO_ECHO_R)

        return [self.f_us,self.b_us,self.l_us,self.r_us]; 
    
    def spin(self):
        self.rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.dist = self.read_us()
            self.pub(self.dist)
            #print (self.dist)
            self.rate.sleep()
        GPIO.cleanup()
    
    def pub(self,dist):
        
        pub.publish("hello world")

if __name__ == '__main__':
    run = ultrasonic()
    run.spin()
    