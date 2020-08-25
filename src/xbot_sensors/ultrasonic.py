#!/usr/bin/python3
import RPi.GPIO as GPIO
from mybot_sdk.robot_setup import get_robot_cfg
from xbot_sensors.hcsr04 import Measurement

class UltrasnoicSensor():

    def __init__(self, side='ALL'):
        self.GPIO_MODE = GPIO.BCM

        Ultrasonic = get_robot_cfg()['Sensors']['Ultrasonic']
        self.GPIO_TRIGGER = Ultrasonic['Trigger']
        self.GPIO_ECHO_F = Ultrasonic['Front']
        self.GPIO_ECHO_B = Ultrasonic['Back']
        self.GPIO_ECHO_R = Ultrasonic['Right']
        self.GPIO_ECHO_L = Ultrasonic['Left']
        self.MAX_RANGE = Ultrasonic['Range']['Max']

        self.ACCURACY = 2
        self.TEMP = 25
        self.SAMPLES = 2
        self.RATE = 0.04
        self.SYSTEM = "metric"

        self.us1,self.us2,self.us3,self.us4 = self.init_us()

    def init_us(self):
        us1 = Measurement(self.GPIO_TRIGGER, self.GPIO_ECHO_F, self.TEMP, self.SYSTEM, self.GPIO_MODE)
        us2 = Measurement(self.GPIO_TRIGGER, self.GPIO_ECHO_L, self.TEMP, self.SYSTEM, self.GPIO_MODE)
        us3 = Measurement(self.GPIO_TRIGGER, self.GPIO_ECHO_B, self.TEMP, self.SYSTEM, self.GPIO_MODE)
        us4 = Measurement(self.GPIO_TRIGGER, self.GPIO_ECHO_R, self.TEMP, self.SYSTEM, self.GPIO_MODE)

        return us1,us2,us3,us4

    def cm_to_m(self,cm):
        m = cm/100.00
        return m

    def get_readings_cm(self):
        f = self.us1.raw_distance(self.SAMPLES,self.RATE)
        l = self.us2.raw_distance(self.SAMPLES,self.RATE)
        b = self.us3.raw_distance(self.SAMPLES,self.RATE)
        r = self.us4.raw_distance(self.SAMPLES,self.RATE)

        return f,l,b,r
    
    def get_readings_m(self):
        f,l,b,r = self.get_readings_cm()

        f = round(f,self.ACCURACY)
        l = round(l,self.ACCURACY)
        b = round(b,self.ACCURACY)
        r = round(r,self.ACCURACY)

        return f,l,b,r
    
    def get_us_side_cm(self,side):
        if side == 0:
            return self.get_readings_cm()[0]
        if side == 1:
            return self.get_readings_cm()[1]
        if side == 2:
            return self.get_readings_cm()[2]
        if side == 3:
            return self.get_readings_cm()[3]
        else:
            raise SystemError("Invalid Requested Ultrasonic")
    
    def get_us_side_m(self,side):
        val = self.get_us_side_cm(side)
        val = self.cm_to_m(val)
        val = round(val,self.ACCURACY)

        return val
