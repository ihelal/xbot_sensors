#!/usr/bin/python
import RPi.GPIO as GPIO
import time
from mybot_sdk.robot_setup import get_robot_cfg

class Switch():
    def __init__(self):
        SwitchSensor = get_robot_cfg()["Accesories"]["ToggleButton"]
        self.GPIO_TRIGGER = SwitchSensor
        self.sensor_value = 0
        self.init_switch()
        
    def init_switch(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
        GPIO.add_event_detect(self.GPIO_TRIGGER,GPIO.RISING,callback=self.button_callback) # Setup event on pin 10 rising edge

    def button_callback(self,channel):
        print("Button was pushed!")
        self.sensor_value = 1

    def read_switch(self):
        return self.sensor_value
    