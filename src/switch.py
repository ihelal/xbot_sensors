#!/usr/bin/python
import rospy
import RPi.GPIO as GPIO
import time

from std_msgs.msg import Int32

class Switch():
    def __init__(self):
        self.GPIO_TRIGGER = 24
        self.sensor_value = 0
        self.init_switch()
        rospy.init_node('switch_sensor', anonymous=False)
        self.pub_switch = rospy.Publisher('/sensor_switch', Int32, queue_size=10)
        
    def init_switch(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
        GPIO.add_event_detect(self.GPIO_TRIGGER,GPIO.RISING,callback=self.button_callback) # Setup event on pin 10 rising edge

    def button_callback(self,channel):
        print("Button was pushed!")
        self.sensor_value = 1
    
    def spin(self):
        self.rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.sensor_value == 1:
                self.pub(1)
            else:
                self.pub(0)
            self.rate.sleep()
        GPIO.cleanup()
    
    def pub(self,value):
        msg = Int32()
        msg.data = value
        self.pub_switch.publish(msg)

if __name__ == '__main__':
    run = Switch()
    run.spin()
    