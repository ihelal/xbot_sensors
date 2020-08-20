#!/usr/bin/python
import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Int64

class Encoders():
    def __init__(self):
        self.LF_ENCODER_PIN = 14
        self.RF_ENCODER_PIN = 15
        self.LB_ENCODER_PIN = 18
        self.RB_ENCODER_PIN = 23

        self.lf_en_val = 0
        self.rf_en_val = 0
        self.lb_en_val = 0
        self.rb_en_val = 0

        self.init_encoders()
        rospy.init_node('encoder_sensor', anonymous=False)
        self.pub_switch = rospy.Publisher('/encoder', Int64, queue_size=10)
        
    def init_encoders(self):
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.LF_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.RF_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.LB_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.RB_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.LF_ENCODER_PIN,GPIO.RISING,callback=self.lf_en_cb)
        GPIO.add_event_detect(self.LF_ENCODER_PIN,GPIO.RISING,callback=self.rf_en_cb)
        GPIO.add_event_detect(self.LF_ENCODER_PIN,GPIO.RISING,callback=self.lb_en_cb)
        GPIO.add_event_detect(self.LF_ENCODER_PIN,GPIO.RISING,callback=self.rb_en_cb)

    def lf_en_cb(self,channel):
        print("")
        self.lf_en_val += 1
        print(self.lf_en_val)
    
    def rf_en_cb(self,channel):
        print("")
        self.rf_en_val += 1
        print(self.lf_en_val)
    
    def lb_en_cb(self,channel):
        print("")
        self.lb_en_val += 1
        print(self.lf_en_val)
    
    def rb_en_cb(self,channel):
        print("")
        self.rb_en_val += 1
        print(self.lf_en_val)
    
    def spin(self):
        self.rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.rate.sleep()
        GPIO.cleanup()
    
    def pub(self,value):
        msg = Int64()
        msg.data = value
        self.pub_switch.publish(msg)
        
    def getEncoderValue(self,pin):
        pass

if __name__ == '__main__':
    run = Encoders()
    run.spin()
    
