#!/usr/bin/python
import rospy
from xbot_sensors.msg import Ultrasonic
from xbot_sensors.ultrasonic import Ultrasonic_Sensors

class Ultrasonic_Node():
    def __init__(self):
        self.us_sensor = Ultrasonic_Sensors()
        self.NODE_RATE = 30 #Hz
        rospy.init_node('ultrasonic', anonymous=False)
        self.pub_us = rospy.Publisher('/sensors/ultrasonic', Ultrasonic, queue_size=10)
 
    def spin(self):
        self.rate = rospy.Rate(self.NODE_RATE) 
        while not rospy.is_shutdown():
            self.readings = self.us_sensor.read_us()
            print(self.readings)
            self.pub(self.readings)
            self.rate.sleep()
    
    def pub(self,data):
        msg = Ultrasonic()
        msg.front_us = data[0]
        msg.back_us = data[1]
        msg.left_us = data[2]
        msg.right_us = data[3]
        self.pub_us.publish(msg)

if __name__ == '__main__':
    run = Ultrasonic_Node()
    run.spin()
