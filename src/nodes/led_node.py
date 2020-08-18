#!/usr/bin/python
import rospy
from std_msgs.msg import Int8
from xbot_sensors.led import LED

class LED_Node():
    def __init__(self):
        self.led = LED()
        self.NODE_RATE = 30 #Hz
        rospy.init_node('led', anonymous=False)
        self.pub = rospy.Publisher('/accessories/led', Int8, queue_size=10)
 
    def spin(self):
        self.rate = rospy.Rate(self.NODE_RATE) 
        while not rospy.is_shutdown():
            self.readings = self.led.read_line()
            rospy.loginfo(self.readings)
            self.pub(self.readings)
            self.rate.sleep()
    
    def pub(self,readings):
        msg = Int8()
        msg.data = readings
        self.pub_.publish(msg)

if __name__ == '__main__':
    run = LED_Node()
    run.spin()
