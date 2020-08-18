#!/usr/bin/python
import rospy
from std_msgs.msg import Int8
from xbot_sensors.switch import Switch

class Switch_Node():
    def __init__(self):
        self.switch_sensor = Switch()
        self.NODE_RATE = 30 #Hz
        rospy.init_node('switch', anonymous=False)
        self.pub_switch = rospy.Publisher('/sensors/switch', Int8, queue_size=10)
 
    def spin(self):
        self.rate = rospy.Rate(self.NODE_RATE) 
        while not rospy.is_shutdown():
            self.readings = self.switch_sensor.read_switch()
            rospy.loginfo(self.readings)
            self.pub(self.readings)
            self.rate.sleep()
    
    def pub(self,readings):
        msg = Int8()
        msg.data = readings
        self.pub_switch.publish(msg)

if __name__ == '__main__':
    run = Switch_Node()
    run.spin()