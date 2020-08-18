#!/usr/bin/python
import rospy
from xbot_sensors.msg import LineFollower
from xbot_sensors.ir import IR

class LineFollower_Node():
    def __init__(self):
        self.line_sensor = IR()
        self.NODE_RATE = 30 #Hz
        rospy.init_node('line_follower', anonymous=False)
        self.pub_ir = rospy.Publisher('/sensors/line_follower', LineFollower, queue_size=10)
 
    def spin(self):
        self.rate = rospy.Rate(self.NODE_RATE) 
        while not rospy.is_shutdown():
            self.readings = self.line_sensor.read_line()
            rospy.loginfo(self.readings)
            self.pub(self.readings)
            self.rate.sleep()
    
    def pub(self,readings):
        msg = LineFollower()
        msg.left_ir = readings[0]
        msg.center_ir = readings[1]
        msg.right_ir = readings[2]
        self.pub_ir.publish(msg)

if __name__ == '__main__':
    run = LineFollower_Node()
    run.spin()
