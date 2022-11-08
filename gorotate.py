#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class GoForward():
    def __init__(self):
        rospy.init_node('GoForward', anonymous=False)  
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        r = rospy.Rate(10);

        move_cmd = Twist()
        move_cmd.linear.x = 0
	move_cmd.angular.z = 0.2

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            r.sleep()
                        
        
    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        GoForward()
    except:
        rospy.loginfo("GoForward node terminated.")

