#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf, math
import tf2_ros
from open_manipulator_msgs.srv import *

###### Setup ########
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
gripper_group_variable_values = gripper_group.get_current_joint_values()
arm.allow_replanning(True)
arm.set_pose_reference_frame('base_link')
end_effector_link = arm.get_end_effector_link()
start_pose = arm.get_current_pose(end_effector_link).pose
arm.set_start_state_to_current_state()
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
	try:
            	trans = tfBuffer.lookup_transform('base_link', 'fiducial_5', rospy.Time())
		print("%s " % trans.transform.translation.x)
		print("%s " % trans.transform.translation.y)
		print("%s " % trans.transform.translation.z)
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		rate.sleep()
            	continue
	print("ok")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
