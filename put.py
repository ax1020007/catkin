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

name1 = 'put'
values1 = [-0.0015,0.7,-0.55,0.55]

name2 = 'go_really'
values2 = [0,-0.6,-0.2,1.500]

def put():
	print("put")
	arm.set_named_target('put')
	arm.go()
	rospy.sleep(6)

def open_gripper():
	print "Opening Gripper..."
	gripper_group_variable_values[0] = 00.0
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(2)

def go_really():
	print("go_really")
	arm.set_named_target('go_really')
	arm.go()
	rospy.sleep(3)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_execute_trajectory', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander("arm")
arm.remember_joint_values(name1, values1)
arm.remember_joint_values(name2, values2)
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
put()
open_gripper()
go_really()
print("ok")
moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)
