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

name1 = 'really'
values1 = [0,-0.460,-0.210,1.600]

def home():
	print("home")
	arm.set_named_target('home')
	arm.go()
	rospy.sleep(3)

def position1():
	print "Move1..."
	start_pose.position.x = px - 0.01
	start_pose.position.y = py
	start_pose.position.z = 0.065

	start_pose.orientation.x = ox
	start_pose.orientation.y = oy
	start_pose.orientation.z = oz
	start_pose.orientation.w = ow
	arm.set_pose_target(start_pose)
	go = arm.go(wait=True)	
	rospy.sleep(5)

def open_gripper():
	print "Opening Gripper..."
	gripper_group_variable_values[0] = 00.0
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(1)

def close_gripper():
	print "Closing Gripper..."
	gripper_group_variable_values[0] = -0.033
 	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(3)

###### Setup ########
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander("arm")
arm.remember_joint_values(name1, values1)
gripper_group = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
gripper_group_variable_values = gripper_group.get_current_joint_values()
arm.allow_replanning(True)
arm.set_pose_reference_frame('base_link')
arm.set_goal_position_tolerance(0.005)
arm.set_goal_orientation_tolerance(0.5)
arm.set_max_acceleration_scaling_factor(0.5)
arm.set_max_velocity_scaling_factor(0.5)
end_effector_link = arm.get_end_effector_link()
start_pose = arm.get_current_pose(end_effector_link).pose
arm.set_start_state_to_current_state()
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
	try:
###### tf ########
            	trans = tfBuffer.lookup_transform('base_link', 'fiducial_4', rospy.Time())
		px = trans.transform.translation.x
		py = trans.transform.translation.y 
		pz = 0.05
		ox = 0.0
		oy = 0.6122038096
		oz = 0.0
		ow = 0.790700003485
###### Main ########
		######### move #########
		rospy.sleep(3)
		position1()
		close_gripper()
		home()
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		rate.sleep()
            	continue
	print("ok")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
