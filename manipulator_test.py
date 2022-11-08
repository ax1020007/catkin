#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

names = 'position'
values = [0.0,0.4,-0.25,1.35]

###### Functions ########
def open_gripper():
	print "Opening Gripper..."
	gripper_group_variable_values[0] = 00.000
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(2)

def close_gripper():
	print "Closing Gripper..."
	gripper_group_variable_values[0] = -00.033
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(2)

def move_home():
	arm_group.set_named_target("home")
	print "Executing Move: Home"
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	print "%s" %arm_group.get_current_joint_values()
	rospy.sleep(5)


def move_position():
	arm_group.set_named_target("position")
	print "Executing Move: Position"
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()

	variable = arm_group.get_current_pose()
	print (variable.pose)
	print "%s" %arm_group.get_current_joint_values()
	rospy.sleep(5)


###### Setup ########
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

#Had probelms with planner failing, Using this planner now. I believe default is OMPL
#arm_group.set_planner_id("RRTConnectkConfigDefault")
#Increased available planning time from 5 to 10 seconds
arm_group.set_planning_time(10);

arm_group.remember_joint_values(names, values)
gripper_group_variable_values = gripper_group.get_current_joint_values()

###### Main ########
#move_home()
open_gripper()
#move_position()
close_gripper()
#move_home()

moveit_commander.roscpp_shutdown()
