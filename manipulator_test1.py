#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf, math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from open_manipulator_msgs.srv import *
from geometry_msgs.msg import PoseStamped, Pose

###### Setup ########
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_execute_trajectory', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
arm_group = moveit_commander.MoveGroupCommander(group_name)
reference_frame = 'world'
arm_group.set_pose_reference_frame(reference_frame)
arm_group.allow_replanning(True)        
arm_group.set_goal_position_tolerance(0.01)
arm_group.set_goal_orientation_tolerance(0.01)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
arm_group.set_planning_time(10);
arm_group.set_goal_position_tolerance(0.01)
arm_group.set_goal_orientation_tolerance(0.01)
pose_goal = geometry_msgs.msg.Pose()
###### Main ########
print "Move..."
pose_goal.position.x = 0.147795666194
pose_goal.position.y = 0.0
pose_goal.position.z = 0.0668806098937
pose_goal.orientation.x = 0.0
pose_goal.orientation.y = 0.6122038096
pose_goal.orientation.z = 0.0
pose_goal.orientation.w = 0.790700003485

arm_group.set_pose_target(pose_goal)
go = arm_group.go(wait=True)
variable = arm_group.get_current_pose()
print (variable.pose)
print "%s" %arm_group.get_current_joint_values()
arm_group.stop()
arm_group.clear_pose_targets()
moveit_commander.roscpp_shutdown()
