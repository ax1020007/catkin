#! /usr/bin/env python
# coding=utf-8

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_get_data', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")
group_gripper = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

#---joint信息(/joint_states) ---
print "馬達信息(/joint_states): %s" %group_arm.get_current_joint_values()

#末端執行器的當前姿態. 
print "%s" %group_arm.get_current_pose()

moveit_commander.roscpp_shutdown()
