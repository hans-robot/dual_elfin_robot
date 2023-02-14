#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import Float32MultiArray


class dual_elfin_demo:
    def __init__(self):        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('dual_elfin_demo_node',anonymous=True)

        self.left_plan_pub=rospy.Publisher('elfin_left_arm_traj', moveit_msgs.msg.RobotTrajectory, queue_size=10)
        self.right_plan_pub=rospy.Publisher('elfin_right_arm_traj', moveit_msgs.msg.RobotTrajectory, queue_size=10)

        self.dual_robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.left_arm_group = moveit_commander.MoveGroupCommander("elfin_left_arm")
        self.left_arm_group.set_goal_position_tolerance(0.0001)
        self.left_arm_group.set_goal_orientation_tolerance(0.0001)
        self.left_arm_group.set_max_acceleration_scaling_factor(0.5)
        self.left_arm_group.set_max_velocity_scaling_factor(0.5)

        self.right_arm_group = moveit_commander.MoveGroupCommander("elfin_right_arm")
        self.right_arm_group.set_goal_position_tolerance(0.0001)
        self.right_arm_group.set_goal_orientation_tolerance(0.0001)
        self.right_arm_group.set_max_acceleration_scaling_factor(0.5)
        self.right_arm_group.set_max_velocity_scaling_factor(0.5)
        
    def move_demo(self):
        left_joint_goal = self.left_arm_group.get_current_joint_values()
        left_joint_goal[0] = 0
        left_joint_goal[1] = -pi/4
        left_joint_goal[2] = 0
        left_joint_goal[3] = -pi/2
        left_joint_goal[4] = 0
        left_joint_goal[5] = pi/3

        right_joint_goal = self.right_arm_group.get_current_joint_values()
        right_joint_goal[0] = 0
        right_joint_goal[1] = -pi/4
        right_joint_goal[2] = 0
        right_joint_goal[3] = -pi/2
        right_joint_goal[4] = 0
        right_joint_goal[5] = pi/3

        left_plan_traj = self.left_arm_group.plan(left_joint_goal)
        right_plan_traj = self.right_arm_group.plan(right_joint_goal)

        self.left_arm_group.execute(left_plan_traj, True)
        self.right_arm_group.execute(right_plan_traj, True)
        
        self.left_plan_pub.publish(left_plan_traj)
        self.right_plan_pub.publish(right_plan_traj) 

if __name__=='__main__':
       test = dual_elfin_demo()
       test.move_demo()
       rospy.spin()
