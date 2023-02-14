#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from CPS import CPSClient
import sys
import rospy
import moveit_msgs.msg
from std_msgs.msg import Float32MultiArray

class dual_elfin_sub_demo:
    def __init__(self):
        rospy.init_node('dual_elfin_sub_node', anonymous=True)
        self.left_plan_sub = rospy.Subscriber("elfin_left_arm_traj",moveit_msgs.msg.RobotTrajectory,self.send_left_arm)
        self.right_plan_sub = rospy.Subscriber("elfin_right_arm_traj",moveit_msgs.msg.RobotTrajectory,self.send_right_arm)
        
        self.cps = CPSClient()
        self.left_robot_id = 0
        self.right_robot_id = 1

        left_robot_ip = "192.168.3.190"
        right_robot_ip = "192.168.3.162"

        self.left_nRet = self.cps.HRIF_Connect(self.left_robot_id,left_robot_ip,10003)
        self.right_nRet = self.cps.HRIF_Connect(self.right_robot_id,right_robot_ip,10003)
        if(self.left_nRet != 0 or self.right_nRet != 0):
            rospy.logerr("Connect Robot Fail!!!")
            rospy.signal_shutdown("Connect Robot Fail...")
            return
        else:
            rospy.loginfo("Connect Robot Success!!!")
    
    def send_left_arm(self, data):
        for i in range(0, len(data.joint_trajectory.points)):
            left_traj = []
            for j in range(0, len(data.joint_trajectory.points[i].positions)):
                left_traj.append(data.joint_trajectory.points[i].positions[j]*57.3)
            left_waypoint_res = self.cps.HRIF_WayPoint(self.left_robot_id, 0, 0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            left_traj,"TCP", "Base", 100, 100, 0, 1, 0, 0, 0, " ")

    def send_right_arm(self, data):
        for i in range(0, len(data.joint_trajectory.points)):
            right_traj = []
            for j in range(0, len(data.joint_trajectory.points[i].positions)):
                right_traj.append(data.joint_trajectory.points[i].positions[j]*57.3)
            right_waypoint_res = self.cps.HRIF_WayPoint(self.right_robot_id, 0, 0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            right_traj,"TCP", "Base", 100, 100, 0, 1, 0, 0, 0, " ")

if __name__=='__main__':
       test = dual_elfin_sub_demo()
       rospy.spin()