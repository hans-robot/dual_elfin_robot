#!/usr/bin/env python2
# -*- coding: utf-8 -*-
 
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
 
 
class test:
    def __init__(self):
 
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('test')
                
        left_arm = moveit_commander.MoveGroupCommander('elfin_left_arm')
        left_end_effector_link = left_arm.get_end_effector_link()       
        left_reference_frame = 'base_link'
        left_arm.set_pose_reference_frame(left_reference_frame)
        left_arm.allow_replanning(True)
        left_arm.set_goal_position_tolerance(0.001)
        left_arm.set_goal_orientation_tolerance(0.01)
        left_arm.set_max_acceleration_scaling_factor(0.5)
        left_arm.set_max_velocity_scaling_factor(0.5)
        left_arm.set_named_target('pose1')

        right_arm = moveit_commander.MoveGroupCommander('elfin_right_arm')
        right_end_effector_link = right_arm.get_end_effector_link()       
        right_reference_frame = 'base_link'
        right_arm.set_pose_reference_frame(right_reference_frame)
        right_arm.allow_replanning(True)
        right_arm.set_goal_position_tolerance(0.001)
        right_arm.set_goal_orientation_tolerance(0.01)
        right_arm.set_max_acceleration_scaling_factor(0.5)
        right_arm.set_max_velocity_scaling_factor(0.5)

        left_arm.go()
        rospy.sleep(1)
               

        target_pose = PoseStamped()
        target_pose.header.frame_id = left_reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.2593
        target_pose.pose.position.y = 0.0636
        target_pose.pose.position.z = 0.1787
        target_pose.pose.orientation.x = 0.70692
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.70729
        
        left_arm.set_start_state_to_current_state()
        left_arm.set_pose_target(target_pose, left_end_effector_link)
        left_traj = left_arm.plan()
        print(left_traj)

        left_arm.execute(left_traj)
        rospy.sleep(1)
 
        right_arm.set_named_target('pose2')
        right_arm.go()
 
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    test()
