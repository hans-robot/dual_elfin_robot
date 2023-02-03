#include <iostream>
#include "HR_Pro.h"
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <fstream>

#include <ros/ros.h>
#include<moveit/move_group_interface/move_group_interface.h>
#include<moveit/robot_trajectory/robot_trajectory.h>
using namespace std;

int main(int argc,char ** argv)
{

    ros::init(argc, argv, "dual_elfin_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    //左臂规划组
    moveit::planning_interface::MoveGroupInterface left_arm("elfin_left_arm");
    left_arm.setGoalJointTolerance(0.0001);
    left_arm.setMaxAccelerationScalingFactor(0.5);
    left_arm.setMaxVelocityScalingFactor(0.5);

    //右臂规划组
    moveit::planning_interface::MoveGroupInterface right_arm("elfin_right_arm");
    right_arm.setGoalJointTolerance(0.0001);
    right_arm.setMaxAccelerationScalingFactor(0.5);
    right_arm.setMaxVelocityScalingFactor(0.5);

    //左臂规划
    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    //右臂规划
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;

    //连接真实机器人
    int left_robot_id = 0;
    int right_robot_id = 1;
    int left_nRet = HRIF_Connect(left_robot_id, "192.168.3.140", 10003);
    int right_nRet = HRIF_Connect(right_robot_id, "192.168.3.162", 10003);
    if(left_nRet !=0 || right_nRet != 0){
        ROS_ERROR("Connect Robot Fail!!!");
        ros::shutdown(); 
        return 0;
    }else{
        ROS_INFO("Connect Robot Success!!!");
    }
 
    left_arm.setNamedTarget("pose1");
    left_arm.move(); 
    sleep(1);
    
    //左臂关节运动
    double targetPose[6] = {0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125};
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];

    //右臂关节运动
    double right_targetPose[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<double> right_joint_group_positions(6);
    right_joint_group_positions[0] = right_targetPose[0];
    right_joint_group_positions[1] = right_targetPose[1];
    right_joint_group_positions[2] = right_targetPose[2];
    right_joint_group_positions[3] = right_targetPose[3];
    right_joint_group_positions[4] = right_targetPose[4];
    right_joint_group_positions[5] = right_targetPose[5];
    
    left_arm.setJointValueTarget(joint_group_positions);
    right_arm.setJointValueTarget(right_joint_group_positions);

    //规划运动
    moveit::planning_interface::MoveItErrorCode success1 = left_arm.plan(left_plan);
    moveit::planning_interface::MoveItErrorCode success2 = right_arm.plan(right_plan);

    if(success1)
      left_arm.execute(left_plan);
      moveit_msgs::RobotTrajectory trajectory;
      //获取规划轨迹
      trajectory.joint_trajectory.joint_names = left_plan.trajectory_.joint_trajectory.joint_names;
      trajectory.joint_trajectory.points = left_plan.trajectory_.joint_trajectory.points;
      for(unsigned i = 0; i < left_plan.trajectory_.joint_trajectory.points.size(); i++)
      {
        double send_traget_angle[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        for(unsigned j =0;j < left_plan.trajectory_.joint_trajectory.points[i].positions.size();j++)
        {
            send_traget_angle[j] = left_plan.trajectory_.joint_trajectory.points[i].positions[j] * 57.3;
        }
        //通过SDK接口发送轨迹到真实机器人
        int left_waypoint_res = HRIF_WayPoint(left_robot_id, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        send_traget_angle[0], send_traget_angle[1], send_traget_angle[2], send_traget_angle[3], send_traget_angle[4], send_traget_angle[5],
        "TCP", "Base", 100, 100, 0, 1, 0, 0, 0, " ");
        std::cout<<"left_arm: "<<left_waypoint_res<<std::endl;
      }


    if(success2){
        right_arm.execute(right_plan);
        moveit_msgs::RobotTrajectory right_trajectory;
        //获取规划轨迹
        right_trajectory.joint_trajectory.joint_names = right_plan.trajectory_.joint_trajectory.joint_names;
        right_trajectory.joint_trajectory.points = right_plan.trajectory_.joint_trajectory.points;
        for(unsigned i = 0; i < right_plan.trajectory_.joint_trajectory.points.size(); i++)
        {
            double send_traget_angle[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            for(unsigned j =0;j < right_plan.trajectory_.joint_trajectory.points[i].positions.size();j++)
            {
                send_traget_angle[j] = right_plan.trajectory_.joint_trajectory.points[i].positions[j] * 57.3;
            }
            //通过SDK接口发送轨迹到真实机器人
            int right_waypoint_res = HRIF_WayPoint(right_robot_id, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            send_traget_angle[0], send_traget_angle[1], send_traget_angle[2], send_traget_angle[3], send_traget_angle[4], send_traget_angle[5],
            "TCP", "Base", 100, 100, 0, 1, 0, 0, 0, " ");
            std::cout<<"right_arm: "<<right_waypoint_res<<std::endl;
        }
    }
    sleep(1);
 
    left_arm.setNamedTarget("pose1");
    left_arm.move();
    sleep(1);

    right_arm.setNamedTarget("pose2");
    right_arm.move();
    sleep(1);
    
    ros::shutdown(); 
 
    return 0;
}