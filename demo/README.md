使用说明
======

本功能包为双臂Moveit与真机控制的示例程序，使用过程中请确保安全！

在使用过程中请确保在Moveit开始规划前仿真中的双臂位置与真机双臂位置一致
### 使用说明
示例程序通过获取Moveit规划的点位，将点位通过标准SDK接口发送给机械臂控制器进行运行
步骤：

    1. 将demo/lib中的libHR_Pro.so放入/usr/lib目录下
    2. 打开src/dual_elfin_demo.cpp，确定在ROS的PC下能够ping同两台机械臂后，修改连接真实机器人HRIF_Connect接口中的IP地址为真实机器人的IP地址，请注意分清真实机器人IP与Moveit中的左右臂的定义
    3. 重新编译demo, catkin_make -DCATKIN_WHITELIST_PACKAGES='demo'
    4. 启动Moveit仿真，参考dual_elfin_robot/README.md
    5. 运行示例程序: rosrun demo demo_node
    6. 用户可自行模改控制双臂程序,在使用过程中谨防撞机