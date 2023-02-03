使用说明
======

本功能包为双臂Moveit与真机控制的示例程序，使用过程中请确保安全！

### 使用说明
示例程序通过获取Moveit规划的点位，将点位通过标准SDK接口发送给机械臂控制器进行运行
步骤：

    1. 将demo/lib中的libHR_Pro.so放入/usr/lib目录下
    2. 打开src/dual_elfin_demo.cpp，修改连接真实机器人HRIF_Connect接口中的IP地址为真实机器人的IP地址，请注意分清真实机器人IP与Moveit中的左右臂的定义
    3. 启动Moveit仿真，参考elfin_arms/README.md
    4. 运行示例程序: rosrun demo demo_node
    5. 用户可自行模改控制双臂程序,在使用过程中谨防撞机