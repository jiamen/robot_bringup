//
// Created by zlc on 2021/5/26.
//
#include "../include/robot_bringup/robot.h"

double RobotV_ = 0;
double RobotYawRate_ = 0;

// 速度控制消息的回调函数
void cmdCallback(const geometry_msgs::Twist& msg)
{
    RobotV_ = msg.linear.x * 1000;      // mm/s
    RobotYawRate_ = msg.angular.z;      // rad/s
}


int main(int argc, char* *argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "mbot_bringup");
    ros::NodeHandle nh;

    // 初始化robot
    robot::robot my_robot;
    if (!my_robot.init())
        ROS_ERROR("my_robot initialized failed!");
    ROS_INFO("my_robot initialized successful.");

    // 订阅速度控制，可能来自键盘、手机App等的遥控，解析得到速度和角速度
    ros::Subscriber sub = nh.subscribe("cmd_vel", 50, cmdCallback);

    // 循环运行
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        // 机器人控制
        my_robot.deal(RobotV_, RobotYawRate_);
        loop_rate.sleep();
    }

    return 0;
}

