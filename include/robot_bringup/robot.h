//
// Created by zlc on 2021/5/26.
//

#ifndef _ROBOT_BRINGUP_ROBOT_H_
#define _ROBOT_BRINGUP_ROBOT_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
// 差分轮式机器人，用Twist速度指令控制
    // linear: XYZ方向上的线速度，单位是m/s
    // angular: XYZ方向上的角速度，单位是rad/s


namespace robot
{

class robot
{
public:
    robot();
    ~robot();
    bool init();
    bool deal(double RobotV, double RobotYawRate);

private:
    void calcOdom();        // 里程计计算
    void pubOdomAndTf();    // 发布Odom和tf

private:
    ros::Time current_time_, last_time_;    // 时间

    double x_;              // 机器人位姿
    double y_;
    double th_;

    double vx_;             // 机器人x方向的速度
    double vy_;             // 机器人y方向的速度
    double vth_;            // 机器人角速度

    unsigned char sensFlag_;    // 通信预留发送和接收标志位，可进行信号控制使用
    unsigned char receFlag_;

    ros::NodeHandle nh_;
    ros::Publisher  pub_;
    tf::TransformBroadcaster odom_broadcaster_;     // 变换坐标 广播
};

}



#endif // _ROBOT_BRINGUP_ROBOT_H_
