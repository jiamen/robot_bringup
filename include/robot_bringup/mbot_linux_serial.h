//
// Created by zlc on 2021/5/26.
//

#ifndef _ROBOT_BRINGUP_MBOT_LINUX_SERIAL_H_
#define _ROBOT_BRINGUP_MBOT_LINUX_SERIAL_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>           // boost::asio  IO口异步工作方式
#include <geometry_msgs/Twist.h>


extern void serialInit();
extern void writeSpeed(double RobotV, double YawRate, unsigned char ctrlFlag);
extern bool readSpeed(double& vx, double& vth, double& vh, unsigned char& ctrlFlag);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);


#endif // _ROBOT_BRINGUP_MBOT_LINUX_SERIAL_H_
