#ifndef ARUCO_H
#define ARUCO_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cmath>
#include <custom_msgs/carplace.h>

// 全局变量
extern custom_msgs::carplace carplace;
extern ros::Publisher carplace_pub;

// 函数声明
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
// void publishCarPose(custom_msgs::carplace carplace, ros::NodeHandle nh);

#endif // ARUCO_H