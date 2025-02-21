#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cmath>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // 将ROS图像消息转换为OpenCV图像
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        //转换为灰度图
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

        // 定义Aruco码字典
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // 用于存储检测到的Aruco码角点和ID
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<int> markerIds;

        // 检测Aruco码
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);

        if (!markerIds.empty()) {
            // 绘制检测到的Aruco码
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            if (markerIds.size() >= 4) {
                // 获取Aruco码的中心点
                cv::Point2f center1(0, 0), center2(0, 0), center3(0, 0), center4(0,0);

                // 按编号计算四个中心点的坐标
                for (int i = 0; i < markerIds.size(); i++) {
                    if (markerIds[i] == 0) {
                        center1 = (markerCorners[i][0] + markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3]) / 4;
                    } else if (markerIds[i] == 1) {
                        center2 = (markerCorners[i][0] + markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3]) / 4;
                    } else if (markerIds[i] == 2) {
                        center3 = (markerCorners[i][0] + markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3]) / 4;
                    } else if (markerIds[i] == 3) {
                        center4 = (markerCorners[i][0] + markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3]) / 4;
                    }
                }

                // 计算两个中心点之间的像素距离
                double distancex1 = std::sqrt(std::pow(center1.x - center2.x, 2) + std::pow(center1.y - center2.y, 2));
                double distancex2 = std::sqrt(std::pow(center3.x - center4.x, 2) + std::pow(center3.y - center4.y, 2));
                double distancey1 = std::sqrt(std::pow(center1.x - center3.x, 2) + std::pow(center1.y - center3.y, 2));
                double distancey2 = std::sqrt(std::pow(center2.x - center4.x, 2) + std::pow(center2.y - center4.y, 2));

                //在1m距离下中心点像素距离：250.5
                //根据当前中心点像素距离计算目标距离
                double disx = 251.9 / (distancex1 + distancex2);
                double disy = 251.9 / (distancey1 + distancey2);
                double distance = (disx + disy) / 2 ;

                // 计算目标角度
                double angle = std::atan2(center2.y - center1.y, center2.x - center1.x);

                // 输出距离和角度到终端
                ROS_INFO("Distance between Aruco markers: %.2f pixels", distance);
                ROS_WARN("DISTANCE: %.4fm", distance);
                ROS_INFO("Angle: %.2f degrees", angle * 180.0 / CV_PI);
            } else {
                ROS_ERROR("Less than four Aruco markers detected.");
            }
        } else {
            ROS_ERROR("No Aruco markers detected.");
        }

        // 显示结果
        cv::imshow("Aruco Detection", frame);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle nh;

    // 图像传输
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_rect_color", 1, imageCallback);

    // 创建OpenCV窗口
    cv::namedWindow("Aruco Detection", cv::WINDOW_NORMAL);

    // 进入ROS事件循环
    ros::spin();

    // 销毁OpenCV窗口
    cv::destroyWindow("Aruco Detection");

    return 0;
}