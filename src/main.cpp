// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/aruco.hpp>
// #include <iostream>
// #include <vector>

// // 全局变量，用于存储深度图
// cv::Mat depth_image;

// // 深度图回调函数
// void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
//     try {
//         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
//         depth_image = cv_ptr->image;
//     } catch (cv_bridge::Exception& e) {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//     }
// }

// // 彩色图像回调函数
// void colorCallback(const sensor_msgs::ImageConstPtr& msg) {
//     try {
//         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//         cv::Mat color_image = cv_ptr->image;

//         // ArUco检测
//         cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
//         std::vector<int> markerIds;
//         std::vector<std::vector<cv::Point2f>> markerCorners;

//         cv::aruco::detectMarkers(color_image, dictionary, markerCorners, markerIds);

//         if (!markerIds.empty()) {
//             // 绘制ArUco码
//             cv::aruco::drawDetectedMarkers(color_image, markerCorners, markerIds);

//             // 遍历每个检测到的ArUco码
//             for (size_t i = 0; i < markerIds.size(); ++i) {
//                 std::cout << "ArUco ID: " << markerIds[i] << std::endl;

//                 // 打印边界信息
//                 std::cout << "Corners: ";
//                 for (const auto& corner : markerCorners[i]) {
//                     std::cout << "(" << corner.x << ", " << corner.y << ") ";
//                 }
//                 std::cout << std::endl;

//                 // 获取该区域的平均深度值
//                 if (!depth_image.empty()) {
//                     const auto& corners = markerCorners[i];
//                     cv::Rect bounding_box = cv::boundingRect(corners);

//                     float depth_sum = 0.0;
//                     int valid_pixel_count = 0;

//                     for (int y = bounding_box.y; y < bounding_box.y + bounding_box.height; ++y) {
//                         for (int x = bounding_box.x; x < bounding_box.x + bounding_box.width; ++x) {
//                             if (x >= 0 && x < depth_image.cols && y >= 0 && y < depth_image.rows) {
//                                 uint16_t depth = depth_image.at<uint16_t>(y, x);
//                                 if (depth > 0) { // 排除无效深度值
//                                     depth_sum += depth;
//                                     valid_pixel_count++;
//                                 }
//                             }
//                         }
//                     }

//                     if (valid_pixel_count > 0) {
//                         float avg_depth = depth_sum / valid_pixel_count * 0.001; // 转换为米
//                         std::cout << "Average Depth: " << avg_depth << " m" << std::endl;
//                     } else {
//                         std::cout << "No valid depth data in region." << std::endl;
//                     }
//                 } else {
//                     std::cout << "Depth image is empty." << std::endl;
//                 }
//             }
//         }

//         // 显示结果
//         cv::imshow("Color Image with ArUco Detection", color_image);
//         cv::waitKey(1);

//     } catch (cv_bridge::Exception& e) {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "aruco_depth_node");
//     ros::NodeHandle nh;

//     // 订阅深度图和彩色图像话题
//     ros::Subscriber depth_sub = nh.subscribe("/camera/depth/image_rect_raw", 1, depthCallback);
//     ros::Subscriber color_sub = nh.subscribe("/camera/color/image_raw", 1, colorCallback);

//     ros::spin();
//     return 0;
// }


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

            if (markerIds.size() >= 2) {
                // 获取第一个和第二个Aruco码的中心点
                cv::Point2f center1(0, 0), center2(0, 0), center3(0, 0), center4(0,0);

                for (const auto& corner : markerCorners[0]) {
                    center1 += corner;
                }
                center1 *= (1.0 / 4.0); // 平均角点得到中心点

                for (const auto& corner : markerCorners[1]) {
                    center2 += corner;
                }
                center2 *= (1.0 / 4.0); 

                for (const auto& corner : markerCorners[2]) {
                    center3 += corner;
                }
                center3 *= (1.0 / 4.0); 

                for (const auto& corner : markerCorners[3]) {
                    center4 += corner;
                }
                center4 *= (1.0 / 4.0); 

                // 计算两个中心点之间的像素距离
                double distancex1 = std::sqrt(std::pow(center1.x - center2.x, 2) + std::pow(center1.y - center2.y, 2));
                double distancex2 = std::sqrt(std::pow(center3.x - center4.x, 2) + std::pow(center3.y - center4.y, 2));
                double distancey1 = std::sqrt(std::pow(center1.x - center3.x, 2) + std::pow(center1.y - center3.y, 2));
                double distancey2 = std::sqrt(std::pow(center2.x - center4.x, 2) + std::pow(center2.y - center4.y, 2));

                //在1m距离下中心点像素距离：250.5
                //根据当前中心点像素距离计算目标距离
                double disx = 251.9 / (distancex1 + distancex2);
                double disy = 251.9 / (distancex1 + distancex2);
                double dis = (disx + disy) / 2 ;

                // 输出距离到终端
                ROS_INFO("Distance between Aruco markers: %.2f pixels", distance);
                ROS_WARN("DISTANCE:%.4fm",dis);
            } else {
                ROS_ERROR("Less than two Aruco markers detected.");
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
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);

    // 创建OpenCV窗口
    cv::namedWindow("Aruco Detection", cv::WINDOW_NORMAL);

    // 进入ROS事件循环
    ros::spin();

    // 销毁OpenCV窗口
    cv::destroyWindow("Aruco Detection");

    return 0;
}
