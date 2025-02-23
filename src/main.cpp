#include "../include/aruco.h"

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle nh;

    // 图像传输
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_rect_color", 1, imageCallback);

    // 初始化发布器
    carplace_pub = nh.advertise<custom_msgs::carplace>("car_place", 1);

    // 进入ROS事件循环
    ros::spin();

    return 0;
}