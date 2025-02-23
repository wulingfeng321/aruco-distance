#include "../include/aruco.h"

custom_msgs::carplace carplace;
ros::Publisher carplace_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // 将ROS图像消息转换为OpenCV图像
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // 转换为灰度图
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        //从frame复制到gray
        //gray = frame.clone();
        cv::Mat gray1;
        gray1 = gray.clone();

        // 直方图均衡化
        cv::equalizeHist(gray, gray);

        // // 双边滤波
        // cv::Mat newr;
        // cv::bilateralFilter(gray,newr,9,50,50);
        // gray=newr.clone();

        // 自适应阈值处理
        //cv::adaptiveThreshold(gray, gray, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);

        // 定义Aruco码字典
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // 用于存储检测到的Aruco码角点和ID
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<int> markerIds;

        // 检测Aruco码
        cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds);

        if (!markerIds.empty()) {
            // 绘制检测到的Aruco码
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
            cv::aruco::drawDetectedMarkers(gray, markerCorners, markerIds);

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

                //在1m距离下中心点像素距离
                //根据当前中心点像素距离计算目标距离
                double disx = 251.9 / (distancex1 + distancex2);
                double disy = 251.9 / (distancey1 + distancey2);
                double distance = (disx + disy) / 2 ;

                // 计算目标相对于相机的yaw
                double yaw = std::atan(distancey1/distancey2);

                // 存储目标位置
                carplace.distance = distance;
                carplace.yaw = yaw * 180.0 / CV_PI - 45;

                // 输出距离和角度到终端
                ROS_INFO("disx1:%.2f disx2:%.2f disy1:%.2f disy2:%.2f",distancex1,distancex2,distancey1,distancey2);
                ROS_WARN("DISTANCE: %.4fm ", carplace.distance);
                ROS_WARN("Yaw: %.2f ", carplace.yaw);
            } else {
                ROS_ERROR("Less than four Aruco markers detected.");
                //存入-1
                carplace.distance = -1;
                carplace.yaw = -1;
            }

        } 
        else {
            ROS_ERROR("No Aruco markers detected.");
            //存入-2
            carplace.distance = -2;
            carplace.yaw = -2;
        }

        // 显示结果
        // cv::imshow("yuantu", gray1);
        cv::imshow("目标检测", gray);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }            
    //频率控制
    // ros::Rate loop_rate(1);//1Hz
    // loop_rate.sleep();
    carplace_pub.publish(carplace);
}

// void publishCarPose(custom_msgs::carplace carplace, ros::NodeHandle nh){
//     ros::Publisher pub = nh.advertise<custom_msgs::carplace>("car_place", 1000);
//     pub.publish(carplace);
// }