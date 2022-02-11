#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>




int main(int argc, char *argv[])  
{
    ros::init (argc, argv, "test");
    cv::Vec3d xyz;
    std::cout << xyz[0] << xyz[1] << xyz[2] << std::endl;
    ROS_INFO("hello world");
    return 0;
}
