#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
 
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc_c.h" 
#include <opencv2/highgui/highgui_c.h>
 
#include <Eigen/Core>
#include <Eigen/Geometry>



#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>

#include "std_msgs/Int8.h"  
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <vector>

#ifndef _PCL_FILTER_H_
#define _ PCL_FILTER_H_

#include <stdio.h>
#include<stdlib.h>
#include<string.h>
#include "std_msgs/Float32MultiArray.h"

#define pi 3.1415926

using namespace std;
using namespace cv;
using namespace Eigen;


class sensor_bridge{
    public:
        sensor_bridge(ros::NodeHandle &nh);
        void image_rgb(const sensor_msgs::ImageConstPtr& input);
        void point_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
        void motion_call(const std_msgs::Int32::ConstPtr& motion_);
        ros::Subscriber rgb_sub;
        ros::Subscriber motion_sub;
        ros::Subscriber PointSub_;
    private:

};

struct color_mean
{
    int r;
    int g;
    int b;
};

struct Past_State{
    int flag;
    int space;
};
extern int mission_c_pixel;
extern Past_State Past;

extern Mat dst;
extern int point_call;

extern std_msgs::Int32 motion_state;
extern double depth_circle;
extern double y_side_min;
extern double y_side_max;
extern double z_side_min;
extern double z_side_max;
extern double x_side_min;
extern double x_side_max;
extern double radius_filter;
void Init_paramenter(ros::NodeHandle &nh);

void on_medianFilter(int, void*);

// void image_rgb(const sensor_msgs::ImageConstPtr& input);

// void point_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud);



extern ros::Publisher move_flag;
// std_msgs::Float32MultiArray msg_move_flag;
extern ros::Publisher motion;





extern string image_topic;
extern string point_topic;

extern double depth;



extern double fx_;
extern double fy_;
extern double cx_;
extern double cy_;


extern double y_side_min,y_side_max,x_side_min,x_side_max;
extern double y_up_min,y_up_max,x_up_min,x_up_max;

// Mat srcImage;

// string image_topic;

extern vector<Vec3f> pcircles;










#endif