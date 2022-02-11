#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <queue>     //队列头文件
#include <utility>     //pair 函数头文件
#include <stack>      //栈头文件
#include <fstream> //保存数据到txt文件需要的头文件
#include <iostream>  
#include <string.h>

#include <nav_msgs/Odometry.h>   //无人机位姿
#include <std_msgs/UInt32.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
//laser
//#include <sensor_msgs/LaserScan.h>
//#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>

using namespace cv;
using namespace std;
using namespace Eigen;

//定义保存到txt文件
ofstream fout("/home/wuxin/px/avoid_ws_ljx/src/yzh.txt");
//无人机位姿及主控发布状态的话题名称定义
std::string  pose_topic = "/mavros/local_position/pose";
std::string  status_topic = "/circle_related/start_calculate";
std::string  point_topic = "/scan";

//无人机位姿和主控状态变量定义
geometry_msgs::Pose current_pose; 
int status_data;       //左右调整
geometry_msgs::Pose last_pose;
int flag_go=0;
//相机外参 参数
Matrix4d exMatrix_;
//储存在世界坐标系下连续5帧xyz均值的变量
Vec3d world_xyz;
//
ros::Publisher local_pos_pub;
bool status = ros::ok();
//现场目测相对位置调整参数
double distance_x0, distance_y0,distance_x1, distance_y1, distance_x2, distance_y2, distance_x3, distance_y3;
double dis_go0, dis_go1,dis_go2, dis_go3;
int flag = 0;    //画图flag第20， 40， 60， 80帧的标志位
//点云切割
double y_side_min,y_side_max,z_side_min,z_side_max,x_side_min,x_side_max,radius_filter;
//1s内订阅图像帧数
int spin_rate = 3;

//laser
double depth;

//tf 坐标变幻
tf::TransformListener* listener_ptr;
tf::TransformListener *tfListener_;
std::string map_frame = "/local_origin";
std::string camera_frame = "/camera_link";

void pose_callback(const nav_msgs::OdometryConstPtr &nav_msg);
void status_callback(const std_msgs::String::ConstPtr & mode);
//void point_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
void point_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
Vec3d camera_to_world (const Eigen::Vector4f& p_c);
void pub_pose_state();
void set_pose();
double v3distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "huofu");
	ros::NodeHandle nh("~");

    //
    nh.param("pose_topic", pose_topic, pose_topic);
    nh.param("status_topic", status_topic, status_topic);
    nh.param("point_topic", point_topic, point_topic);
    //现场目测相对位置调整参数
    nh.param("distance_x0", distance_x0, distance_x0);
	nh.param("distance_y0", distance_y0, distance_y0);
    nh.param("distance_x1", distance_x1, distance_x1);
	nh.param("distance_y1", distance_y1, distance_y1);
	nh.param("distance_x2", distance_x2, distance_x2);
	nh.param("distance_y2", distance_y2, distance_y2);
	nh.param("distance_x3", distance_x3, distance_x3);
	nh.param("distance_y3", distance_y3, distance_y3);
    //调整目标点到框中心的距离
    nh.param("dis_go0", dis_go0, dis_go0);
    nh.param("dis_go1", dis_go1, dis_go1);
    nh.param("dis_go2", dis_go2, dis_go2);
    nh.param("dis_go3", dis_go3, dis_go3);
    
    //点云
    nh.param("y_side_min", y_side_min, y_side_min);
	nh.param("y_side_max", y_side_max, y_side_max);
	nh.param("z_side_max", z_side_max, z_side_max);
	nh.param("z_side_min", z_side_min, z_side_min);
    nh.param("x_side_max", x_side_max, x_side_max);
	nh.param("x_side_min", x_side_min, x_side_min);
    nh.param("radius_filter", radius_filter, radius_filter);
    //tf
    nh.param("map_frame", map_frame, map_frame);
    nh.param("camera_frame", camera_frame, camera_frame);
    //1s内订阅图像帧数
    nh.param("spin_rate", spin_rate, spin_rate);
    tfListener_ = new tf::TransformListener();
 

    ros::Subscriber PointSub_ = nh.subscribe<sensor_msgs::PointCloud2>(point_topic, 1,point_callback);
    ros::Subscriber PoseSub_ = nh.subscribe<nav_msgs::Odometry>(pose_topic, 1, pose_callback);
    ros::Subscriber status_sub = nh.subscribe<std_msgs::String>(status_topic, 1,  status_callback);
    //local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    local_pos_pub = nh.advertise<std_msgs::String>("/circle_related/adjustment_message", 1);

   
   


    ros::Rate rate(spin_rate);
    bool status = ros::ok();
    while (status)
    {
        bool status = ros::ok();
        //if(status_data>=40){
            ros::spinOnce();
            pub_pose_state();
            rate.sleep();
        //}
        
    }

    return 0;
}



//无人机位姿回调函数
void pose_callback(const nav_msgs::OdometryConstPtr &nav_msg)
{
	nav_msgs::Odometry current_odo = *nav_msg;
	current_pose = current_odo.pose.pose;
}

//主控发布状态回调函数
void status_callback(const std_msgs::String::ConstPtr & mode)
{	
	string status_mode = mode->data.c_str();
    status_data = stoi(status_mode);
}


//相机系到世界系
Vec3d camera_to_world (const Eigen::Vector4f& p_c)
{	geometry_msgs::PointStamped camera;
    geometry_msgs::PointStamped world;
    cout<<"p_c: " <<p_c[0] << ","<<p_c[1] << ","<<p_c[2] <<endl;
    camera.header.frame_id = camera_frame;
    camera.point.x = p_c[0];
    camera.point.y = p_c[1];
    camera.point.z = p_c[2];
    if (!tfListener_->waitForTransform(map_frame, camera_frame, ros::Time(0), ros::Duration(3)))
			{
				ROS_ERROR("Could not get transform from %s to %s after 1 second!", map_frame.c_str(), camera_frame.c_str());
	}
    tfListener_->transformPoint(map_frame,camera,world);
    cout<<"world"<<world.point.x << ","<<world.point.y << ","<<world.point.z << endl;
    if(isinf(world.point.x)||isinf(world.point.y)||isinf(world.point.z)){
        return Vec3d(0,0,0);
    }
    return Vec3d(world.point.x,world.point.y,world.point.z);
    
}





//发布
void pub_pose_state() {
    //无人机刚通过c点 调整角度
    //cout<<"status_data: "<<status_data<<endl;
    if(status_data == 40) {
        if(flag_go==0){
            //检测到圆  不在中心  进行调整
            //直接依据角度进行调整 无人机前进为x  右侧为y  世界坐标系 上z 前z  右y  左偏 x增加  右篇 x减小
            std_msgs::String msg;
            string data="KO ";
            last_pose.position.x = current_pose.position.x + distance_x0;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = current_pose.position.y + distance_y0;
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = current_pose.position.z ;//机体坐标系z朝下
            data = data + to_string(last_pose.position.z)+" ";
            //circle_pose.header.frame_id = map_frame;
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go = 1;	
        }
        else if(flag_go==1 && v3distance(current_pose,last_pose)<0.2){
                Vec3d world_xyz(0, 0, 0);
                flag_go=2;
                ros::Duration(1);
            }
        else if(flag_go==2 && world_xyz[0] != 0 && world_xyz[1] != 0 && world_xyz[2] != 0 ){
            std_msgs::String msg;
            string data="OK ";
            last_pose.position.x = world_xyz[0] + dis_go0;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = world_xyz[1];
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = world_xyz[2];
            data = data + to_string(last_pose.position.z)+" ";
            //circle_pose.header.frame_id = map_frame;
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go = 3;
        }
    }
	else if(status_data == 41) {
        if(flag_go==3){
            //检测到圆  不在中心  进行调整
            //直接依据角度进行调整 无人机前进为x  右侧为y  世界坐标系 上z 前z  右y  左偏 x增加  右篇 x减小
            std_msgs::String msg;
            string data="KO ";
            last_pose.position.x = current_pose.position.x + distance_x1;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = current_pose.position.y + distance_y1;
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = current_pose.position.z ;//机体坐标系z朝下
            data = data + to_string(last_pose.position.z)+" ";
            //circle_pose.header.frame_id = map_frame;
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go = 4;	
        }
        else if(flag_go==4 && v3distance(current_pose,last_pose)<0.2){
                Vec3d world_xyz(0, 0, 0);
                flag_go=5;
                ros::Duration(1);
            }
        else if(flag_go==5 && world_xyz[0] != 0 && world_xyz[1] != 0 && world_xyz[2] != 0 ){
            std_msgs::String msg;
            string data="OK ";
            last_pose.position.x = world_xyz[0] + dis_go1;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = world_xyz[1];
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = world_xyz[2];
            data = data + to_string(last_pose.position.z)+" ";
            //circle_pose.header.frame_id = map_frame;
             msg.data = data;
            local_pos_pub.publish(msg);
            flag_go = 6;
        }
    }
    else if(status_data == 42) {
        if(flag_go==6){
            //检测到圆  不在中心  进行调整
            //直接依据角度进行调整 无人机前进为x  右侧为y  世界坐标系 上z 前z  右y  左偏 x增加  右篇 x减小
            std_msgs::String msg;
            string data="KO ";
            last_pose.position.x = current_pose.position.x + distance_x2;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = current_pose.position.y + distance_y2;
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = current_pose.position.z ;//机体坐标系z朝下
            data = data + to_string(last_pose.position.z)+" ";
            //circle_pose.header.frame_id = map_frame;
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go=7;	
        }
        else if(flag_go==7 && v3distance(current_pose,last_pose)<0.2){
                Vec3d world_xyz(0, 0, 0);
                flag_go=8;
                ros::Duration(1);
        }
        else if(flag_go==8 && world_xyz[0] != 0 && world_xyz[1] != 0 && world_xyz[2] != 0){
            std_msgs::String msg;
            string data="OK ";
            last_pose.position.x = world_xyz[0] + dis_go2;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = world_xyz[1];
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = world_xyz[2];
            data = data + to_string(last_pose.position.z)+" ";
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go = 9;
        }
    }
    else if(status_data == 43) {
        if(flag_go==9){
            //检测到圆  不在中心  进行调整
            //直接依据角度进行调整 无人机前进为x  右侧为y  世界坐标系 上z 前z  右y  左偏 x增加  右篇 x减小
            std_msgs::String msg;
            string data="KO ";
            last_pose.position.x = current_pose.position.x + distance_x3;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = current_pose.position.y + distance_y3;
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = current_pose.position.z ;//机体坐标系z朝下
            data = data + to_string(last_pose.position.z)+" ";
            //circle_pose.header.frame_id = map_frame;
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go=10;	
        }
        else if(flag_go==10 && v3distance(current_pose,last_pose)<0.2){
                Vec3d world_xyz(0, 0, 0);
                flag_go=11;
                ros::Duration(1);
            }
        else if(flag_go==11 && world_xyz[0] != 0 && world_xyz[1] != 0 && world_xyz[2] != 0){
            std_msgs::String msg;
            string data="OK ";
            last_pose.position.x = world_xyz[0] + dis_go3;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = world_xyz[1];
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = world_xyz[2];
            data = data + to_string(last_pose.position.z)+" ";
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go = 12;
        }
    }
    else if(status_data==44){
        //任务D结束
        bool status = 0;
    }
    else{
        
    }
}



void set_pose(){
    tf::StampedTransform transform;
    tf::TransformListener listener;
    if (!tfListener_->waitForTransform(map_frame, camera_frame, ros::Time(0), ros::Duration(3)))
			{
				ROS_ERROR("Could not get transform from %s to %s after 1 second!", map_frame.c_str(), camera_frame.c_str());
				return;
	}
	tfListener_->lookupTransform(map_frame, camera_frame, ros::Time(0), transform);   
}

double v3distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2){
    return sqrt(pow((p1.position.x-p2.position.x),2)+pow((p1.position.y-p2.position.y),2)+pow((p1.position.z-p2.position.z),2));
}

void point_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{	if(cloud->data.size()>50){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud, *pcl_pc_in);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sideway (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr side_filted (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
        //切割x y和z
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, y_side_min)));  
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, y_side_max)));  
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, z_side_min)));  
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, z_side_max)));  
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, x_side_min)));  
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, x_side_max)));  

        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition (range_cond);               
        condrem.setInputCloud (pcl_pc_in);                   //输入点云
        condrem.setKeepOrganized(false);               //设置保持点云的结构
        condrem.filter (*sideway);
        if(sideway->points.size()>10){
             pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;  //创建滤波器对象
            pcFilter.setInputCloud(sideway);             //设置待滤波的点云
            pcFilter.setRadiusSearch(radius_filter);               // 设置搜索半径
            pcFilter.setMinNeighborsInRadius(10);      // 设置一个内点最少的邻居数目
            pcFilter.filter(*side_filted);        //滤波结果存储到cloud_filtered
            //前面是z 左右是y 上下是x
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*side_filted, centroid);
            world_xyz = camera_to_world(centroid);
        }
    }
	
}

