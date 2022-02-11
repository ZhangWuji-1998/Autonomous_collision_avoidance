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
#include <image_transport/image_transport.h>
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
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
using namespace cv;
using namespace std;
using namespace Eigen;

//霍夫圆函数的参数默认值
double dp = 1;
double min_dist = 60;
double param1 = 150;
double param2 = 60;
int min_radius = 2;
int max_radius = 200;
//D455相机内参
double fx_ = 381.3028564453125;
double fy_ = 381.02349853515625;
double cx_ = 316.6654052734375;
double cy_ = 243.59825134277344;
//圆心由像素坐标转换到相机坐标系下的x, y
double camera_cx;
double camera_cy;
//储存一帧内像素对应的深度
Mat depthdata;
double frame_depth;
//储存连续5帧稳定圆心状态的数据
vector<Vec3d> con_5frame;
//txt深度保存
vector<float> save_depth_txt;
//定义保存到txt文件
ofstream fout("/home/wuxin/px/avoid_ws_ljx/src/yzh.txt");
//无人机位姿及主控发布状态的话题名称定义
std::string  image_topic = "/camera/rgb/image_raw";
std::string  depth_topic = "/camera/depth/image_raw";
std::string  pose_topic = "/mavros/local_position/pose";
std::string  status_topic = "/test02/status";
std::string  point_topic = "/scan";

//无人机位姿和主控状态变量定义
geometry_msgs::Pose current_pose; 
int status_data;       //左右调整
geometry_msgs::PoseStamped circle_pose;
geometry_msgs::Pose last_pose;
int flag_41=0;
int flag_42=0;
int flag_43=0;
int flag_44=0;
int flag_45=0;
int flag_go=0;
//相机外参 参数
Matrix4d exMatrix_;
//储存在世界坐标系下连续5帧xyz均值的变量
Vec3d world_xyz;
//
ros::Publisher local_pos_pub;
std_msgs::UInt32 road_status;
bool status = ros::ok();
//现场目测相对位置调整参数
double distance_x1, distance_y1, distance_x2, distance_y2, distance_x3, distance_y3;
int flag = 0;    //画图flag第20， 40， 60， 80帧的标志位

//1s内订阅图像帧数
int spin_rate = 3;

//laser
double depth;

//tf 坐标变幻
tf::TransformListener* listener_ptr;
tf::TransformListener *tfListener_;
std::string map_frame = "/local_origin";
std::string camera_frame = "/camera_link";

Vec3d ring_min_depth_to_camera(Vec3f cc);
Vec3f max_r_circle(vector<Vec3f> pcircles);
void huofu_detect(Mat img, Mat img_filter);
void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
void depthCallback(const sensor_msgs::Image::ConstPtr &msg);
void pose_callback(const nav_msgs::OdometryConstPtr &nav_msg);
void status_callback(const std_msgs::UInt32::ConstPtr &mode);
void point_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
Vec3d camera_to_world(const Vec3d &p_c);
void pub_pose_state();
void set_pose();
double v3distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "huofu");
	ros::NodeHandle nh("~");

    //霍夫圆参数
    nh.param("dp", dp, dp);
    nh.param("min_dist", min_dist, min_dist);
    nh.param("param1", param1, param1);
    nh.param("param2", param2, param2);
    nh.param("min_radius", min_radius, min_radius);
    nh.param("max_radius", max_radius, max_radius);

    //相机内参
    nh.param("fx_", fx_, fx_);
	nh.param("fy_", fy_, fy_);
	nh.param("cx_", cx_, cx_);
	nh.param("cy_", cy_, cy_);
    //
    nh.param("image_topic", image_topic, image_topic);
    nh.param("depth_topic", depth_topic, depth_topic);
    nh.param("pose_topic", pose_topic, pose_topic);
    nh.param("status_topic", status_topic, status_topic);
    nh.param("point_topic", point_topic, point_topic);
    //现场目测相对位置调整参数
    nh.param("distance_x1", distance_x1, distance_x1);
	nh.param("distance_y1", distance_y1, distance_y1);
	nh.param("distance_x2", distance_x2, distance_x2);
	nh.param("distance_y2", distance_y2, distance_y2);
	nh.param("distance_x3", distance_x3, distance_x3);
	nh.param("distance_y3", distance_y3, distance_y3);

    nh.param("map_frame", map_frame, map_frame);
    nh.param("camera_frame", camera_frame, camera_frame);
    //1s内订阅图像帧数
    nh.param("spin_rate", spin_rate, spin_rate);
    tfListener_ = new tf::TransformListener();
    // tf::TransformListener listener;
    // listener.waitForTransform(map_frame, camera_frame, ros::Time(), ros::Duration(3.0));
    // listener_ptr = &listener;


    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub2 = it.subscribe(image_topic, 1, imageCallback);
    image_transport::Subscriber sub3 = it.subscribe(depth_topic, 1, depthCallback);
   
    ros::Subscriber PoseSub_ = nh.subscribe<nav_msgs::Odometry>(pose_topic, 1, pose_callback);
    ros::Subscriber status_sub = nh.subscribe<std_msgs::UInt32>(status_topic, 1,  status_callback);
    //local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    local_pos_pub = nh.advertise<std_msgs::String>("/D", 1);

   
   


    ros::Rate rate(spin_rate);
    bool status = ros::ok();
    while (status)
    {
        bool status = ros::ok();
        ros::spinOnce();
        //std::cout << "回调执行" <<endl;
        if (flag == 100) {
            for (int i = 0; i <= save_depth_txt.size(); i++) {
                fout << save_depth_txt[i] << endl;
             }
            fout.close();
        }
        flag++;


        pub_pose_state();
        rate.sleep();
    }

    return 0;
}


//利用大顶堆获得半径最大圆
Vec3f max_r_circle (vector<Vec3f> pcircles) {
    //定义大顶堆的比较函数，因为是要将第i个圆的下标和它的半径用pair组合成一个组合的数据类型。
    class mycomparison {
        public:
            bool operator()(const pair<int, float> &lhs, const pair<int, float>& rhs) {
                return lhs.second < rhs.second;
            }
    };
    priority_queue<pair <int, float>, vector<pair<int, float>>, mycomparison> big_heap;    //定义大顶堆，完成对多个圆的最大半径圆的输出。
    for (size_t i = 0; i < pcircles.size(); i++) {
            Vec3f cr = pcircles[i];
            pair<int, float> p(i, cr[2]);
            big_heap.push(p);
        }
        return pcircles[big_heap.top().first];
}


//获取圆环上在相机坐标系下的最小深度，及圆心在相机坐标系下的坐标
Vec3d ring_min_depth_to_camera(Vec3f cc) {
    int rr = cc[2] + 0.05 * cc[2];
    vector<float> depth_save;
    //depth_save.push_back(depth);
    for (int i = cc[0] - rr; i <= cc[0] + rr; i++)
    {
        for (int j = cc[1] - rr; j <= cc[1] + rr; j++)
        {
            int dis = (int)(i - cc[0])^2 + (int)(j - int(cc[1])^2);
            if (dis  >= cc[2] - 0.05*cc[2] && dis  <= cc[2] + 0.05*cc[2]) 
            {
                if (!depthdata.empty()) {
                    float *data = depthdata.ptr<float>(i, j);
                    if (*data != 0) {
                        // if(*data>1 && *data<10){//不确定单位是毫米还是米
                        depth_save.push_back(*data);
                        // cout << "depth_save: " << depth_save << endl;
                        //}
                        // else{
                        //     cout << "depthdata is so big" << endl;
                        // }
                        
                    }

                }
                else
                    cout << "No depthdata" << endl;
            }              
        }
    }
    if (!depth_save.empty()) {

        save_depth_txt = depth_save;        //用于储存保存在txt文件里的深度数据。

        float min_depth = *min_element(depth_save.begin(), depth_save.end());
        cout <<  "mini_depth: " <<min_depth << endl;
        //由像素坐标系转换到相机坐标系
        //cout<<"圆心x： "<< cc[0] <<" 圆心y："<< cc[1] <<endl;
        // camera_cx = (cc[0] - cx_) * min_depth / fx_;
        // camera_cy = (cc[1] - cy_) * min_depth / fy_;
        camera_cx = (cc[0] - cx_) * min_depth / fx_;
        camera_cy = (cc[1] - cy_) * min_depth / fy_;
        //这里使用laser scan获得的距离
        return Vec3d(camera_cx, camera_cy, min_depth);
        //return Vec3d(camera_cx, camera_cy, min_depth);
    }
    cout << "No depth_save" << endl;
}

//连续5帧发送真值判断
vector<Vec3d> continue_5_frame(Vec3d &save_cex_cey_depth) {
    if (con_5frame.empty() && save_cex_cey_depth[0] != 0 && save_cex_cey_depth[1] != 0 && save_cex_cey_depth[2] != 0)
    {
        con_5frame.push_back(save_cex_cey_depth);
    }
    if (!con_5frame.empty() && con_5frame.size() < 5 && save_cex_cey_depth[0] != 0 && save_cex_cey_depth[1] != 0 && save_cex_cey_depth[2] != 0) {
        //什么意思
        float xx = abs(con_5frame[con_5frame.size() - 1][2] - save_cex_cey_depth[2]);
        if (xx < 60) con_5frame.push_back(save_cex_cey_depth);
        else if (xx > 60) con_5frame.clear();
    }
    return con_5frame;
}

//霍夫圆检测
void huofu_detect(Mat img) 
{   
    //将订阅到的图像进行滤波，转化为灰度图
    Mat mediandst;
    Mat grayImage;
	medianBlur(img, mediandst, 3);           //中值滤波
    cvtColor(mediandst, grayImage, COLOR_BGR2GRAY); //转为灰度图像
    blur(grayImage, grayImage, Size(4, 4));     //均值滤波
    vector<Vec3f> pcircles;
    HoughCircles(grayImage, pcircles, HOUGH_GRADIENT, dp, min_dist, param1, param2, min_radius, max_radius);
    Vec3d save_cex_cey_depth;   //储存一帧内圆心x, y, depth。
	
	if(pcircles.size()==0){
		cout<<"can't find circles"<<endl;
		return;
	}
	else if(pcircles.size()==1){
		//cout<<"circle_num:1"<<endl;
		Vec3f cc = pcircles[0];
		Vector2d cc_2d(cc[0], cc[1]);
        circle(img, Point(cc[0], cc[1]), cc[2], Scalar(0, 0, 255), 2, LINE_AA);
        circle(img, Point(cc[0], cc[1]), 2, Scalar(125, 25, 255), 2, LINE_AA);
        // cout<<"x: "<<cc[0] << " y: " <<cc[1] <<" r: "<<cc[2] <<endl;

        save_cex_cey_depth = ring_min_depth_to_camera(cc);
        cout << "单帧相机坐标系下圆心xyz为: " << " "
        << save_cex_cey_depth[0] << " " << save_cex_cey_depth[1] << " " << save_cex_cey_depth[2] << endl << endl;

        imshow("circle", img);
        waitKey(2);
	}
	else if(pcircles.size()>1){
        //cout<<"circle_num:"<< pcircles.size()<<endl;
        Vec3f cc_max = max_r_circle(pcircles);
        circle(img, Point(cc_max[0], cc_max[1]), cc_max[2], Scalar(0, 0, 255), 2, LINE_AA);
        // cout << img.cols << "  " << img.rows << endl;
        save_cex_cey_depth = ring_min_depth_to_camera(cc_max);
        cout << "单帧相机坐标系下圆心xyz为: " << " "
        << save_cex_cey_depth[0] << " " << save_cex_cey_depth[1] << " " << save_cex_cey_depth[2] << endl << endl;
        
        imshow("circle", img);
		waitKey(2);
	}

    con_5frame = continue_5_frame(save_cex_cey_depth);
    //cout << "1111111连续5帧大小：" << con_5frame.size() << endl << endl;
    if (con_5frame.size() == 5)
    {
        float everage_5frame_x =0, everage_5frame_y=0, everage_5frame_d=0;
        for (int i = 0; i <= con_5frame.size(); i++) {
            everage_5frame_x = con_5frame[i][0] + everage_5frame_x;
            everage_5frame_y = con_5frame[i][1] + everage_5frame_y;
            everage_5frame_d = con_5frame[i][2] + everage_5frame_d;
        }
        everage_5frame_x = everage_5frame_x / 5;
        everage_5frame_y = everage_5frame_y / 5;
        everage_5frame_d = everage_5frame_d / 5;
        cout << "连续5帧稳定的均值圆心数据为：" << everage_5frame_x << " " 
        << everage_5frame_y << " " << everage_5frame_d << endl << endl;
        con_5frame.clear();
        //将连续5帧相机坐标系下的均值数据转换到世界坐标系下。
        
        world_xyz = camera_to_world(Vec3d(everage_5frame_x, everage_5frame_y, everage_5frame_d));    
    }
}

//获取图像回调函数
void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    Mat img;
    try
    {
        img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
    }
    huofu_detect(img);
    set_pose();

}

//获取相素对应深度
void depthCallback(const sensor_msgs::Image::ConstPtr &msg)
{	
    try
    {
        depthdata = cv_bridge::toCvShare(msg, "32FC1")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
    }
}

//无人机位姿回调函数
void pose_callback(const nav_msgs::OdometryConstPtr &nav_msg)
{
	nav_msgs::Odometry current_odo = *nav_msg;
	current_pose = current_odo.pose.pose;
}

//主控发布状态回调函数
void status_callback(const std_msgs::UInt32::ConstPtr & mode)
{	
	 status_data = (*mode).data;
}


//相机系到世界系
Vec3d camera_to_world (const Vec3d& p_c)
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
	if(status_data == 41) {
        if(flag_go==0){
            //检测到圆  不在中心  进行调整
            //直接依据角度进行调整 无人机前进为x  右侧为y  世界坐标系 上z 前z  右y  左偏 x增加  右篇 x减小
            std_msgs::String msg;
            string data="KO";
            last_pose.position.x = current_pose.position.x + distance_x1;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = current_pose.position.y + distance_y1;
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = current_pose.position.z ;//机体坐标系z朝下
            data = data + to_string(last_pose.position.z);
            //circle_pose.header.frame_id = map_frame;
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go = 1;	
        }
        else if(flag_go==1 && v3distance(current_pose,last_pose)<0.2){
                Vec3d world_xyz(0, 0, 0);
                flag_go=2;
                ros::Duration(2);
            }
        else if(flag_go==2 && world_xyz[0] != 0 && world_xyz[1] != 0 && world_xyz[2] != 0 ){
            std_msgs::String msg;
            string data="OK";
            last_pose.position.x = world_xyz[0] - 0.5;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = world_xyz[1];
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = world_xyz[2];
            data = data + to_string(last_pose.position.z);
            //circle_pose.header.frame_id = map_frame;
            cout<<"publish: "<<circle_pose.pose.position.x <<","<<circle_pose.pose.position.y<<","<<circle_pose.pose.position.z<<endl;
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go = 3;
        }
    }
    else if(status_data == 42) {
        if(flag_go==3){
            //检测到圆  不在中心  进行调整
            //直接依据角度进行调整 无人机前进为x  右侧为y  世界坐标系 上z 前z  右y  左偏 x增加  右篇 x减小
            std_msgs::String msg;
            string data="KO";
            last_pose.position.x = current_pose.position.x + distance_x1;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = current_pose.position.y + distance_y1;
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = current_pose.position.z ;//机体坐标系z朝下
            data = data + to_string(last_pose.position.z);
            //circle_pose.header.frame_id = map_frame;
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go=4;	
        }
        else if(flag_go==4 && v3distance(current_pose,last_pose)<0.2){
                Vec3d world_xyz(0, 0, 0);
                flag_go=5;
                ros::Duration(2);
        }
        else if(flag_go==5 && world_xyz[0] != 0 && world_xyz[1] != 0 && world_xyz[2] != 0){
            std_msgs::String msg;
            string data="OK";
            last_pose.position.x = world_xyz[0] - 0.5;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = world_xyz[1];
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = world_xyz[2];
            data = data + to_string(last_pose.position.z);
            //circle_pose.header.frame_id = map_frame;
            cout<<"publish: "<<circle_pose.pose.position.x <<","<<circle_pose.pose.position.y<<","<<circle_pose.pose.position.z<<endl;
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go = 6;
        }
    }
    else if(status_data == 43) {
        if(flag_go==6){
            //检测到圆  不在中心  进行调整
            //直接依据角度进行调整 无人机前进为x  右侧为y  世界坐标系 上z 前z  右y  左偏 x增加  右篇 x减小
            std_msgs::String msg;
            string data="KO";
            last_pose.position.x = current_pose.position.x + distance_x1;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = current_pose.position.y + distance_y1;
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = current_pose.position.z ;//机体坐标系z朝下
            data = data + to_string(last_pose.position.z);
            //circle_pose.header.frame_id = map_frame;
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go=7;	
        }
        else if(flag_go==7 && v3distance(current_pose,last_pose)<0.2){
                Vec3d world_xyz(0, 0, 0);
                flag_go=8;
                ros::Duration(2);
            }
        else if(flag_go==8 && world_xyz[0] != 0 && world_xyz[1] != 0 && world_xyz[2] != 0){
            std_msgs::String msg;
            string data="OK";
            last_pose.position.x = world_xyz[0] - 0.5;
            data = data + to_string(last_pose.position.x)+" ";
            last_pose.position.y = world_xyz[1];
            data = data + to_string(last_pose.position.y)+" ";
            last_pose.position.z = world_xyz[2];
            data = data + to_string(last_pose.position.z);
            //circle_pose.header.frame_id = map_frame;
            cout<<"publish: "<<circle_pose.pose.position.x <<","<<circle_pose.pose.position.y<<","<<circle_pose.pose.position.z<<endl;
            msg.data = data;
            local_pos_pub.publish(msg);
            flag_go = 9;
        }
    }
    else if(status_data==44){
        //任务D结束
        bool status = 0;
    }
    else{
        return;
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