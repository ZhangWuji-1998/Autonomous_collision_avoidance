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


#include <nav_msgs/Odometry.h>   //无人机位姿
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>


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

//无人机位姿和主控状态变量定义
geometry_msgs::Pose current_pose; 
int status_data;       //左右调整
geometry_msgs::PoseStamped circle_pose;
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
std_msgs::UInt8 road_status;
bool status = ros::ok();

//现场目测相对位置调整参数
double distance_x1, distance_y1, distance_x2, distance_y2, distance_x3, distance_y3;

int flag = 0;    //画图flag第20， 40， 60， 80帧的标志位

//1s内订阅图像帧数
int spin_rate = 3;

Vec3d ring_min_depth_to_camera(Vec3f cc);
Vec3f max_r_circle(vector<Vec3f> pcircles);
void huofu_detect(Mat img, Mat img_filter);
void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
void depthCallback(const sensor_msgs::Image::ConstPtr &msg);
void pose_callback(const nav_msgs::OdometryConstPtr &nav_msg);
void status_callback(const std_msgs::UInt8::ConstPtr &mode);
Vec3d camera_to_world(const Vec3d &p_c);
void pub_pose_state();

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
    //现场目测相对位置调整参数
    nh.param("distance_x1", distance_x1, distance_x1);
	nh.param("distance_y1", distance_y1, distance_y1);
	nh.param("distance_x2", distance_x2, distance_x2);
	nh.param("distance_y2", distance_y2, distance_y2);
	nh.param("distance_x3", distance_x3, distance_x3);
	nh.param("distance_y3", distance_y3, distance_y3);

    //1s内订阅图像帧数
    nh.param("spin_rate", spin_rate, spin_rate);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub2 = it.subscribe(image_topic, 1, imageCallback);
    image_transport::Subscriber sub3 = it.subscribe(depth_topic, 1, depthCallback);
   
    ros::Subscriber PoseSub_ = nh.subscribe<nav_msgs::Odometry>(pose_topic, 1, pose_callback);
    ros::Subscriber status_sub = nh.subscribe<std_msgs::UInt8>(status_topic, 1,  status_callback);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);




    ros::Rate rate(spin_rate);
    bool status = ros::ok();
    while (status)
    {
        bool status = ros::ok();
        ros::spinOnce();
        //std::cout << "回调执行" <<endl;


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
                        if(*data>0.5 && *data<5000){//不确定单位是毫米还是米
                            cout<<"*data: "<<*data<<endl;
                            depth_save.push_back(*data);
                        }
                        else{
                            cout << "depthdata is so big" << endl;
                        }
                        
                    }

                }
                else
                    cout << "No depthdata" << endl;
            }              
        }
    }
    if (!depth_save.empty()) {

        // save_depth_txt = depth_save;        //用于储存保存在txt文件里的深度数据。

        float min_depth = *min_element(depth_save.begin(), depth_save.end());
        cout <<  "mini_depth: " <<min_depth << endl;
        //由像素坐标系转换到相机坐标系
        cout<<"圆心x： "<< cc[0] <<" 圆心y："<< cc[1] <<endl;
        camera_cx = (cc[0] - cx_) * min_depth / fx_;
        camera_cy = (cc[1] - cy_) * min_depth / fy_;
        return Vec3d(camera_cx, camera_cy, min_depth);
    }
    cout << "No depth_save" << endl;
}

//连续5帧发送真值判断
vector<Vec3d> continue_5_frame(Vec3d &save_cex_cey_depth) {
    if (con_5frame.empty() && save_cex_cey_depth[0] != 0 && save_cex_cey_depth[1] != 0 && save_cex_cey_depth[2] != 0)
    {
        con_5frame.push_back(save_cex_cey_depth);
    }
    if (!con_5frame.empty() && con_5frame.size() < 5) {
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
        //cout << "单帧相机坐标系下圆心xyz为: " << " "
        //<< save_cex_cey_depth[0] << " " << save_cex_cey_depth[1] << " " << save_cex_cey_depth[2] << endl << endl;

        imshow("circle", img);
        waitKey(2);
	}
	else if(pcircles.size()>1){
        //cout<<"circle_num:"<< pcircles.size()<<endl;
        Vec3f cc_max = max_r_circle(pcircles);
        circle(img, Point(cc_max[0], cc_max[1]), cc_max[2], Scalar(0, 0, 255), 2, LINE_AA);
        // cout << img.cols << "  " << img.rows << endl;
        save_cex_cey_depth = ring_min_depth_to_camera(cc_max);
        //cout << "单帧相机坐标系下圆心xyz为: " << " "
        //<< save_cex_cey_depth[0] << " " << save_cex_cey_depth[1] << " " << save_cex_cey_depth[2] << endl << endl;
        
        imshow("circle", img);
		waitKey(2);
	}

    con_5frame = continue_5_frame(save_cex_cey_depth);
    //cout << "1111111连续5帧大小：" << con_5frame.size() << endl << endl;
    if (con_5frame.size() == 5)
    {
        float everage_5frame_x, everage_5frame_y, everage_5frame_d;
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
void status_callback(const std_msgs::UInt8::ConstPtr & mode)
{	
	 status_data = (*mode).data;
}


//相机系到世界系
Vec3d camera_to_world (const Vec3d& p_c)
{	
    //实时获取相机外参
    Quaterniond q(current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z);
	Vector3d t_(current_pose.position.x,current_pose.position.y,current_pose.position.z);
	q.normalize();
    Matrix3d R_(q.toRotationMatrix());
	exMatrix_ << R_(0, 0), R_(0, 1), R_(0, 2), t_(0,0),
		R_(1, 0), R_(1, 1), R_(1, 2), t_(1,0),
		R_(2, 0), R_(2, 1), R_(2, 2), t_(2,0),
		0, 0, 0, 1;

    //exMatrix_储存的即为外参矩阵，以下为根据外参矩阵由相机系下转换到世界坐标系下。
	Vector4d p_c_q{ p_c[0],p_c[1],p_c[2],1 };
	Vector4d p_w_q = exMatrix_.inverse() * p_c_q;
    return Vec3d{ p_w_q(0,0),p_w_q(1,0),p_w_q(2,0) };
}





//发布
void pub_pose_state() {
    //无人机刚通过c点 调整角度
	if(status_data == 30 && flag_41 == 0) {
		flag_41++;
        cout<<"distance_x1:"<<distance_x1<<endl;
		//直接依据角度进行调整 无人机前进为x  右侧为y  世界坐标系 上z 前z  右y  左偏 x增加  右篇 x减小
		circle_pose.pose.position.x = current_pose.position.x + distance_x1;
		circle_pose.pose.position.y = current_pose.position.y + distance_y1;
		circle_pose.pose.position.z = current_pose.position.z ;//机体坐标系z朝下
		local_pos_pub.publish(circle_pose);	
		flag_go = 1;
	}
    //无人机已经在第一个圈前调整完毕 准备检测
    if (status_data == 31) {
        if(flag_go==1){
          Vec3d world_xyz(0, 0, 0);
          flag_go=2;
        }
        else if(flag_go==2 && world_xyz[0] != 0 && world_xyz[1] != 0 && world_xyz[2] != 0){
             circle_pose.pose.position.x = world_xyz[0] + 0.5;
            circle_pose.pose.position.y = world_xyz[1];
            circle_pose.pose.position.z = world_xyz[2];
            local_pos_pub.publish(circle_pose);
            flag_go = 3;
        }
    }

    //通过第一个圈后需要调整方向
    if(status_data ==41 && flag_42 == 0){
        flag_42++;
        //直接依据角度进行调整 无人机前进为x  右侧为y  世界坐标系 上z 前z  右y  左偏 x增加  右篇 x减小
        circle_pose.pose.position.x = current_pose.position.x + distance_x2;
        circle_pose.pose.position.y = current_pose.position.y + distance_y2;
        circle_pose.pose.position.z = current_pose.position.z ;//机体坐标系z朝下
        local_pos_pub.publish(circle_pose);
            
    }
    //无人机已经在第二个圈前调整完毕 准备检测
    if (status_data == 42) {
        if(flag_go==3){
          Vec3d world_xyz(0, 0, 0);
          flag_go=4;
        }
        else if(flag_go==4 && world_xyz[0] != 0 && world_xyz[1] != 0 && world_xyz[2] != 0){
             circle_pose.pose.position.x = world_xyz[0] + 0.5;
            circle_pose.pose.position.y = world_xyz[1];
            circle_pose.pose.position.z = world_xyz[2];
            local_pos_pub.publish(circle_pose);
            flag_go = 5;
        }
    }
	
    //通过第二个圈  需要调整
    if(status_data==43 && flag_43 == 0){
        flag_43++;
        //直接依据角度进行调整 无人机前进为x  右侧为y  世界坐标系 上z 前z  右y  左偏 x增加  右篇 x减小
        circle_pose.pose.position.x = current_pose.position.x + distance_x3;
        circle_pose.pose.position.y = current_pose.position.y + distance_y3;
        circle_pose.pose.position.z = current_pose.position.z ;//机体坐标系z朝下
        local_pos_pub.publish(circle_pose);	
    }
		//无人机已经在第三个圈前调整完毕 准备检测
    if (status_data== 43) {
        if(flag_go==5){
          Vec3d world_xyz(0, 0, 0);
          flag_go=6;
        }
        else if(flag_go==6 && world_xyz[0] != 0 && world_xyz[1] != 0 && world_xyz[2] != 0){
             circle_pose.pose.position.x = world_xyz[0] + 0.5;
            circle_pose.pose.position.y = world_xyz[1];
            circle_pose.pose.position.z = world_xyz[2];
            local_pos_pub.publish(circle_pose);
            flag_go = 7;
        }
    }
    if(status_data==45){
        //任务D结束
        bool status = 0;
    }
}
