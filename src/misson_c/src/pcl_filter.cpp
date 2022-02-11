#include<pcl_filter.h>
// #include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"mission");
    ROS_INFO("Started PCL writer Node");
    ros::NodeHandle nh("~");

    Init_paramenter(nh);

    sensor_bridge sensor_bridge(nh);
    // Sensor_Bridge.sensor_bridge(nh)
     
    //  ros::Subscriber bat_sub = nh.subscribe("/camera/depth_registered/points",10,pcl_filter);
     
    //  while(ros::ok()){

    //  }
    // ROS_INFO("%d",num);
    ros::spin();    
     

    return 0;
 }