#include <pcl_filter.h>

using namespace std;
using namespace cv;
using namespace Eigen;

double depth_circle = 0.0;
// Eigen::Matrix3d inMatrix_;
double y_side_min,y_side_max,z_side_min,z_side_max,x_side_min,x_side_max,radius_filter;

void sensor_bridge::point_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud)//
{
	//if(cloud->data.size()>50){
    if(point_call == 3){
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
       // if(sideway->points.size()>10){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;  //创建滤波器对象
        pcFilter.setInputCloud(sideway);             //设置待滤波的点云
        pcFilter.setRadiusSearch(1.0);               // 设置搜索半径
        pcFilter.setMinNeighborsInRadius(10);      // 设置一个内点最少的邻居数目
        pcFilter.filter(*side_filted);        //滤波结果存储到cloud_filtered
        //前面是z 左右是y 上下是x
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*side_filted, centroid);
        depth_circle = centroid[2];
        cout<< depth_circle << endl;
       // }
    // cv::namedWindow("img");
	// std_msgs::Int32 motion_state;

	// static int motion_flag = 0;
    // if(cv::waitKey(2) == 114 || cv::waitKey(2) == 82){
	// 	motion_flag = 1;
    // }
    // else if(cv::waitKey(2) == 115 || cv::waitKey(2) == 83){
    //     motion_flag = 0;
    // }
	// motion_state.data = motion_flag;
	// motion.publish(motion_state);
	//}
    
    
    }
        
}

// void setInternalParams()
// {	
// 	inMatrix_ << fx_, 0, cx_,
// 				0, fy_, cy_,
// 				0, 0, 1;
// }

// void setExternalParams()

// {	tf::StampedTransform transform;
// 	Quaterniond q(current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z);
// 	Vector3d t_(current_pose.position.x,current_pose.position.y,current_pose.position.z);
// 	q.normalize();
//     Matrix3d R_(q.toRotationMatrix());
// }


Vec3d pixel2camera (const Vector2d& p_p)
{
    return Vec3d (
               ( p_p(0,0)-cx_) *depth/fx_,
               ( p_p(1,0)-cy_) *depth/fy_,
               depth
           );
}


Vector3d camera2body ( const Vector3d& p_c)
{	
	Eigen::AngleAxisd angle_axisx(-pi / 2, Eigen::Vector3d(1, 0, 0));//1系绕x轴顺时针旋转90
	Eigen::Vector3d rotated_v1 = angle_axisx.matrix().inverse()*p_c;
	Eigen::AngleAxisd angle_axisz(pi / 2, Eigen::Vector3d(0, 0, 1));//2系绕z轴逆时针旋转90
	Eigen::Vector3d rotated_v2 = angle_axisz.matrix().inverse()*rotated_v1;
    return rotated_v2;
}

Eigen::Vector3d cv2eigen(const Vec3d& p_cv){
	return Vector3d(p_cv[0],p_cv[1],p_cv[2]);
}

double v3distance(const Vec3d& p1, const Vec3d& p2){
	return sqrt(pow((p1[0]-p2[0]),2)+pow((p1[1]-p2[1]),2)+pow((p1[2]-p2[2]),2));
}
 


double pointsDist(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
{
		return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}




// int main(int argc, char** argv){
//      ros::init(argc,argv,"pcl_filter");
//      ROS_INFO("Started PCL writer Node");
//      ros::NodeHandle nh;
//      ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw",10,image_rgb);
     
//     //  ros::Subscriber bat_sub = nh.subscribe("/camera/depth_registered/points",10,pcl_filter);
     
//     //  while(ros::ok()){

//     //  }
//      ROS_INFO("%d",num);
//      ros::spin();    
     

//      return 0;
//  }

