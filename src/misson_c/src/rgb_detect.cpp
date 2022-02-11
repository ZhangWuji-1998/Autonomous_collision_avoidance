#include <pcl_filter.h>

ros::Publisher pub;
using namespace std;
using namespace cv;
using namespace Eigen;


Mat dst;


Mat mediandst;

cv::Mat frame;
Mat grayImage;
const int median_value = 5;

const double dp = 1;
const int min_dist = 60;

const double param1 = 150;
const double param2 = 60;
int min_radius = 5;
int max_radius = 50;

// string image_topic;



string image_topic;

string motion_topic;
double depth;
Past_State Past;
int point_call = 0;


double fx_;
double fy_;
double cx_;
double cy_;


std_msgs::Int32 motion_state;




vector<Vec3f> pcircles;


ros::Publisher move_flag;
ros::Publisher motion;

void  Init_paramenter(ros::NodeHandle &nh){



    // setInternalParams();
    nh.param("image_topic", image_topic, image_topic);
    nh.param("motion_topic", motion_topic, motion_topic);

    nh.param("min_radius", min_radius, min_radius);
    nh.param("max_radius", max_radius, max_radius);

    nh.param("fx_", fx_, fx_);
    nh.param("fy_", fx_, fx_);
    nh.param("cx_", cx_, cx_);
    nh.param("cy_", cy_, cy_);
    
    nh.param("z_side_min", z_side_min, z_side_min);
    nh.param("z_side_max", z_side_max, z_side_max);
    nh.param("y_side_min", y_side_min, y_side_min);
    nh.param("y_side_max", y_side_max, y_side_max);
    nh.param("x_side_min", x_side_min, x_side_min);
    nh.param("x_side_max", x_side_max, x_side_max);

    
    move_flag =  nh.advertise<std_msgs::String>("move_flag", 1000);
    // motion = nh.advertise<std_msgs::Int32>("motion",1000);

    Past.flag = 0;
    Past.space = 0;

    // ros::Subscriber rgb_sub = nh.subscribe(image_topic,10,image_rgb);

    // ros::Subscriber PointSub_ = nh.subscribe<sensor_msgs::PointCloud2>(point_topic, 10, boost::bind(&point_callback, _1));

}





 


sensor_bridge::sensor_bridge(ros::NodeHandle &nh){
    PointSub_ = nh.subscribe("/camera/depth/points", 10, &sensor_bridge::point_callback,this);
    rgb_sub = nh.subscribe(image_topic,10,&sensor_bridge::image_rgb,this);
    motion_sub = nh.subscribe(motion_topic,10,&sensor_bridge::motion_call,this);
    // rgb_sub = nh.subscribe(image_topic,10,boost::bind(&sensor_bridge::image_rgb,_1,&"motion"))
    

}






void sensor_bridge::motion_call(const std_msgs::Int32::ConstPtr& motion_){

    motion_state = *motion_;

    std_msgs::String msg;
    std::stringstream ss;
    Vec3f cc;
    static int flag_staright = 0;
    cout << motion_state.data << endl;

    if(motion_state.data == 3&&flag_staright == 0)
    {
        point_call = 3;
        if(pcircles.size()!=0){
            cc = pcircles[0];
            circle(dst, Point(frame.cols / 2, frame.rows / 2), 2, Scalar(255, 255, 255), 2, LINE_AA);
            if(pcircles.size()>=1&&abs(cc[0]-frame.cols / 2)>20){
                cout << "not in the horizon center" << endl;
                
                if(cc[0]<frame.cols / 2){
                    if(Past.flag == -1){
                        if(Past.space > frame.cols / 2 - cc[0]){ //前一状态下的偏移空间大于当前右移空间 直飞

                           cout << "staight forward 5m" << endl;
                            ss<<"p";
                            ss<<" ";
                            ss<<depth_circle << " ";
                            // ss << 5; 
                            flag_staright = 1;

                        }
                        else{
                            if(Past.space < frame.cols / 2 - cc[0]){ //前一状态下的偏移空间小于于当前右移空间 
                                cout << "onto right 0.1m" << endl;
                                ss<<"r";
                                ss<<" ";
                                ss <<-0.1 << " ";
                            }                   
                        }
                    }
                    else{
                        Past.flag = 1;
                        cout << "onto left 0.2m" << endl;
                        ss<<"r";
                        ss<<" ";
                        ss << 0.2 << " ";
                        // ss << "onto left 0.2m";
                        Past.space = frame.cols / 2 - cc[0];
                    }
                    msg.data = ss.str();
                    move_flag.publish(msg);
                    cout << "on the left" << endl;
                }

                if(cc[0]>frame.cols / 2){
                    if(Past.flag == 1){
                        if(Past.space > cc[0] - frame.cols / 2){ //前一状态下的偏移空间大于当前左偏空间 直飞

                            cout<<"staight forward 5m" <<endl;
                            ss<<"p";
                            ss<<" ";
                            ss<<depth_circle << " ";
                            // ss << 5;
                            flag_staright = 1;
                            // ss << "staight forward 5m"; 
                        }
                        else{
                            if(Past.space <  cc[0] - frame.cols / 2){ //前一状态下的偏移空间小于于当前左偏空间
                            cout<<"onto left 0.1m" <<endl;
                            ss<<"r";
                            ss<<" ";
                            ss << -0.1 << " ";   
                            // ss << "onto left 0.1m";
                            }                   
                        }
                    }
                    else{
                        Past.flag = -1;
                        cout<<"onto right 0.2m" <<endl;
                        ss<<"r";
                        ss<<" ";
                        ss <<- 0.2 << " ";
                        // ss << "onto right 0.2m";
                        Past.space = cc[0] - frame.cols / 2;
                    }
                    msg.data = ss.str();
                    move_flag.publish(msg);
                    
                    cout << "on the right" << endl;
                }
                
            }
            else if(pcircles.size()>=1&&abs(cc[0]-frame.cols / 2)<=20){

                 ss<<"p";
                ss<<" ";
                ss<<depth_circle << " ";
                // ss << 5;
                flag_staright = 1;
                cout << "staight forward 5m" <<endl;
                msg.data = ss.str();
                move_flag.publish(msg);
            }

        }
        else if(pcircles.size() == 0){
            cout<<"onto right 0.5m"<<endl;
            ss<<"r";
            ss<<" ";
            ss << 0.3 << " ";
            msg.data = ss.str();
            move_flag.publish(msg);
            // ss << "onto right 0.5m";

            // 
            // move_flag.publish(msg);
        }
        msg.data = ss.str();
    }
    else if(motion_state.data != 3){
        
        circle(dst, Point(frame.cols / 2, frame.rows / 2), 2, Scalar(255, 0, 0), 2, LINE_AA);

    }
    else if(flag_staright == 1){
        point_call = 1;
        // move_flag.publish(msg);
        // move_flag.publish(msg);
    }
    // flag_staright = 0;
    ss.clear();

}

void sensor_bridge::image_rgb(const sensor_msgs::ImageConstPtr& input){


    // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(input,  sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    frame = cv_ptr->image;
    
    
    Vec3f cc;



	
    medianBlur(frame, mediandst, median_value);


    cvtColor(mediandst,grayImage,CV_BGR2GRAY);
	blur(grayImage, grayImage, Size(4, 4)); 	//mohu chuli size biger more vague


    
    HoughCircles(grayImage,pcircles,HOUGH_GRADIENT, dp,min_dist,param1,param2,min_radius,max_radius);

	mediandst.copyTo(dst);

    cout<< "Hough_callback: " << frame.cols << " , " << frame.rows <<endl;


	if(pcircles.size()==0){
        
		cout<<"can't find circles"<<endl;
		// return;
	}
	else{

			cout << pcircles.size() << " circles found in the frame" << endl;

			if(pcircles.size()==1){
				// cout<<"size==1"<<endl;
				cc = pcircles[0];
				Vector2d cc_2d(cc[0], cc[1]); //yuanxin zuobiao

				circle(dst, Point(cc[0], cc[1]), cc[2], Scalar(0, 0, 255), 2, LINE_AA);		//contours 
				circle(dst, Point(cc[0], cc[1]), 2, Scalar(125, 25, 255), 2, LINE_AA);
				circle(grayImage, Point(cc[0], cc[1]), 2, Scalar(125, 25, 255), 2, LINE_AA); //center
				circle(grayImage, Point(cc[0], cc[1]), cc[2], Scalar(0, 0, 255), 2, LINE_AA);	
				cout<< "circle info: " << cc[0] << "," <<cc[1] << "," <<cc[2]<<endl;
				

			}
			else if(pcircles.size()>1){
				// Vec3f c0 = pcircles[0];


				for(int i=0;i<pcircles.size()-1;i++){
        		
            		for(int j=0;j<pcircles.size()-1-i;j++){
						
						Vec3f cc_temp1 =  pcircles[j];
						Vec3f cc_temp2 =  pcircles[j+1];
						if(cc_temp1[2]<cc_temp2[2]){
							Vec3f cc = pcircles[j];
							
							pcircles[j]=pcircles[j+1];
							pcircles[j+1]=cc;
						}
                	}
				}
				// cout<<"pcircles.size()"<<pcircles.size()<<endl;
                cout<< "circle info: " << cc[0] << "," <<cc[1] << "," <<cc[2]<<endl;
                // cout<<"pcircles.size()"<<pcircles.size()<<endl;
				for (size_t i = 0; i < pcircles.size(); i++) {
					cc = pcircles[i];
					circle(dst, Point(cc[0], cc[1]), cc[2], Scalar(0, 0, 255), 2, LINE_AA);			
					circle(dst, Point(cc[0], cc[1]), 2, Scalar(125, 25, 255), 2, LINE_AA);
				}
		


			} 

		}

    
    cv::namedWindow("img", WINDOW_NORMAL);
    cv::imshow("img",dst);
    cv::waitKey(1);

 }
