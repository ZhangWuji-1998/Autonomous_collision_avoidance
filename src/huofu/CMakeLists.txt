cmake_minimum_required(VERSION 3.0.2)
project(huofu)



## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   sensor_msgs
#   cv_bridge
#   image_transport
#   pcl_conversions
#   pcl_ros
# )

find_package(catkin REQUIRED COMPONENTS 
roscpp 
rospy
std_msgs 
pcl_conversions 
pcl_ros 
cv_bridge
image_transport 
sensor_msgs
geometry_msgs
tf
laser_geometry 
tf_conversions
 )

 find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cloud_kmeans
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(huofu src/huofu.cpp)
# add_executable(test src/test.cpp)


target_link_libraries(huofu
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
# target_link_libraries(test
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
# )
