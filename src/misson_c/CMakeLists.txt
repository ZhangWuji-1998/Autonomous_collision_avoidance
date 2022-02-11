cmake_minimum_required(VERSION 3.0.2)
project(mission_c)

add_compile_options(-std=c++11)

catkin_package(
  DEPENDS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_msgs
  cv_bridge
  pcl_conversions 
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_msgs
  cv_bridge
  pcl_conversions 
)

find_package(OpenCV 3.2 REQUIRED)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

 
add_executable(mission_c src/pcl_filter.cpp
                         src/rgb_detect.cpp
                         src/pcl_segement.cpp
                                ) 
add_dependencies(mission_c ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 

target_link_libraries(mission_c
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )


