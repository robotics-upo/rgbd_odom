cmake_minimum_required(VERSION 2.8.3)
project(rgbd_odom)

add_compile_options(-std=c++11) 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  tf
  cvsba
  pcl_conversions
  pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(rgbd_odom_node src/rgbd_odom_node.cpp)
target_link_libraries(rgbd_odom_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


