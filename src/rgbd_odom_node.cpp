#include <ros/ros.h>
#include "rgbdodom.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rgbd_odom_node");  
  
	// Resolv RGBD camera name
	ros::NodeHandle lnh("~");
	std::string cameraTopic;
	if(!lnh.getParam("camera_topic", cameraTopic))
		cameraTopic = "camera";
  
	// Visual odometry instance
    std::string nodeName = ros::this_node::getName();
	RgbdOdom odom(nodeName, cameraTopic);
	
	// Spin for ever
	ros::spin();
}
