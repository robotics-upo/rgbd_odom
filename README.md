# rgbd_odom
ROS package for robust odometry estimation based on RGBD data

This package implements a viual odometry sysem based on RGBD cameras. The system makes use of the visual+depth RGBD images to robustly estimate the robot motion.

Computer efficiency has been considered in the source code development, it runs smoothly in a single core i7. The algorithms implements the following methods:
* Odometry based on key-framing to reduce the impact of the drift.
* Perspective n Point (PnP) for frame-to-keyframe transform estimation.
* Optional loose coupled integration of IMU roll and pitch angles.
* Window bundle adjustment for pose refinement (not map).

## General requirements
The source code has been tested in ROS indigo with Ubuntu Lunux 14.04. However, no major requirements are needed except the software packages listed in Dependencies 

## Dependencies
This package depends on non-linear optimization ROS package than can be downloaded here (https://github.com/robotics-upo/nonlinear_optimization) 

## Compilation
In order to build the package, git clone this package (and the nonlinear_optimization) into your *src* directory of your Catkin workspace and compile it by using *catkin_make* as usual.
