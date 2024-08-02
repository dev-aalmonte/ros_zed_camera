# ROS ZED Camera Practice
NOTE: This project was made to practice ros integration with the StereoLabs camera. It does not have an endgoal.
---
Practice to Integrate zed camera topic on ros. Also it applies some filtering and segmentation using PCL.

## Prerequisites
1. pcl
2. pcl_conversions

## Build
1. Clone the repository
> cd {ROS2 workspace path}/src

> git clone https://github.com/dev-aalmonte/ros_zed_camera.git
2. Build and source
> colcon build

> source install/setup.bash
3. Launch
> ros2 launch ros_zed_camera zed_filter.launch.py
