#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

cd /data/ && ros2 bag record /camera_image/cam_HL /camera_image/cam_HR /camera_image/cam_ML /camera_image/cam_MR /camera_image/cam_VL /camera_image/cam_VR

