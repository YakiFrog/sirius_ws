#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
while : ;do
    read -p "Press [Enter] key to start tf2 static transform publisher 1..."
    source install/setup.bash
    # ros2 run tf2_ros static_transform_publisher 0.3 0 0.165 0 0 0 base_footprint laser
    ros2 run tf2_ros static_transform_publisher 0.3 0 0.365 0 0 0 base_footprint camera_link
done
