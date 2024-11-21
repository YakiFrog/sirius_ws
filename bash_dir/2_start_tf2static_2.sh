#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
while : ;do
    read -p "Press [Enter] key to start tf2 static transform publisher 2..."
    source install/setup.bash
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint velodyne
done