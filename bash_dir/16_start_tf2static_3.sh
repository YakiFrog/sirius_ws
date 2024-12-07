#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
while : ;do
    read -p "Press [Enter] key to start tf2 static transform publisher 3..."
    source install/setup.bash
    # base_footprint と laser を結ぶ
    ros2 run tf2_ros static_transform_publisher 0.340 0 0.207 0 0 0 base_footprint laser
done