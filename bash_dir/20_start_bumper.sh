#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/roboteq_ws
export ROS_DOMAIN_ID=57
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
while : ;do
    read -p "Press [Enter] key to start bumper..."
    source install/setup.bash
    ros2 launch sirius_navigation bumper.launch.py
done