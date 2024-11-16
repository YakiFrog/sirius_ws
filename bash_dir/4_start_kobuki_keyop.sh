#!/bin/sh
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
while : ;do
    read -p "Press [Enter] key to start kobuki keyop..."
    cd ~/sirius_ws
    source install/setup.bash
    ros2 run kobuki_keyop kobuki_keyop_node --ros-args -p linear_vel_max:=1.0 -p angular_vel_step:=0.05
done