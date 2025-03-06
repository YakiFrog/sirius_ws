# #!/bin/bash
# trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
# cd ~/sirius_ws
# export ROS_DOMAIN_ID=57
# while : ;do
#     read -p "Press [Enter] key to start kobuki node..."
#     source install/setup.bash
#     ros2 launch kobuki_node kobuki_node-launch.py
# done

#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/roboteq_ws
export ROS_DOMAIN_ID=57
while : ;do
    read -p "Press [Enter] key to start roboteq node..."
    source install/setup.bash
    ros2 launch roboteq_ros2_driver roboteq_ros2_driver.launch.py 
done