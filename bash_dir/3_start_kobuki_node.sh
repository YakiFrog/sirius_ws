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
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
while : ; do
    read -p "Press [Enter] key to start roboteq node..."
    # Ask whether to publish TF. Default: enable (true) when user just presses Enter.
    read -p "TFを有効にしますか？ (Y/n) デフォルト: Y -> " tf_choice
    if [ -z "${tf_choice}" ] || [[ "${tf_choice}" =~ ^[Yy] ]]; then
        pub_tf=true
    else
        pub_tf=false
    fi
    source install/setup.bash
    ros2 launch roboteq_ros2_driver roboteq_ros2_driver.launch.py pub_odom_tf:=${pub_tf}
done