
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
while : ;do
    read -p "Press [Enter] key to start slamtoolbox..."
    source /home/sirius24/.bashrc
    cd ~/sirius_ws
    source install/setup.bash
    ros2 launch slam_toolbox online_async_launch.py
done