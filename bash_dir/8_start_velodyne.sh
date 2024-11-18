
while : ;do
    read -p "Press [Enter] key to start velodyne..."
    source /home/sirius24/.bashrc
    cd ~/sirius_ws
    source install/setup.bash
    ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py
done