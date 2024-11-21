import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    point_pkg = get_package_share_directory('sirius_navigation')
    point_path = os.path.join(point_pkg, 'config', 'example_point.yaml')
    point_file_arg = DeclareLaunchArgument(
        'point_file', default_value=point_path
    )

    #map_dir = get_package_share_directory('turtlebot3_navigation2')
    #map_dir = '/home/nwrlab/map/15hall_2.yaml'
    
    #map_path = os.path.join(map_dir)

    return LaunchDescription([
        Node(
            package = 'sirius_navigation',
            namespace = 'sirius',
            executable = 'move_goal',
            name = 'sirius_navigation',
            parameters = [{'goal_tolerance_':2.5}]
        ),
    ])
