import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 各パッケージのディレクトリパスを取得
    sirius_navigation_pkg = get_package_share_directory('sirius_navigation')

    # 引数の定義
    move_goal_file_arg = DeclareLaunchArgument(
        'move_goal_file', # 引数名
        default_value=os.path.join(sirius_navigation_pkg, 'config', 'move_goal.yaml'),
        description='Path to the goal file for move_goal.py'
    )

    initial_pose_node = Node(
        package='sirius_navigation',
        executable='initial_pose.py',
        name='initial_pose_node',
        output='screen',
    )

    move_goal_node = Node(
        package='sirius_navigation',
        executable='move_goal.py',
        name='move_goal_node',
        output='screen',
    )

    return LaunchDescription([
        move_goal_file_arg,
        initial_pose_node,
        move_goal_node,
    ])
    
"""
ros2 launch sirius_navigation all_in_one_nav2.launch.py move_goal_file:=move_goal.yaml
ros2 launch sirius_navigation all_in_one_nav2.launch.py move_goal_file:=first.yaml
ros2 launch sirius_navigation all_in_one_nav2.launch.py move_goal_file:=second.yaml
"""