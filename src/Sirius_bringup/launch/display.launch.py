import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    packages_name = "Sirius_description"
    xacro_file_name = "Sirius.xacro"
    rviz_file_name = "urdf.rviz"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(packages_name), "urdf", xacro_file_name]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    # use_sim_time
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(packages_name), "rviz", rviz_file_name]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    joint_state_pub_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Gazeboのノードを追加
    gazebo_launch_file = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={"world": PathJoinSubstitution([FindPackageShare(packages_name), "worlds", "your_world.world"])}.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "sirius"],
        output="screen",
    )

    nodes = [
        rviz_node,
        robot_state_pub_node,
        joint_state_pub_gui_node,
        # gazebo,
        # spawn_entity,
    ]

    return LaunchDescription(nodes)