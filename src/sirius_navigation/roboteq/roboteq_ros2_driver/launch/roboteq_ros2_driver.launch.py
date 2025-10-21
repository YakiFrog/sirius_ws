# MIT License
#
# Copyright (c) []
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    # ROS packages
    pkg_roboteq = get_package_share_directory('roboteq_ros2_driver')

    # Config
    roboteq_config = os.path.join(pkg_roboteq, 'config/', 'roboteq.yaml')

    # Nodes
    roboteq_ros2_driver = Node(
        package='roboteq_ros2_driver',
        executable='roboteq_ros2_driver',
        name='roboteq_ros2_driver',
        output='screen',
        respawn = True,
        parameters=[roboteq_config],
    )

    """
    velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne'),
                'launch/velodyne-all-nodes-VLP16-launch.py'
            )
        )
    )
    """
    
    # tf
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    tf2_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.86', '0', '0', '0', 'base_footprint', 'velodyne']
    )
    
    tf2_node3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.30', '0', '0.165', '0', '0', '0', 'base_footprint', 'laser']
    )

    return LaunchDescription([
        # Nodes
        #velodyne,
        tf2_node,
        tf2_node2,
        tf2_node3,
        roboteq_ros2_driver,
    ])
