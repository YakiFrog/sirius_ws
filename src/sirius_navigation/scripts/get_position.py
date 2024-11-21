#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener 
import tf2_ros
import tf2_py as tf2
import yaml

file_path = "/home/sirius/sirius_ws/src/sirius_navigation/config/map.yaml"

class GetRobotPose(Node):
    def __init__(self):
        super().__init__('get_robot_pose')
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)
        self.timer = self.create_timer(30.0, self.timer_callback)
        self.positions_list = []
        

    def timer_callback(self):
        position = []
        try:
            transform = self.tfBuffer.lookup_transform('map','base_link',rclpy.time.Time())

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            position = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]

        except tf2.LookupException:
            self.get_logger().warn('Transform not found')

        except tf2.ExtrapolationException:
            self.get_logger().warn('Extrapolation error')

        self.positions_list.append(position)

        data = {"points":self.positions_list}
        

        # YAMLファイルに書き込む
        with open(file_path, 'w', encoding='utf-8') as f:
            #yaml.dump(data, f, default_flow_style=True, allow_unicode=True)
            yaml.dump(data, f)

        self.get_logger().info('Success!')

def main(args=None):
    rclpy.init(args=args)
    node = GetRobotPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

