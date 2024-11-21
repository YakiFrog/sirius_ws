#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener 
import tf2_ros
import tf2_py as tf2
import yaml
import math
import sys

file_path = "/home/sirius24/sirius_ws/map_waypoints/waypoints/map.yaml"

class GetPose(Node):
    def __init__(self):
        super().__init__('get_pose')
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)
        self.position = []
        self.positions_list = []
        self.timer = self.create_timer(1.0, self.get_position)

    def get_position(self):
        try:
            transform = self.tfBuffer.lookup_transform('map','base_link',rclpy.time.Time())

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            self.position = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]

            self.positions_list.append(self.position)

            data = {'points':self.positions_list}

            with open(file_path, 'w', encoding='utf-8') as f:
                yaml.dump(data, f)

            self.get_logger().info('Success!')

            sys.exit()

        except tf2.LookupException:
            self.get_logger().warn('Transform not found')

        except tf2.ExtrapolationException:
            self.get_logger().warn('Extrapolation error')
        

def main(args=None):
    rclpy.init(args=args)
    node = GetPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

