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

file_path = "/home/sirius24/sirius_ws/map_waypoints/waypoints/map.yaml"

class GetPose(Node):
    def __init__(self):
        super().__init__('get_pose')
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.positions_list = []
        self.check_position = [0,0]
        self.position = []

    def timer_callback(self):
        distance = 0
        try:
            transform = self.tfBuffer.lookup_transform('map','base_link',rclpy.time.Time())

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            self.position = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]

            x_distance = self.check_position[0] - translation.x
            y_distance = self.check_position[1] - translation.y
            distance = math.sqrt(x_distance ** 2 + y_distance ** 2)

        except tf2.LookupException:
            self.get_logger().warn('Transform not found')

        except tf2.ExtrapolationException:
            self.get_logger().warn('Extrapolation error')

        if distance > 5.0:
            self.positions_list.append(self.position)

            #取得した座標の更新
            self.check_position[0] = translation.x
            self.check_position[1] = translation.y

            self.get_logger().info('Success!')

    def finish_write(self):
        self.positions_list.append(self.position)
        data = {"points":self.positions_list}

        # YAMLファイルに書き込む
        with open(file_path, 'w', encoding='utf-8') as f:
            yaml.dump(data, f)

        self.get_logger().info('Finish')

def main(args=None):
    rclpy.init(args=args)
    node = GetPose()

    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        node.finish_write()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

