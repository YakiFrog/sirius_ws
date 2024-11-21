#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import yaml
from builtin_interfaces.msg import Duration

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_viewer')
        self.pub = self.create_publisher(Marker, 'waypoint_markers', 10)
        self.load_waypoints()
        self.timer = self.create_timer(1.0, self.timer_callback)

    def load_waypoints(self):
        try:
            with open('/home/sirius/sirius_ws/src/sirius_navigation/config/map.yaml', 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                self.waypoints = data.get('points', [])
            print("Waypoints loaded")
        except FileNotFoundError:
            self.get_logger().error("File not found")
            self.waypoints = []
        except yaml.YAMLError:
            self.get_logger().error("YAML error")
            self.waypoints = []

    def timer_callback(self):
        # インタラクティブマーカーノードをクリア
        # self.int_marker_server.clear()
        for counter, waypoint in enumerate(self.waypoints):
            marker_data = Marker()
            marker_data.header.frame_id = "map"
            marker_data.header.stamp = self.get_clock().now().to_msg()
            marker_data.ns = "basic_shapes"
            marker_data.id = counter
            marker_data.action = Marker.ADD
            marker_data.pose.position.x = waypoint[0]  # x座標
            marker_data.pose.position.y = waypoint[1]  # y座標
            marker_data.pose.position.z = waypoint[2]  # z座標
            marker_data.pose.orientation.x = waypoint[3]
            marker_data.pose.orientation.y = waypoint[4]
            marker_data.pose.orientation.z = waypoint[5]
            marker_data.pose.orientation.w = waypoint[6]
            marker_data.color.r = 1.0
            marker_data.color.g = 0.0
            marker_data.color.b = 0.0
            marker_data.color.a = 1.0
            marker_data.scale.x = 1.0
            marker_data.scale.y = 0.3
            marker_data.scale.z = 0.3

            duration = Duration()
            duration.sec = 0
            duration.nanosec = 0
            marker_data.lifetime = duration
            # marker_data.lifetime = rclpy.duration.Duration(seconds=1.0)
            marker_data.type = Marker.ARROW

            self.pub.publish(marker_data)

            # Create background marker
            bg_marker = Marker()
            bg_marker.header.frame_id = "map"
            bg_marker.header.stamp = self.get_clock().now().to_msg()
            bg_marker.ns = "basic_shapes"
            bg_marker.id = counter + 2 * len(self.waypoints)  # Ensure unique ID
            bg_marker.action = Marker.ADD
            bg_marker.pose.position.x = waypoint[0]
            bg_marker.pose.position.y = waypoint[1]
            bg_marker.pose.position.z = waypoint[2] + 0.5  # Slightly above the arrow
            bg_marker.pose.orientation.x = 0.0
            bg_marker.pose.orientation.y = 0.0
            bg_marker.pose.orientation.z = 0.0
            bg_marker.pose.orientation.w = 1.0
            bg_marker.color.r = 1.0  # White color
            bg_marker.color.g = 1.0
            bg_marker.color.b = 1.0
            bg_marker.color.a = 0.5
            bg_marker.scale.x = 1.0  # Background width
            bg_marker.scale.y = 1.0  # Background height
            bg_marker.scale.z = 0.01  # Background thickness
            bg_marker.type = Marker.CYLINDER

            self.pub.publish(bg_marker)

            # text marker
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "basic_shapes"
            text_marker.id = counter + len(self.waypoints)  # Ensure unique ID
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = waypoint[0]
            text_marker.pose.position.y = waypoint[1]
            text_marker.pose.position.z = waypoint[2] + 0.5  # Slightly above the arrow
            text_marker.pose.orientation.x = 0.0
            text_marker.pose.orientation.y = 0.0
            text_marker.pose.orientation.z = 0.0
            text_marker.pose.orientation.w = 1.0
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.scale.z = 0.75  # Text height
            text_marker.text = str(counter)
            text_marker.type = Marker.TEXT_VIEW_FACING

            self.pub.publish(text_marker)

def main(args=None):
    rclpy.init(args=args)
    waypoint_manager = WaypointManager()
    rclpy.spin(waypoint_manager)
    waypoint_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("Waypoint_Viewer made by Kotani: Started!! v2")
    main()
