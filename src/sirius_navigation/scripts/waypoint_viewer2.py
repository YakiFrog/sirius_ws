#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import yaml
from builtin_interfaces.msg import Duration

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_viewer')
        self.pub = self.create_publisher(Marker, 'waypoint_markers', 10)
        self.int_marker_server = InteractiveMarkerServer(self, self.get_name())
        self.load_waypoints()
        self.timer = self.create_timer(1.0, self.timer_callback)

    def load_waypoints(self):
        try:
            with open('/home/sirius/sirius_ws/src/sirius_navigation/config/map.yaml', 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                self.waypoints = data.get('points', [])
        except FileNotFoundError:
            self.get_logger().error("File not found")
            self.waypoints = []
        except yaml.YAMLError:
            self.get_logger().error("YAML error")
            self.waypoints = []

    def create_interactive_marker(self, waypoint, id):
        int_marker_name = f"waypoint_{id}"
        
        # 既存マーカーがあるか確認
        existing_marker = self.int_marker_server.get(int_marker_name)
        
        if existing_marker:
            # 既存マーカーの位置を更新
            existing_marker.pose.position.x = waypoint[0]
            existing_marker.pose.position.y = waypoint[1]
            existing_marker.pose.position.z = waypoint[2]
            self.int_marker_server.insert(existing_marker)
        else:
            # 新しいマーカーを作成
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "map"
            int_marker.header.stamp = self.get_clock().now().to_msg()
            int_marker.name = int_marker_name
            int_marker.description = "Waypoint"
            int_marker.pose.position.x = waypoint[0]
            int_marker.pose.position.y = waypoint[1]
            int_marker.pose.position.z = waypoint[2]
            int_marker.pose.orientation.x = waypoint[3]
            int_marker.pose.orientation.y = waypoint[4]
            int_marker.pose.orientation.z = waypoint[5]
            int_marker.pose.orientation.w = waypoint[6]

            # 平面移動用コントロール
            move_control = InteractiveMarkerControl()
            move_control.name = "move_xy"
            move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
            move_control.always_visible = True  # 常に表示
            int_marker.controls.append(move_control)

            # Z軸回転用コントロール（必要なら残す）
            rotate_control = InteractiveMarkerControl()
            rotate_control.name = "rotate_z"
            rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            rotate_control.orientation.w = 1.0
            rotate_control.orientation.x = 0.0
            rotate_control.orientation.y = 1.0
            rotate_control.orientation.z = 0.0
            int_marker.controls.append(rotate_control)

            # 可視化用マーカー
            marker_control = InteractiveMarkerControl()
            marker_control.always_visible = True
            marker = Marker()
            marker.type = Marker.ARROW
            marker.scale.x = 1.0
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_control.markers.append(marker)

            # テキストマーカーを追加（色を黒に設定）
            text_marker = Marker()
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.text = f"{id}"
            text_marker.pose.position.x = waypoint[0]
            text_marker.pose.position.y = waypoint[1]
            text_marker.pose.position.z = waypoint[2] + 0.01  # 少し上に配置
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0  # 不透明
            text_marker.scale.z = 0.3  # フォントサイズ
            marker_control.markers.append(text_marker)
            int_marker.controls.append(marker_control)

            # 新しいマーカーをサーバーに追加
            self.int_marker_server.insert(int_marker)
            self.int_marker_server.setCallback(int_marker.name, self.process_feedback)

        # サーバーを更新
        self.int_marker_server.applyChanges()

    def create_marker(self, waypoint, id):
        marker = Marker()
        marker.type = Marker.ARROW
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        return marker

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # 位置が変更された場合の処理
            new_position = feedback.pose.position
            self.get_logger().info(f'Waypoint {feedback.marker_name} moved to {new_position}')

    def timer_callback(self):
        # インタラクティブマーカーノードをクリア
        # self.int_marker_server.clear()

        # ウェイポイントごとにインタラクティブマーカーを作成
        for counter, waypoint in enumerate(self.waypoints):
            self.create_interactive_marker(waypoint, counter)

        # すべてのインタラクティブマーカーをサーバーに更新
        self.int_marker_server.applyChanges()

def main(args=None):
    rclpy.init(args=args)
    waypoint_manager = WaypointManager()
    rclpy.spin(waypoint_manager)
    waypoint_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("Waypoint_Viewer made by Kotani: Started!!")
    main()
