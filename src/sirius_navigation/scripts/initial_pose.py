#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml 
import argparse

class InitialPosePublisher(Node):
    def __init__(self, map_yaml = "map.yaml"):
        super().__init__('initial_pose_publisher')
        map_path = "/home/sirius24/sirius_ws/map_waypoints/map/{}".format(map_yaml)

        # YAMLファイルを読み込む
        try:
            with open(map_path, 'r') as file:
                map_data = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f"Map file {map_path} not found.")
            return
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file {map_path}: {e}")
            return

        # "origin"キーの確認
        if "origin" not in map_data:
            self.get_logger().error("Origin data not found in map file.")
            return

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.initial_pose = map_data["origin"]
        
        # 初期位置を一度だけパブリッシュする
        self.publish_initial_pose()

    def publish_initial_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        
        pose_msg.pose.pose.position.x = self.initial_pose[0]
        pose_msg.pose.pose.position.y = self.initial_pose[1]
        pose_msg.pose.pose.position.z = 0.0
    
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = 0.01
        pose_msg.pose.covariance[7] = 0.01
        pose_msg.pose.covariance[35] = 0.01
        
        self.publisher.publish(pose_msg)
        self.get_logger().info("Published initial pose.")
        
def main(args=None):
    parser = argparse.ArgumentParser(description="Set the map yaml file name.")
    parser.add_argument('--map_yaml', type=str, default="map.yaml", help="Map yaml file name (default: map.yaml)")
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    node = InitialPosePublisher(map_yaml=parsed_args.map_yaml)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
