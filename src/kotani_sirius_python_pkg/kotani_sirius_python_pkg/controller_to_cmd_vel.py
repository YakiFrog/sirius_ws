#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

class ControllerToCmdVel(Node):
    def __init__(self):
        super().__init__('controller_to_cmd_vel')
        # 一次遅れ系フィルタのための変数
        self.x_prev = 0.0
        self.z_prev = 0.0
        self.last_time = time.time()
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Node started: controller_to_cmd_vel")

    def joy_callback(self, msg):
        twist = Twist()
        # 値の範囲は-1.0から1.0
        twist.linear.x = msg.axes[1] * 0.6 # 左スティックの垂直軸
        twist.angular.z = msg.axes[2] * 0.6 # 右スティックの水平軸
        
        # 一次遅れ系フィルタ
        alpha = 1 # 0.0 < alpha < 1.0 (1に近いほど応答が早くなる)
        twist.linear.x = twist.linear.x * alpha + self.x_prev * (1 - alpha)
        twist.angular.z = twist.angular.z * alpha + self.z_prev * (1 - alpha)
        
        # 加速度制限
        max_acceleration = 0.5 # m/s^2 (速度よりも小さい値にする)
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        max_delta_v = max_acceleration * dt
        delta_linear_x = twist.linear.x - self.x_prev
        delta_angular_z = twist.angular.z - self.z_prev
        
        if abs(delta_linear_x) > max_delta_v:
            twist.linear.x = self.x_prev + max_delta_v * (1 if delta_linear_x > 0 else -1)
        
        if abs(delta_angular_z) > max_delta_v:
            twist.angular.z = self.z_prev + max_delta_v * (1 if delta_angular_z > 0 else -1)
            
        self.x_prev = twist.linear.x
        self.z_prev = twist.angular.z
        
        # 小数点以下第2位までの値に丸める
        twist.linear.x = round(twist.linear.x, 1) # 0.1以下の入力でガクガクなる
        
        # もし，linear.xが0.0のとき，angular.zの値を*2する
        if twist.linear.x == 0.0:
            twist.angular.z = twist.angular.z * 2
            
        twist.angular.z = round(twist.angular.z, 3) # 0.7
        
        self.publisher_.publish(twist)
        self.get_logger().info(f"Publishing: {twist}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
"""
コントローラの入力に基づいてTwistメッセージを生成し、/cmd_velトピックにパブリッシュするノードです。

$ ros2 run joy joy_node
$ ros2 run kotani_sirius_python_pkg controller_to_cmd_vel

"""