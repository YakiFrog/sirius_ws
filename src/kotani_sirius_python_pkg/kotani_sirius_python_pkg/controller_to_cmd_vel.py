#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.duration import Duration

class ControllerToCmdVel(Node):
    def __init__(self):
        super().__init__('controller_to_cmd_vel')
        
        # Joyメッセージ受信のタイムアウト監視用
        self.last_joy_time = self.get_clock().now()
        self.timeout_duration = Duration(seconds=1.0)  # タイムアウト時間（秒）

        # 一次遅れ系フィルタのための変数
        self.x_prev = 0.0
        self.z_prev = 0.0

        # パブリッシャーとサブスクライバーの設定
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning

        # タイマー設定 (タイムアウト監視)
        self.create_timer(0.1, self.check_timeout)

        self.get_logger().info("Node started: controller_to_cmd_vel")

    def joy_callback(self, msg):
        # 最後のJoyメッセージ受信時間を更新
        self.last_joy_time = self.get_clock().now()
        twist = Twist()
        
        # 小さな値を無視する（デッドバンド）
        if abs(msg.axes[1]) < 0.03:
            msg.axes[1] = 0.0
        if abs(msg.axes[2]) < 0.1:
            msg.axes[2] = 0.0
            
        twist.linear.x = msg.axes[1] * 0.6  # 左スティックの垂直軸
        twist.angular.z = msg.axes[2] * 0.6  # 右スティックの水平軸
        
        # 加速度制限
        max_acceleration = 0.5  # m/s^2
        current_time = self.get_clock().now()
        dt = (current_time - self.last_joy_time).nanoseconds / 1e9  # 秒に変換
        
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
        twist.linear.x = round(twist.linear.x, 2)
        
        # もし，linear.xが0.0のとき，angular.zの値を*2する
        if twist.linear.x == 0.0:
            twist.angular.z = twist.angular.z * 2
            
        twist.angular.z = round(twist.angular.z, 3)
        
        self.publisher_.publish(twist)
        self.get_logger().info(f"Publishing: {twist}")

    def check_timeout(self):
        """Joyメッセージのタイムアウトをチェックし、停止信号を送信する。"""
        current_time = self.get_clock().now()
        if (current_time - self.last_joy_time) > self.timeout_duration:
            # タイムアウトとみなして停止信号を送信
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.get_logger().warn("No Joy message received. Stopping the robot.")

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
