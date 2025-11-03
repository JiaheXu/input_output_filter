#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class RawInputTester(Node):
    def __init__(self):
        super().__init__('raw_input_tester')
        self.pub = self.create_publisher(String, 'user_speech', 10)

        self.timer = self.create_timer(2.0, self.timer_callback)
        self.test_msgs = [
            "导航至御苑门",
            "带我去御园门",
            "带我去佛手印",
        ]
        self.index = 0
        self.get_logger().info("✅ RawInputTester started, publishing to 'raw_input'")

    def timer_callback(self):
        if self.index < len(self.test_msgs):
            msg = String()
            msg.data = self.test_msgs[self.index]
            self.pub.publish(msg)
            self.get_logger().info(f"📢 Published: {msg.data}")
            self.index += 1
        else:
            self.get_logger().info("🎉 All test messages published. Shutting down...")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RawInputTester()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
