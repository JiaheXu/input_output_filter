#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class RawInputTester(Node):
    def __init__(self):
        super().__init__('raw_input_tester')
        self.pub = self.create_publisher(String, 'user_speech', 10)

        self.timer = self.create_timer(20.0, self.timer_callback)
        self.test_msgs = [
            # "导航至 御苑门",
            # "带我去御园门",
            # "介绍御园门",
            # "来一段舞蹈",
            # "站起来",
            # "卧倒",
            # "往前走",
            # "退一步",
            # "右转弯",
            # "握个手",
            # "跳一下",
            # "月球漫步",            
            # "快一点",            
            # "慢一点",            
            # "走快点",            
            # "等下我",            
            # "停下来",
            # "停",
            "唱首歌",

            "给我一首李白的诗",
            "给我一首杜甫的诗",
            "给我一首李白的田园诗",
            "给我作一首诗",
            "给我作一首爱国诗",

            "今天的天气真好",
            "你是谁",
            "讲个笑话吧",
            "做个自我介绍",

            "带我去吃宫廷菜",
            "我想吃面食",
            "哪里有清真菜",
            "我想吃土",

            "园区有多大？",
            "参观大概需要多久？",
            "景区几点关门？",
            "哪里有卫生间？",
            "景区哪里有吃的？",

            "给我推荐一个节目",
            "给我推荐一个歌舞表演",
            "接下来元功门有什么表演",
            "给我节目上春山的具体信息",
            "十二点有什么表演",
            "十一点有什么表演",
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
