import rclpy
from rclpy.node import Node
from pino_msgs.msg import AudioMSG   # adjust if package name is different
import random

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_audio_publisher')
        self.publisher_ = self.create_publisher(AudioMSG, 'audio_cmd', 10)
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = AudioMSG()
        # Example texts with different patterns
        sample_texts = [
            "大家好，现在时间是12:00",
            "请在15:30-16:30之间到达",
            "欢迎来到（测试用例）大唐芙蓉园",
            "普通一句话测试",
        ]
        msg.text = random.choice(sample_texts)
        msg.voice = "zf_xiaoyi"
        msg.volume = 2.0
        msg.speed = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(f"📝 Published test msg: {msg.text}")

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Test publisher shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

