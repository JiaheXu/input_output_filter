import rclpy
from rclpy.node import Node
import socket
import json
import re

from pino_msgs.msg import AudioMSG   # adjust package name if different

class UDPSegmentSenderNode(Node):
    def __init__(self):
        super().__init__('udp_segment_sender')

        # Declare parameters (with defaults)
        self.declare_parameter('host', '192.168.1.120')
        self.declare_parameter('port', 8888)
        self.declare_parameter('default_voice', 'zf_xiaoyi')
        self.declare_parameter('default_volume', 3.0)
        self.declare_parameter('default_speed', 0.8)

        # Load parameter values
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.default_voice = self.get_parameter('default_voice').get_parameter_value().string_value
        self.default_volume = self.get_parameter('default_volume').get_parameter_value().double_value
        self.default_speed = self.get_parameter('default_speed').get_parameter_value().double_value

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Create subscriber
        self.subscription = self.create_subscription(
            AudioMSG,
            'audio_cmd',   # topic name
            self.listener_callback,
            10
        )

        self.get_logger().info(f"🚀 UDP Segment Sender Node started, sending to {self.host}:{self.port}")
        self.init()

    def init(self):
        audio_msg = AudioMSG()
        audio_msg.text = "语音测试"
        audio_msg.cmd = "speak"
        audio_msg.voice = 'zf_xiaoyi'
        audio_msg.volume = 1.2
        audio_msg.speed = 0.8
        
        self.listener_callback( audio_msg )

        return

    def normalize_time(self, text: str) -> str:
        def repl_colon(match):
            hh, mm = match.groups()
            if mm == "00":
                return f"{int(hh)}点"
            return f"{int(hh)}点{mm}"

        text = re.sub(r'(\d{1,2}):(\d{2})', repl_colon, text)
        text = re.sub(r'(\d+点\d*)(?:-)(\d+点\d*)', r'\1至\2', text)
        text = re.sub(r'-', '', text)   # ✅ remove any remaining '-'
        return text

    def remove_parentheses_content(self, text: str) -> str:
        cleaned = re.sub(r"\([^)]*\)", "", text)      # ()
        cleaned = re.sub(r"\（[^)]*\）", "", cleaned)  # （）
        cleaned = re.sub(r"\s+", " ", cleaned).strip()
        return cleaned

    def listener_callback(self, msg: AudioMSG):
        print("in callback")
        self.get_logger().info(f"📥 Received msg: {msg.text}")

        text = self.normalize_time(self.remove_parentheses_content(msg.text))
        if(len(text)< 4):
            return
        self.get_logger().info(f"🧹 Cleaned text: {text}")
        self.get_logger().info(f"AudioMSG type: {msg.cmd}")
        if(msg.cmd == ""):
            msg.cmd = 'speak'
        payload = {
            "cmd": msg.cmd,
            "text": text,
            "voice": msg.voice if msg.voice else self.default_voice,
            # "volume": msg.volume if msg.volume > 0.1 else self.default_volume,
            "volume": 3.0,
            "speed": msg.speed if msg.speed > 0.2 else self.default_speed,
        }

        try:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.sock.sendto(data, (self.host, self.port))
            self.get_logger().info(f"📤 UDP Sent: {payload}")
        except Exception as e:
            self.get_logger().error(f"❌ UDP send error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UDPSegmentSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down UDP Segment Sender Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

