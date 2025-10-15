#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import difflib
import yaml
import os
from utils import (
    load_command_map,
    load_noise_config,
    load_replacements_config,
    PlaceMatcher
)
from sensor_msgs.msg import NavSatFix
from datetime import datetime
from pino_msgs.srv import Text   # custom service
from pathlib import Path
from pino_msgs.msg import AudioMSG   # ✅ use your custom message

home_dir = str(Path.home())
NAV_THRESHOLD = 70  # fuzzy match score threshold

class CommandMapper(Node):
    def __init__(self):
        super().__init__('command_mapper')

        config_path = home_dir + '/javis_ws/src/input_output_filter/config/'
        config_file = config_path + 'command_map.yaml'
        noise_file = config_path + 'noise.yaml'
        repl_file = config_path + 'replacements.yaml'
        places_file = config_path + 'map.yaml'

        # Load configs
        self.noise_keywords = load_noise_config(noise_file)
        self.replacements = load_replacements_config(repl_file)
        self.command_map = load_command_map(config_file)
        self.place_matcher = PlaceMatcher(places_file)
        
        self.NAV_KEYWORDS = ["导航", "路线", "地图", "位置", "坐标", "去", "到"]

        self.get_logger().info(f"Loaded command map from {config_path}, total commands={len(self.command_map)}")

        # Logs
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        self.log_file = os.path.join(
            os.path.dirname(__file__),
            f"received_input_{timestamp}.txt"
        )

        # Subscribers
        self.sub = self.create_subscription(String, '/user_speech', self.listener_callback, 10)
        self.gps_sub = self.create_subscription(String, '/gps_raw', self.gps_callback, 10)

        # Publishers
        self.motion_pub = self.create_publisher(Int32, '/motion_cmd', 10)
        self.llm_pub = self.create_publisher(String, '/llm_input', 10)
        self.nav_goal_pub = self.create_publisher(NavSatFix, "/nav_goal", 10)
        self.navigate_pub = self.create_publisher(String, '/navigate', 10)
        self.audio_pub = self.create_publisher(AudioMSG, "audio_cmd", 10)
        
        # GPS state
        self.current_lat = None
        self.current_lon = None
        
        self.voice = 'zf_xiaoxiao'
        # LLM service client
        self.cli = self.create_client(Text, "llm_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting for llm_service...")

    def publish_audio(self, text: str, cmd: str= 'speak', voice: str = None, volume: float = 0.0, speed: float = 0.0):
        msg = AudioMSG()
        msg.cmd = cmd
        msg.text = text
        msg.voice = voice if voice else self.voice
        msg.volume = volume
        msg.speed = speed
        self.audio_pub.publish(msg)
        self.get_logger().info(f"🎙️ Published AudioMSG: {msg}")


    def gps_callback(self, msg: String):
        try:
            lat, lon = map(float, msg.data.split(","))
            self.current_lat, self.current_lon = lat, lon
        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse GPS: {e}")

    def save_input(self, text: str):
        try:
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(text + "\n")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save input: {e}")

    def semd_motion_cmd(self, motion_cmd):
        msg = Int32()
        msg.data = motion_cmd
        self.motion_pub.publish(msg)
    
    def recognize_action_command(self, user_cmd: str) -> str:
        if user_cmd in self.command_map:
            return self.command_map[user_cmd]
        for key in self.command_map.keys():
            if key in user_cmd:
                return self.command_map[key]
        matches = difflib.get_close_matches(user_cmd, self.command_map.keys(), n=1, cutoff=0.6)
        if matches:
            return self.command_map[matches[0]]
        return None

    def replace_substrings(self, text: str, replacements: dict) -> str:
        for old, new in replacements.items():
            text = text.replace(old, new)
        return text

    def listener_callback(self, msg: String):
        user_cmd = msg.data.strip()
        self.save_input(user_cmd)

        if any(k in user_cmd for k in self.noise_keywords):
            return

        self.get_logger().info(f"收到原始输入: {user_cmd}")

        # Motion commands
        is_action = self.recognize_action_command(user_cmd)
        if is_action is not None:
            self.get_logger().info(f"action type: {is_action}")        
            self.semd_motion_cmd(is_action)
            return
        else:
            self.get_logger().info("Not a motion cmd !!!!!!!!!!")

        # Place navigation
        if any(k in user_cmd for k in self.NAV_KEYWORDS):
            self.get_logger().info(f"NAVIGATION CMD!!!")
            place, score = self.place_matcher.find_best_match(user_cmd)
            if place and score > NAV_THRESHOLD:
                target = self.place_matcher.find_nearest_point(place, self.current_lat, self.current_lon)
                if target:
                    self.get_logger().info(f"🧭 导航请求 → {place} 最近点 {target} (score={score})")

                    # Publish readable name
                    nav_str = String()
                    nav_str.data = f"NAVIGATE:{place}"
                    self.navigate_pub.publish(nav_str)

                    # Publish GPS goal
                    nav_msg = NavSatFix()
                    nav_msg.header.stamp = self.get_clock().now().to_msg()
                    nav_msg.header.frame_id = "map"
                    nav_msg.latitude = target["lat"]
                    nav_msg.longitude = target["lon"]
                    nav_msg.altitude = 0.0
                    self.nav_goal_pub.publish(nav_msg)
                    self.publish_audio("好的，正在规划路径中。")
                    return
            self.get_logger().info(f"🧭 导航请求 FAILED !!!!!!!!!!!!!")
            self.publish_audio("抱歉，我没有找到合适路径。")
            return
        # Otherwise → send to LLM
        req = Text.Request()
        req.text = self.replace_substrings(user_cmd, self.replacements)
        future = self.cli.call_async(req)

        def _callback(fut):
            try:
                resp = fut.result()
                out_msg = String()
                if resp.success:
                    out_msg.data = resp.response
                else:
                    out_msg.data = "LLM 失败: " + resp.response
                self.llm_pub.publish(out_msg)
                self.get_logger().info(f"→ 发布到 /robot_action: {out_msg.data}")
            except Exception as e:
                self.get_logger().error(f"❌ LLM 调用异常: {e}")

        future.add_done_callback(_callback)


def main(args=None):
    rclpy.init(args=args)
    node = CommandMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

