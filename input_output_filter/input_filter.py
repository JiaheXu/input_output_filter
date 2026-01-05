#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
from std_msgs.msg import String, Int32, Bool
import difflib
import yaml
import os
from utils import (
    load_command_map,
    load_noise_config,
    load_replacements_config,
)
import time
from rapidfuzz import fuzz
from sensor_msgs.msg import NavSatFix
from datetime import datetime
from pino_msgs.srv import Text   # custom service
from pathlib import Path
from pino_msgs.msg import AudioMSG   # ✅ custom message
from math import radians, sin, cos, sqrt, atan2
from pypinyin import lazy_pinyin
from std_msgs.msg import String as RosString

home_dir = str(Path.home())
NAV_THRESHOLD = 70  # fuzzy match score threshold
SILENT_TIME = 90.0
close_distance = 20.0


def haversine_distance(lat1, lon1, lat2, lon2):
    """Compute great-circle distance in meters between two GPS points."""
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

class PlaceMatcher:
    def __init__(self, config_path: str = "./map.yaml"):
        self.points = self._load_points(config_path)

    def _load_points(self, config_path: str):
        """Load all points into a flat list with index, name, lat, lon"""
        config_file = Path(config_path)
        if not config_file.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        with open(config_file, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)

        points = []
        for idx, entry in raw.items():
            name = entry.get("name")
            lat, lon = entry.get("lat"), entry.get("lon")
            if not name or lat is None or lon is None:
                continue
            points.append({
                "idx": int(idx),
                "name": name,
                "lat": float(lat),
                "lon": float(lon),
                "pinyin": " ".join(lazy_pinyin(name))
            })
        return points

    def find_best_match(self, text: str, threshold: int = 70):
        """Find the best matching point using fuzzy matching"""
        text_pinyin = " ".join(lazy_pinyin(text))
        best_point, best_score = None, 0

        for p in self.points:
            name = p["name"]
            pinyin_form = p["pinyin"]

            # fuzzy partial match
            score_text = fuzz.partial_ratio(name, text)
            score_pinyin = fuzz.partial_ratio(pinyin_form, text_pinyin)
            score = max(score_text, score_pinyin)

            if score > best_score:
                best_point, best_score = p, score

        if best_score >= threshold:
            return best_point, best_score
        return None, best_score


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
        self.last_audio_time = time.time()
        self.NAV_KEYWORDS = ["导", "航", "路线", "地图", "位置", "坐标", "去","志", "到","至", "道行"]
        self.WANDER_KEYWORDS = ["随便走", "到处逛逛", "随便逛逛", "游览模式", "前进","向前走","往前","前走","往前走"]
        self.FUNC_PLACE = ["办公区", "肯德基", "茶话弄"]

        self.get_logger().info(f"Loaded command map from {config_path}, total commands={len(self.command_map)}")

        # Logs
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        self.log_file = os.path.join(
            os.path.dirname(__file__),
            f"received_input_{timestamp}.txt"
        )

        # Callback groups
        self.fast_group = MutuallyExclusiveCallbackGroup()  # GPS + speech
        self.llm_group = ReentrantCallbackGroup()           # LLM client
        self.timer_group = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.sub = self.create_subscription(
            String, '/user_speech', self.listener_callback, 10,
            callback_group=self.fast_group
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps_raw', self.gps_callback, 10,
            callback_group=self.fast_group
        )
        self.speaker_playing_sub = self.create_subscription(
            Bool, "speaker_playing", self.speaker_cb, 10,
            callback_group=self.fast_group
        )

        # Publishers
        self.motion_pub = self.create_publisher(Int32, '/motion_cmd', 10)
        self.llm_pub = self.create_publisher(String, '/llm_input', 10)
        self.nav_goal_pub = self.create_publisher(NavSatFix, "/nav_goal", 10)
        self.navigate_pub = self.create_publisher(String, '/navigate', 10)
        self.audio_pub = self.create_publisher(AudioMSG, "audio_cmd", 10)
        self.keypoint_pub = self.create_publisher(Int32, "keypoint", 10)

        # ---- NEW: Current goal publisher ----
        
        self.current_goal_pub = self.create_publisher(RosString, "/nav_current_goal", 10)

        # ---- NEW: record data flag ----
        self.record_data_pub = self.create_publisher(Bool, "/record_data", 10)
        self.record_data = False   # 默认 False

        # ---- Track current navigation goal ----
        self.current_goal_name = None
        self.current_goal_lat = None
        self.current_goal_lon = None

        self.voice = 'zf_xiaoyi'

        # Track GPS and visited points
        self.current_lat = None
        self.current_lon = None
        self.visited_points = set()

        # LLM service client (separate group)
        self.cli = self.create_client(Text, "llm_service", callback_group=self.llm_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting for llm_service...")
        
        self.create_timer(1.0, self.timer_callback, callback_group=self.timer_group)

    def publish_record_flag(self, flag: bool):
        msg = Bool()
        msg.data = flag
        self.record_data_pub.publish(msg)
        self.get_logger().info(f"📝 record_data = {flag}")

    # ---------- Utility methods ----------
    def publish_audio(self, text: str, cmd: str = 'speak', voice: str = 'zf_xiaoyi', volume: float = 3.0, speed: float = 1.0):
        msg = AudioMSG()
        msg.cmd = cmd
        msg.text = text
        msg.voice = voice
        msg.volume = volume
        msg.speed = speed
        self.audio_pub.publish(msg)
        self.get_logger().info(f"🎙️ Published AudioMSG: {msg}")

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

    def speaker_cb(self, msg: Bool):
        if(msg.data):
            self.last_audio_time = time.time()

    # ---------- GPS proximity intro ----------
    def gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.check_proximity_and_introduce()
        if self.current_goal_name is not None:
            dist = haversine_distance(
                self.current_lat, self.current_lon,
                self.current_goal_lat, self.current_goal_lon
            )

            if dist < close_distance:
                self.get_logger().info(
                    f"🎯 Arrived at goal {self.current_goal_name}, dist={dist:.1f}m → Clearing goal."
                )

                # Clear goal state
                self.current_goal_name = None
                self.current_goal_lat = None
                self.current_goal_lon = None

                # Publish None to UI
                msg_goal = RosString()
                msg_goal.data = "None"
                self.current_goal_pub.publish(msg_goal)

    def check_proximity_and_introduce(self):
        if self.current_lat is None or self.current_lon is None:
            return
        
        for point in self.place_matcher.points:
            place_name, lat, lon = point["name"], point["lat"], point["lon"]
            dist = haversine_distance(self.current_lat, self.current_lon, lat, lon)

            if ( dist <= close_distance ) and ( place_name not in self.visited_points) and ( place_name not in self.FUNC_PLACE):
                self.get_logger().info(f"📍 Near {place_name} ({dist:.1f}m) → introducing")

                self.visited_points.add(place_name)

                req = Text.Request()
                req.text = f"介绍{place_name}"
                future = self.cli.call_async(req)

                def _callback(fut):
                    try:
                        resp = fut.result()
                        if resp.success:
                            self.publish_audio(resp.response)
                        else:
                            self.publish_audio("介绍失败")
                    except Exception as e:
                        self.get_logger().error(f"❌ Intro LLM 调用异常: {e}")

                future.add_done_callback(_callback)

    # ---------- Speech commands ----------
    def listener_callback(self, msg: String):
        user_cmd = msg.data.strip()
        self.save_input(user_cmd)
        self.last_audio_time = time.time() 
        if any(k in user_cmd for k in self.noise_keywords):
            return

        self.get_logger().info(f"收到原始输入: {user_cmd}")

        # ---------- Record data voice control ----------
        if "记录数据" in user_cmd:
            self.record_data = True
            self.publish_record_flag(True)
            self.publish_audio("已开始记录数据")
            return

        if "停止记录" in user_cmd:
            self.record_data = False
            self.publish_record_flag(False)
            self.publish_audio("已停止记录")
            return


        # Motion commands
        is_action = self.recognize_action_command(user_cmd)
        if is_action is not None:
            self.get_logger().info(f"action type: {is_action}")
            self.semd_motion_cmd(is_action)
            if(is_action == 0): # stop cmd
                self.publish_audio(cmd = 'stop', text = '')
            return

        # Navigation
        
        if any(k in user_cmd for k in self.NAV_KEYWORDS):
            point, score = self.place_matcher.find_best_match(user_cmd, NAV_THRESHOLD)
            if point:
                self.get_logger().info(
                    f"🧭 导航请求 → {point['name']} idx={point['idx']} (score={score})"
                )

                # ---- NEW: store current goal ----
                self.current_goal_name = point["name"]
                self.current_goal_lat = point["lat"]
                self.current_goal_lon = point["lon"]

                # ---- NEW: publish current goal ----
                msg_goal = RosString()
                msg_goal.data = self.current_goal_name
                self.current_goal_pub.publish(msg_goal)

                # Original navigation publish
                nav_str = String()
                nav_str.data = f"NAVIGATE:{point['name']}"
                self.navigate_pub.publish(nav_str)

                nav_msg = NavSatFix()
                nav_msg.header.stamp = self.get_clock().now().to_msg()
                nav_msg.header.frame_id = "map"
                nav_msg.latitude = point["lat"]
                nav_msg.longitude = point["lon"]
                nav_msg.altitude = 0.0
                self.nav_goal_pub.publish(nav_msg)

                idx_msg = Int32()
                idx_msg.data = point["idx"]
                self.keypoint_pub.publish(idx_msg)

                motion_msg = Int32()
                motion_msg.data = 20
                self.motion_pub.publish(motion_msg)

                self.publish_audio(f"好的，正在规划到{point['name']}的路径。")
                return

            self.publish_audio("抱歉，我没有找到合适路径。")
            return


        # Wander
        if any(k in user_cmd for k in self.WANDER_KEYWORDS):
            motion_msg = Int32()
            motion_msg.data = 30
            self.motion_pub.publish(motion_msg)
            self.publish_audio("好的，切换至游览模式")
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

    # -------- Timers -------- #
    def timer_callback(self):
        now = time.time()
        if(now - self.last_audio_time > SILENT_TIME):
            msg = String()
            msg.data = "随机对话。"
            self.listener_callback(msg)
            

# ---------- Main ----------
def main(args=None):
    rclpy.init(args=args)
    node = CommandMapper()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
