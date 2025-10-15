import os
import yaml
from pathlib import Path
from pypinyin import lazy_pinyin
from rapidfuzz import fuzz
from math import radians, sin, cos, sqrt, atan2

def load_command_map(path, logger=None):
    if not os.path.exists(path):
        if logger:
            logger.error(f"Config file not found: {path}")
        return {}

    with open(path, "r", encoding="utf-8") as f:
        raw_map = yaml.safe_load(f) or {}

    flat_map = {}
    for action, aliases in raw_map.items():
        for alias in aliases:
            flat_map[alias] = int(action)
    return flat_map


def load_noise_config(path, logger=None):
    if not os.path.exists(path):
        if logger:
            logger.warn(f"Noise config not found: {path}, using empty list")
        return []
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    return cfg.get("noise_keywords", [])


def load_replacements_config(path, logger=None):
    if not os.path.exists(path):
        if logger:
            logger.warn(f"Replacements config not found: {path}, using empty dict")
        return {}
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    return cfg.get("replacements", {})


class PlaceMatcher:
    def __init__(self, config_path: str = "./map.yaml"):
        self.places_map = self._load_places(config_path)
        self.places_pinyin = {p: " ".join(lazy_pinyin(p)) for p in self.places_map.keys()}

    def _load_places(self, config_path: str):
        config_file = Path(config_path)
        if not config_file.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        with open(config_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return data or {}

    def find_best_match(self, text: str, threshold: int = 70):
        """Return best matching place name and score"""
        text_pinyin = " ".join(lazy_pinyin(text))
        best_match, best_score = None, 0
        for place, pinyin_form in self.places_pinyin.items():
            score_text = fuzz.partial_ratio(place, text)
            score_pinyin = fuzz.partial_ratio(pinyin_form, text_pinyin)
            score = max(score_text, score_pinyin)
            if score > best_score:
                best_match, best_score = place, score
        if best_score >= threshold:
            return best_match, best_score
        return None, best_score

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat/2)**2 + cos(radians(lat1))*cos(radians(lat2))*sin(dlon/2)**2
        return R * 2 * atan2(sqrt(a), sqrt(1-a))

    def find_nearest_point(self, place: str, cur_lat=None, cur_lon=None):
        """Return nearest lat/lon for given place"""
        if place not in self.places_map:
            return None
        candidates = self.places_map[place]
        if isinstance(candidates, dict):
            candidates = [candidates]
        if cur_lat is None or cur_lon is None:
            return candidates[0]
        return min(candidates, key=lambda p: self.haversine(
            cur_lat, cur_lon, p["lat"], p["lon"]
        ))

