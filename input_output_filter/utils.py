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


