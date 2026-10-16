# config.py
from dataclasses import dataclass
from typing import Tuple

@dataclass
class SpatialConfig:
    layout_style: str = 'double_corridor'
    room_grid: Tuple[int, int] = (4, 2)
    room_size: Tuple[float, float] = (5.0, 4.0)
    corridor_width: float = 2.0
    wall_thickness: float = 0.0
    anchor_deployment: str = 'hybrid'

@dataclass
class RFConfig:
    rssi_at_1m: float = -60.0
    path_loss_exp: float = 2.5
    noise_std: float = 3.5
    waf_dbm: float = 8.0

@dataclass
class KinematicsConfig:
    tag_hz: float = 1.0
    base_speed: float = 1.2
    speed_noise_std: float = 0.3