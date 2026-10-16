# simulator.py
import numpy as np
from typing import List, Tuple
from config import KinematicsConfig, RFConfig

class TrajectoryGenerator:
    def __init__(self, config: KinematicsConfig):
        self.config = config

    def generate(self, waypoints: List[Tuple[float, float]]) -> np.ndarray:
        path = []
        current_pos = np.array(waypoints[0])
        path.append(current_pos.copy())
        dt = 1.0 / self.config.tag_hz

        for wp in waypoints[1:]:
            target = np.array(wp)
            while np.linalg.norm(target - current_pos) > 0.1:
                speed = max(0.5, np.random.normal(self.config.base_speed, self.config.speed_noise_std))
                step_dist = speed * dt
                vec = target - current_pos
                dist = np.linalg.norm(vec)
                
                if step_dist >= dist:
                    current_pos = target.copy()
                else:
                    current_pos = current_pos + (vec / dist) * step_dist
                path.append(current_pos.copy())
        return np.array(path)

class SensorModel:
    def __init__(self, rf_config: RFConfig, walls: List, anchors: np.ndarray):
        self.rf = rf_config
        self.walls = walls
        self.anchors = anchors

    def _ccw(self, A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    def _check_intersection(self, p1, p2, w1, w2):
        return self._ccw(p1, w1, w2) != self._ccw(p2, w1, w2) and self._ccw(p1, p2, w1) != self._ccw(p1, p2, w2)

    def get_measurements(self, true_pos: np.ndarray) -> np.ndarray:
        rssi_list = []
        for anchor in self.anchors:
            dist = max(np.linalg.norm(true_pos - anchor), 0.1)
            clean_rssi = self.rf.rssi_at_1m - 10 * self.rf.path_loss_exp * np.log10(dist)
            
            wall_crossings = sum([1 for w in self.walls if self._check_intersection(true_pos, anchor, w[0], w[1])])
            real_rssi = clean_rssi - (wall_crossings * self.rf.waf_dbm)
            
            noisy_rssi = real_rssi + np.random.normal(0, self.rf.noise_std)
            rssi_list.append(noisy_rssi)
        return np.array(rssi_list)

