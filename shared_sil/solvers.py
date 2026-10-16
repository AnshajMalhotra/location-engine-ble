# solvers.py
import numpy as np
from abc import ABC, abstractmethod
from typing import List
from scipy.optimize import minimize
from config import RFConfig
from evaluator import PerformanceEvaluator

class BaseLocator(ABC):
    @abstractmethod
    def step(self, rssi_list: np.ndarray) -> np.ndarray:
        pass

class RawSolver(BaseLocator):
    def __init__(self, anchors: np.ndarray, rf_config: RFConfig, top_k: int = 4):
        self.anchors = anchors
        self.rf = rf_config
        self.top_k = top_k

    def _rssi_to_distance(self, rssi):
        return 10 ** ((self.rf.rssi_at_1m - rssi) / (10 * self.rf.path_loss_exp))

    def step(self, rssi_list: np.ndarray) -> np.ndarray:
        top_indices = np.argsort(rssi_list)[-self.top_k:]
        def loss_function(pos):
            err = 0
            for idx in top_indices:
                err += (np.linalg.norm(pos - self.anchors[idx]) - self._rssi_to_distance(rssi_list[idx]))**2
            return err
        result = minimize(loss_function, np.array([10.0, 5.0]), method='Nelder-Mead', options={'maxiter': 200})
        return result.x

class ParticleFilterSolver(BaseLocator):
    def __init__(self, num_particles: int, rooms: List, walls: List, anchors: np.ndarray, rf_config: RFConfig, use_waf: bool = True):
        self.num_particles = num_particles
        self.rooms = rooms
        self.walls = walls
        self.anchors = anchors
        self.rf = rf_config
        self.use_waf = use_waf
        self.particles = np.random.uniform(low=[0, 0], high=[20, 10], size=(num_particles, 2))
        self.weights = np.ones(num_particles) / num_particles
        self.motion_noise = 1.5
        self.top_k = 4

    def _ccw(self, A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    def _check_intersection(self, p1, p2, w1, w2):
        return self._ccw(p1, w1, w2) != self._ccw(p2, w1, w2) and self._ccw(p1, p2, w1) != self._ccw(p1, p2, w2)

    def _is_walkable(self, old_pos, new_pos):
        x, y = new_pos
        inside = any([r[0] <= x <= r[2] and r[1] <= y <= r[3] for r in self.rooms])
        if not inside: return False
        for w in self.walls:
            if self._check_intersection(old_pos, new_pos, w[0], w[1]):
                return False
        return True

    def _predict(self):
        noise = np.random.normal(0, self.motion_noise, size=(self.num_particles, 2))
        proposed = self.particles + noise
        for i in range(self.num_particles):
            if self._is_walkable(self.particles[i], proposed[i]):
                self.particles[i] = proposed[i]

    def _update(self, rssi_list: np.ndarray):
        top_indices = np.argsort(rssi_list)[-self.top_k:]
        for i, particle in enumerate(self.particles):
            prob = 1.0
            for idx in top_indices:
                anchor = self.anchors[idx]
                dist = max(np.linalg.norm(particle - anchor), 0.1)
                expected_rssi = self.rf.rssi_at_1m - 10 * self.rf.path_loss_exp * np.log10(dist)

                if self.use_waf:
                    wall_crossings = sum([1 for w in self.walls if self._check_intersection(particle, anchor, w[0], w[1])])
                    expected_rssi -= (wall_crossings * self.rf.waf_dbm)

                error = rssi_list[idx] - expected_rssi
                prob *= np.exp(-(error**2) / (2 * self.rf.noise_std**2))

            self.weights[i] = prob + 1e-12 

        total_weight = np.sum(self.weights)
        if total_weight < 1e-15:
            self.particles = np.random.uniform(low=[0, 0], high=[20, 10], size=(self.num_particles, 2))
            self.weights = np.ones(self.num_particles) / self.num_particles
        else:
            self.weights /= total_weight

    def _resample(self):
        indices = np.random.choice(self.num_particles, size=self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def _estimate(self):
        return np.average(self.particles, axis=0, weights=self.weights)

    def step(self, rssi_list: np.ndarray) -> np.ndarray:
        self._predict()
        self._update(rssi_list)
        est_pos = self._estimate()
        self._resample()
        return est_pos