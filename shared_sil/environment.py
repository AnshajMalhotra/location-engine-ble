# environment.py
import numpy as np
from typing import List, Tuple
from config import SpatialConfig

class MapBuilder:
    def __init__(self, config: SpatialConfig):
        self.config = config
        self.rooms: List[List[float]] = []
        self.walls: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
        self.anchors: List[np.ndarray] = []

    def build(self):
        # 简化版：硬编码生成双侧走廊拓扑 (实际应根据 config 动态演算)
        self.rooms = [
            [0, 6, 5, 10], [5, 6, 10, 10], [10, 6, 15, 10], [15, 6, 20, 10],
            [0, 0, 5, 4], [5, 0, 10, 4], [10, 0, 15, 4], [15, 0, 20, 4],
            [0, 4, 20, 6]
        ]
        self.anchors = np.array([
            [2.5, 8], [7.5, 8], [12.5, 8], [17.5, 8],
            [2.5, 2], [7.5, 2], [12.5, 2], [17.5, 2],
            [5, 5], [15, 5]
        ])
        self.walls = [
            [(1, 6), (5, 6)], [(6, 6), (10, 6)], [(11, 6), (15, 6)], [(16, 6), (20, 6)],
            [(1, 4), (5, 4)], [(6, 4), (10, 4)], [(11, 4), (15, 4)], [(16, 4), (20, 4)],
            [(5, 6), (5, 10)], [(10, 6), (10, 10)], [(15, 6), (15, 10)],
            [(5, 0), (5, 4)], [(10, 0), (10, 4)], [(15, 0), (15, 4)]
        ]
        return self.rooms, self.walls, self.anchors