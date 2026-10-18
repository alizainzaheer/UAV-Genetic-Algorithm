"""Grid environment for multi-UAV disaster surveillance."""
from __future__ import annotations
import numpy as np

class UAVEnvironment:
    def __init__(self, size: int = 50, num_drones: int = 3, seed: int = 7):
        self.size = size
        self.num_drones = num_drones
        self.rng = np.random.default_rng(seed)
        self.obstacles = np.zeros((size, size), dtype=bool)
        self.hazards = np.zeros((size, size), dtype=float)
        self.priorities = np.full((size, size), 0.15, dtype=float)
        self.targets = [(10, 40), (25, 12), (38, 35), (42, 8), (15, 25)]
        self.starts = [(2, 2), (2, 47), (47, 2)]
        self.ends = [(47, 47), (47, 25), (25, 47)]
        self._build_map()

    def _build_map(self):
        # Obstacles: [y_start:y_end, x_start:x_end]
        self.obstacles[8:15, 8:17] = True
        self.obstacles[18:26, 30:34] = True
        self.obstacles[33:40, 12:20] = True
        self.obstacles[5:9, 36:44] = True
        self.obstacles[39:44, 29:38] = True
        # Hazard pockets
        self.hazards[20:29, 8:17] = 0.85
        self.hazards[31:39, 37:45] = 0.65
        self.hazards[7:13, 24:30] = 0.45
        # Priority areas
        self.priorities[5:15, 35:45] = 1.0
        self.priorities[30:42, 3:12] = 0.9
        self.priorities[22:31, 22:29] = 0.75
        self.priorities[self.obstacles] = 0.0
        for x, y in self.starts + self.ends:
            self.obstacles[y, x] = False

    def is_blocked(self, x: float, y: float) -> bool:
        ix = int(np.clip(round(x), 0, self.size - 1))
        iy = int(np.clip(round(y), 0, self.size - 1))
        return bool(self.obstacles[iy, ix])

    def nearest_safe(self, x: float, y: float) -> tuple[float, float]:
        x = float(np.clip(x, 0, self.size - 1))
        y = float(np.clip(y, 0, self.size - 1))
        ix, iy = int(round(x)), int(round(y))
        if not self.obstacles[iy, ix]:
            return x, y
        safe_y, safe_x = np.where(~self.obstacles)
        idx = np.argmin((safe_x - x) ** 2 + (safe_y - y) ** 2)
        return float(safe_x[idx]), float(safe_y[idx])

    def segment_samples(self, p1, p2, spacing=0.5):
        distance = float(np.linalg.norm(np.array(p2, dtype=float) - np.array(p1, dtype=float)))
        count = max(2, int(distance / spacing) + 1)
        return [tuple(np.array(p1) + (np.array(p2) - np.array(p1)) * t)
                for t in np.linspace(0, 1, count)]

    def segment_is_safe(self, p1, p2) -> bool:
        return all(not self.is_blocked(x, y) for x, y in self.segment_samples(p1, p2))

    def repair_waypoint(self, point):
        return self.nearest_safe(float(point[0]), float(point[1]))

    def hazard_at(self, x, y):
        ix = int(np.clip(round(x), 0, self.size - 1))
        iy = int(np.clip(round(y), 0, self.size - 1))
        return float(self.hazards[iy, ix])

    def priority_total(self):
        return float(self.priorities.sum())
