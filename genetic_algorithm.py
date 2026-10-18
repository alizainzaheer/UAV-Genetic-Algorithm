"""Object-oriented Genetic Algorithm for UAV route/sensor optimization."""
from __future__ import annotations
from dataclasses import dataclass
import copy
import numpy as np

@dataclass
class Mission:
    # Each drone has K rows, each row: [x, y, speed_gene, sensor_gene].
    genes: np.ndarray

class GeneticAlgorithm:
    def __init__(self, env, population_size=50, generations=100,
                 crossover_rate=0.8, mutation_rate=0.10, elite_count=2,
                 tournament_size=3, seed=12):
        self.env = env
        self.population_size = population_size
        self.generations = generations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.elite_count = elite_count
        self.tournament_size = tournament_size
        self.rng = np.random.default_rng(seed)
        self.waypoints_per_drone = 6
        self.speed_levels = [1.0, 2.0, 3.0]
        self.history = {"best": [], "average": [], "coverage": [], "detection": [],
                        "energy": [], "time": [], "risk": [], "feasible_rate": []}

    def random_mission(self):
        genes = np.zeros((self.env.num_drones, self.waypoints_per_drone, 4), dtype=float)
        for d in range(self.env.num_drones):
            for w in range(self.waypoints_per_drone):
                x = self.rng.uniform(0, self.env.size - 1)
                y = self.rng.uniform(0, self.env.size - 1)
                x, y = self.env.repair_waypoint((x, y))
                genes[d, w] = [x, y, self.rng.integers(0, 3), self.rng.integers(0, 2)]
        return Mission(genes)

    def repair(self, mission):
        g = mission.genes
        g[:, :, 0] = np.clip(g[:, :, 0], 0, self.env.size - 1)
        g[:, :, 1] = np.clip(g[:, :, 1], 0, self.env.size - 1)
        for d in range(self.env.num_drones):
            for w in range(self.waypoints_per_drone):
                x, y = self.env.repair_waypoint(g[d, w, :2])
                g[d, w, 0], g[d, w, 1] = x, y
                g[d, w, 2] = int(np.clip(round(g[d, w, 2]), 0, 2))
                g[d, w, 3] = int(g[d, w, 3] >= 0.5)
        return mission

    def paths(self, mission):
        paths = []
        for d in range(self.env.num_drones):
            points = [self.env.starts[d]]
            points.extend((float(row[0]), float(row[1])) for row in mission.genes[d])
            points.append(self.env.ends[d])
            paths.append(points)
        return paths

    def evaluate(self, mission):
        self.repair(mission)
        paths = self.paths(mission)
        coverage = np.zeros_like(self.env.priorities, dtype=bool)
        detected = set()
        total_energy = total_time = total_risk = 0.0
        violation = 0.0
        max_range = 180.0
        max_time = 450.0
        for d, path in enumerate(paths):
            drone_distance = 0.0
            for seg in range(len(path) - 1):
                p1, p2 = path[seg], path[seg + 1]
                dist = float(np.linalg.norm(np.array(p2) - np.array(p1)))
                drone_distance += dist
                # The speed/sensor gene for the segment ending at a waypoint.
                if seg < self.waypoints_per_drone:
                    speed_gene = int(mission.genes[d, seg, 2])
                    sensor_on = bool(mission.genes[d, seg, 3])
                else:
                    speed_gene, sensor_on = 1, bool(mission.genes[d, -1, 3])
                speed = self.speed_levels[speed_gene]
                total_energy += dist * (1.0 + 0.12 * speed**2) + (1.5 if sensor_on else 0.0)
                total_time += dist / speed + (1.0 if sensor_on else 0.0)
                samples = self.env.segment_samples(p1, p2, spacing=0.5)
                if not self.env.segment_is_safe(p1, p2):
                    violation += 1.0
                for x, y in samples:
                    total_risk += self.env.hazard_at(x, y) * (dist / max(1, len(samples)))
                    if sensor_on:
                        radius = 3.0
                        x0, x1 = max(0, int(x-radius)), min(self.env.size, int(x+radius)+1)
                        y0, y1 = max(0, int(y-radius)), min(self.env.size, int(y+radius)+1)
                        yy, xx = np.ogrid[y0:y1, x0:x1]
                        visible = (xx-x)**2 + (yy-y)**2 <= radius**2
                        coverage[y0:y1, x0:x1] |= visible
                        for ti, target in enumerate(self.env.targets):
                            if (target[0]-x)**2 + (target[1]-y)**2 <= radius**2:
                                detected.add(ti)
            if drone_distance > max_range:
                violation += (drone_distance - max_range) / max_range
        if total_time > max_time:
            violation += (total_time - max_time) / max_time
        valid_priority = self.env.priorities > 0
        denom = self.env.priorities[valid_priority].sum()
        covered_weight = self.env.priorities[coverage & valid_priority].sum()
        cov = float(covered_weight / denom) if denom else 0.0
        det = len(detected) / max(1, len(self.env.targets))
        # Normalize costs to keep all terms on comparable scales.
        e_norm = min(total_energy / 900.0, 2.0)
        t_norm = min(total_time / 180.0, 2.0)
        r_norm = min(total_risk / 180.0, 2.0)
        # Constraint penalty: severe enough that feasible plans are preferred.
        penalty = min(2.0, violation) * 0.75
        fitness = 0.45*cov + 0.25*det - 0.15*e_norm - 0.10*t_norm - 0.05*r_norm - penalty
        return {"fitness": float(fitness), "coverage": cov, "detection": float(det),
                "energy": float(total_energy), "time": float(total_time),
                "risk": float(total_risk), "violations": float(violation),
                "feasible": violation < 1e-9, "coverage_map": coverage, "paths": paths}

    def tournament_select(self, population, scores):
        indices = self.rng.integers(0, len(population), size=self.tournament_size)
        winner = max(indices, key=lambda i: scores[i]["fitness"])
        return copy.deepcopy(population[winner])

    def crossover(self, parent1, parent2):
        child1, child2 = copy.deepcopy(parent1), copy.deepcopy(parent2)
        if self.rng.random() < self.crossover_rate:
            # Swap one or more whole drone route segments to preserve linked genes.
            d = int(self.rng.integers(0, self.env.num_drones))
            start = int(self.rng.integers(0, self.waypoints_per_drone))
            end = int(self.rng.integers(start + 1, self.waypoints_per_drone + 1))
            child1.genes[d, start:end, :] = parent2.genes[d, start:end, :]
            child2.genes[d, start:end, :] = parent1.genes[d, start:end, :]
            # Also sometimes exchange a complete drone segment.
            if self.rng.random() < 0.25:
                d2 = int(self.rng.integers(0, self.env.num_drones))
                child1.genes[d2] = parent2.genes[d2].copy()
                child2.genes[d2] = parent1.genes[d2].copy()
        return self.repair(child1), self.repair(child2)

    def mutate(self, mission):
        for d in range(self.env.num_drones):
            for w in range(self.waypoints_per_drone):
                if self.rng.random() < self.mutation_rate:
                    # Gaussian spatial mutation.
                    mission.genes[d, w, 0] += self.rng.normal(0, 3.0)
                    mission.genes[d, w, 1] += self.rng.normal(0, 3.0)
                if self.rng.random() < self.mutation_rate:
                    mission.genes[d, w, 2] = self.rng.integers(0, 3)
                if self.rng.random() < self.mutation_rate:
                    mission.genes[d, w, 3] = 1 - int(mission.genes[d, w, 3])
        return self.repair(mission)

    def run(self):
        population = [self.random_mission() for _ in range(self.population_size)]
        best_overall = None
        best_score = None
        for gen in range(self.generations):
            evaluations = [self.evaluate(m) for m in population]
            scores = [e["fitness"] for e in evaluations]
            ranked = sorted(range(len(population)), key=lambda i: scores[i], reverse=True)
            if best_overall is None or scores[ranked[0]] > best_score["fitness"]:
                best_overall = copy.deepcopy(population[ranked[0]])
                best_score = self.evaluate(best_overall)
            self.history["best"].append(float(scores[ranked[0]]))
            self.history["average"].append(float(np.mean(scores)))
            for key in ["coverage", "detection", "energy", "time", "risk"]:
                self.history[key].append(float(evaluations[ranked[0]][key]))
            self.history["feasible_rate"].append(float(np.mean([e["feasible"] for e in evaluations])))
            if gen == self.generations - 1:
                break
            next_population = [copy.deepcopy(population[i]) for i in ranked[:self.elite_count]]
            while len(next_population) < self.population_size:
                p1 = self.tournament_select(population, evaluations)
                p2 = self.tournament_select(population, evaluations)
                c1, c2 = self.crossover(p1, p2)
                next_population.append(self.mutate(c1))
                if len(next_population) < self.population_size:
                    next_population.append(self.mutate(c2))
            population = next_population
        return best_overall, self.evaluate(best_overall), self.history
