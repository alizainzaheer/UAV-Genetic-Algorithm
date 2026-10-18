"""Run the UAV Genetic Algorithm project."""
from pathlib import Path
import sys
import json
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))

from environment import UAVEnvironment
from genetic_algorithm import GeneticAlgorithm
from visualization import save_visualizations

def main():
    env = UAVEnvironment(size=50, num_drones=3, seed=7)
    ga = GeneticAlgorithm(
        env,
        population_size=50,
        generations=100,
        crossover_rate=0.8,
        mutation_rate=0.10,
        elite_count=2,
        tournament_size=3,
        seed=12
    )
    print("Starting UAV Genetic Algorithm...")
    best_mission, result, history = ga.run()
    save_visualizations(env, best_mission, result, history, ROOT / "results")

    summary = {k: result[k] for k in
               ["fitness", "coverage", "detection", "energy", "time", "risk", "violations", "feasible"]}
    with open(ROOT / "results" / "metrics.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print("\nBest mission metrics")
    print("--------------------")
    for key, value in summary.items():
        print(f"{key:12s}: {value}")
    print(f"\nSaved plots and summary to: {ROOT / 'results'}")
    print("Note: this is a simplified grid simulation, not real flight certification.")

if __name__ == "__main__":
    main()
