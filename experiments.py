"""Baseline and hyperparameter experiments for the assignment."""
from pathlib import Path
import argparse
import csv
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))
from environment import UAVEnvironment
from genetic_algorithm import GeneticAlgorithm

def baseline_random_search(env, trials=500, seed=99):
    # Use the same chromosome representation and evaluator for a fair comparison.
    ga = GeneticAlgorithm(env, population_size=10, generations=1, seed=seed)
    best = None
    best_result = None
    start = time.perf_counter()
    for _ in range(trials):
        candidate = ga.random_mission()
        result = ga.evaluate(candidate)
        if best_result is None or result["fitness"] > best_result["fitness"]:
            best, best_result = candidate, result
    return best_result, time.perf_counter() - start

def run_ga(env, population=50, generations=100, mutation=0.10, tournament=3, seed=12):
    ga = GeneticAlgorithm(env, population_size=population, generations=generations,
                          mutation_rate=mutation, tournament_size=tournament, seed=seed)
    start = time.perf_counter()
    _, result, _ = ga.run()
    elapsed = time.perf_counter() - start
    return result, elapsed

def write_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["baseline", "sensitivity", "all"], default="baseline")
    args = parser.parse_args()
    out = ROOT / "results"
    env = UAVEnvironment(size=50, num_drones=3, seed=7)

    if args.mode in ("baseline", "all"):
        print("Running GA versus random-search baseline...")
        random_result, random_seconds = baseline_random_search(env, trials=500)
        ga_result, ga_seconds = run_ga(env, population=50, generations=100)
        rows = [
            {"method": "Random search (500 candidates)", "fitness": random_result["fitness"],
             "coverage": random_result["coverage"], "detection": random_result["detection"],
             "energy": random_result["energy"], "time": random_result["time"],
             "risk": random_result["risk"], "violations": random_result["violations"],
             "feasible": random_result["feasible"], "runtime_seconds": round(random_seconds, 3)},
            {"method": "Genetic Algorithm (50 x 100)", "fitness": ga_result["fitness"],
             "coverage": ga_result["coverage"], "detection": ga_result["detection"],
             "energy": ga_result["energy"], "time": ga_result["time"],
             "risk": ga_result["risk"], "violations": ga_result["violations"],
             "feasible": ga_result["feasible"], "runtime_seconds": round(ga_seconds, 3)}
        ]
        write_rows(out / "baseline_comparison.csv", rows)
        for row in rows:
            print(row)

    if args.mode in ("sensitivity", "all"):
        print("Running parameter sensitivity experiments. This may take several minutes.")
        rows = []
        experiments = (
            [("population_size", v, v, 50, 0.10, 3) for v in (20, 50, 100)] +
            [("mutation_rate", v, 50, 50, v, 3) for v in (0.03, 0.10, 0.20)] +
            [("tournament_size", v, 50, 50, 0.10, v) for v in (2, 3, 5)]
        )
        # Tuple format: label, value, population, generations, mutation, tournament.
        for label, value, pop, gens, mutation, tournament in experiments:
            result, seconds = run_ga(env, population=pop, generations=gens,
                                     mutation=mutation, tournament=tournament, seed=12)
            row = {"parameter": label, "value": value, "population_size": pop,
                   "generations": gens, "mutation_rate": mutation,
                   "tournament_size": tournament, "fitness": result["fitness"],
                   "coverage": result["coverage"], "detection": result["detection"],
                   "energy": result["energy"], "time": result["time"],
                   "risk": result["risk"], "violations": result["violations"],
                   "feasible": result["feasible"], "runtime_seconds": round(seconds, 3)}
            rows.append(row)
            print(row)
        write_rows(out / "sensitivity_analysis.csv", rows)
        print(f"Saved sensitivity results to {out / 'sensitivity_analysis.csv'}")

if __name__ == "__main__":
    main()
