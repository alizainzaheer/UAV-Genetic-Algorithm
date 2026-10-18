UAV Genetic Optimizer
A modular Python simulation that uses a custom Genetic Algorithm (GA) to plan routes, speed schedules, and sensor activation for a fleet of three UAVs surveying a 50×50 disaster grid with obstacles, hazard pockets, and priority target regions.

Author: Ali Zain (03-134242-012) - BSCS5A

Headline Results
Best evolved mission (main.py, seed = 12):

Metric	Value
Fitness	-1.1318
Priority-weighted coverage	0.8467 (84.7 %)
Target detection	1.000 (5/5)
Energy	830.56
Flight time	217.66 s
Hazard risk	12.49
Constraint violations	7.0
Feasible (zero violations)	No at population 50
GA vs random-search baseline (500 candidates):

Method	Fitness	Coverage	Time	Risk
Random search	-1.256	0.781	427.7 s	68.3
Genetic Algorithm (50x100)	-1.132	0.847	217.7 s	12.5
GA is +9.9 % fitness, +6.6 pp coverage, -49 % flight time, -82 % risk.
Only population 100 reached feasibility (zero violations) in the sensitivity study — see results/sensitivity_analysis.csv.

Project Structure
uav-genetic-optimizer/
├── README.md
├── requirements.txt
├── main.py                       # Run the GA and save plots + metrics
├── experiments.py                # Baseline + sensitivity experiments
├── src/
│   ├── __init__.py
│   ├── environment.py            # 50x50 grid, obstacles, hazards, priorities
│   ├── genetic_algorithm.py      # Mission, GeneticAlgorithm class
│   └── visualization.py          # All matplotlib outputs
├── results/
│   ├── fitness_convergence.png
│   ├── uav_paths.png
│   ├── coverage_heatmap.png
│   ├── coverage_detection.png
│   ├── best_mission_summary.txt
│   ├── metrics.json
│   ├── baseline_comparison.csv
│   └── sensitivity_analysis.csv
└── report/
    └── UAV_GA_Report.pdf

    Features
Environment: 50 x 50 grid with obstacles, hazard pockets, priority regions, and 5 target locations; 3 start/end pairs.

Chromosome: hybrid 3 x 6 x 4 array per drone — real-valued waypoints, integer speed genes, binary sensor genes.

Genetic operators: tournament selection, segment-swap crossover (preserves waypoint–speed–sensor linkage), Gaussian spatial mutation, elitism.

Constraint handling: nearest-safe waypoint repair + additive penalty for obstacle crossings, range excess, and time excess.

Outputs: fitness convergence, UAV route map, coverage heatmap, coverage/detection curves, mission summary, JSON metrics.

Reproducible: fixed seeds for environment (7), GA (12), and random baseline (99).

Running the Simulation
python main.py
Produces 5 plots + metrics.json + best_mission_summary.txt in results/.
Runtime: about 8 minutes on a typical laptop (50 x 100 GA + evaluation).

Baseline and Sensitivity Experiments
python experiments.py --mode baseline      # ~10 min  (GA + 500 random)
python experiments.py --mode sensitivity   # ~20-30 min (9 GA runs)
python experiments.py --mode all           # both, ~30-40 min
Saves results/baseline_comparison.csv and results/sensitivity_analysis.csv.

Implementation Map (for graders)
Requirement	Location
Chromosome representation	Mission dataclass — src/genetic_algorithm.py
Population initialization	GeneticAlgorithm.random_mission()
Selection	GeneticAlgorithm.tournament_select()
Crossover (segment-swap)	GeneticAlgorithm.crossover()
Mutation (spatial + gene)	GeneticAlgorithm.mutate()
Elitism / termination	GeneticAlgorithm.run()
Constraint repair	GeneticAlgorithm.repair(), UAVEnvironment.repair_waypoint()
Penalty function	GeneticAlgorithm.evaluate() — penalty = min(2.0, violation) * 0.75
Fitness function	GeneticAlgorithm.evaluate()
Baseline comparison	experiments.baseline_random_search()
Visualizations	src/visualization.py::save_visualizations()
Fitness Function
Weighted scalar objective:

F = 0.45*coverage + 0.25*detection
  - 0.15*normalized_energy
  - 0.10*normalized_time
  - 0.05*normalized_risk
  - constraint_penalty
coverage — priority-weighted fraction of grid covered by active sensors

detection — fraction of 5 targets detected

normalized_energy, normalized_time, normalized_risk — raw costs divided by 900, 180, 180, each clipped at 2.0

constraint_penalty — 0.75 * min(2, violations); dominance over the 0.70 max reward means any single violation is worse than any valid plan

Weights are modeling choices; see the report (section 2.2) for justification.

Important Modeling Limitations
This is an educational grid simulation, not a flight-certified planner.

Simplified energy model: d * (1 + 0.12 * speed^2) + 1.5 * sensor.

Discretized hazard map; sensor radius fixed at 3 cells with no line-of-sight modeling.

Segment safety is checked by sampling at 0.5-cell spacing — corridors narrower than 0.5 may be missed.

Waypoint repair snaps to the nearest free cell; a future version would use A* for the segment between repaired waypoints.

Results use a single seed (12). A 10-seed confidence-interval study is recommended before any real deployment.

Suggested Experiments
Vary one parameter at a time with the environment seed fixed at 7:

Population size: 20, 50, 100

Mutation rate: 0.03, 0.10, 0.20

Tournament size: 2, 3, 5

Record best fitness, coverage, runtime, and feasibility for each trial.
experiments.py --mode sensitivity runs exactly these nine configurations and writes the results to results/sensitivity_analysis.csv.

Observed finding: population size 100 is the only setting that reached zero constraint violations; lower populations trade feasibility for coverage.

Reproducibility Note
The reported numbers come from running the committed code with the seeds listed above. If you modify anything, re-run main.py and experiments.py --mode all, and use the new CSV values in any report — do not mix old and new outputs.