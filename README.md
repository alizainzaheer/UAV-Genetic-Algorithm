# UAV Genetic Algorithm — Multi-Drone Disaster Coverage Planning

An Artificial Intelligence project that uses a Genetic Algorithm (GA)
to plan flight routes for three UAVs (drones) performing a search-and-rescue
mission over a 50x50 grid containing no-fly zones, hazard areas, and
high-priority disaster targets.

## Problem Statement

Three drones must fly from fixed starting positions to search a disaster
zone. The mission must simultaneously:

1. Maximize priority-weighted area coverage — sweep high-priority cells
2. Maximize target detection — visit priority targets with sensors on
3. Minimize energy consumption — save battery
4. Minimize flight time — reach survivors fast
5. Minimize hazard exposure — avoid danger zones

Balancing five competing objectives across three coordinated drones is a
combinatorial optimization problem. A Genetic Algorithm is well-suited
because it explores many candidate route plans in parallel and improves
them generation by generation without needing a differentiable objective.

## Chromosome Representation

Each chromosome encodes a complete 3-drone mission as a hybrid array of
shape 3 x 6 x 4 (three drones, six waypoints each, four genes per waypoint):

| Field | Description |
|-------|-------------|
| x, y | Real-valued waypoint coordinates in [0, 49] |
| speed | Integer gene in {0, 1, 2} mapping to {1.0, 2.0, 3.0} m/s |
| sensor | Binary gene: 0 = sensor off, 1 = sensor on |

Speed drives both flight time (d / speed) and energy
(d * (1 + 0.12 * speed^2)). The sensor gene costs extra energy and time
per segment but drives coverage and detection. Waypoints that fall inside
obstacles are repaired to the nearest safe cell before evaluation.

## Fitness Function
F = 0.45C + 0.25D - 0.15E - 0.10T - 0.05R - 0.75min(2, V)


Where:

- C = priority-weighted fraction of grid covered by active sensors
- D = fraction of the 5 targets detected
- E = normalized energy (energy / 900, clipped at 2.0)
- T = normalized flight time (time / 180, clipped at 2.0)
- R = normalized hazard risk (risk / 180, clipped at 2.0)
- V = constraint violation count (obstacle crossings + range excess + time excess)

The penalty weight of 0.75 is chosen so that a single violation is worse
than the maximum possible coverage + detection reward of 0.70. The cap at
2.0 prevents one broken chromosome from collapsing the average fitness.

## Genetic Operators

| Operator | Implementation |
|----------|----------------|
| Selection | Tournament selection (size = 3) |
| Crossover | Segment-swap on a random drone's contiguous waypoint block (p = 0.80); whole-drone swap (p = 0.25) |
| Mutation | Per-gene with rate 0.10 (Gaussian spatial noise on x,y; resample speed; flip sensor) |
| Elitism | Top 2 chromosomes copied unchanged to next generation |
| Termination | 100 generations in main.py; 50 in experiments |

## Installation

Requires Python 3.10+ and numpy, matplotlib.

## How to Run

Generate the best deployment plan and save graphs:

Run the random-search baseline comparison only:

Run the hyperparameter sensitivity experiments only:

Run both experiments:

## Outputs

After running main.py, the results/ folder contains:

| File | Description |
|------|-------------|
| fitness_convergence.png | Best and average fitness per generation |
| uav_paths.png | Drone trajectories with obstacles, hazards, targets |
| coverage_heatmap.png | Which grid cells were covered by sensors |
| coverage_detection.png | Coverage and detection score across generations |
| best_mission_summary.txt | Full waypoint list and metric summary |
| metrics.json | Final metric dictionary |

After running experiments.py, the results/ folder additionally contains:

| File | Description |
|------|-------------|
| baseline_comparison.csv | GA vs random-search comparison |
| sensitivity_analysis.csv | All 9 hyperparameter trials |

## Experimental Findings

### Hyperparameter sensitivity (single seed = 12, 50 generations)

| Parameter | Value | Fitness | Coverage | Violations | Feasible | Runtime |
|-----------|-------|---------|----------|------------|----------|---------|
| Population size | 20 | -1.141 | 0.822 | 7 | No | 107s |
| | 50 | -1.156 | 0.856 | 7 | No | 265s |
| | 100 | +0.330 | 0.734 | 0 | Yes | 426s |
| Mutation rate | 0.03 | -1.109 | 0.841 | 5 | No | 236s |
| | 0.10 | -1.156 | 0.856 | 7 | No | 253s |
| | 0.20 | -1.160 | 0.845 | 9 | No | 236s |
| Tournament size | 2 | -1.152 | 0.800 | 4 | No | 243s |
| | 3 | -1.156 | 0.856 | 7 | No | 253s |
| | 5 | -1.118 | 0.855 | 7 | No | 235s |

Key finding: Population size 100 is the only setting that reached zero
constraint violations. Smaller populations trade feasibility for coverage.

### GA vs Random Search (500 candidates vs 50x100 GA)

| Method | Best fitness | Coverage | Time | Risk | Runtime |
|--------|--------------|----------|------|------|---------|
| Genetic Algorithm | -1.132 | 0.847 | 217.7s | 12.5 | 486.4s |
| Random search | -1.256 | 0.781 | 427.7s | 68.3 | 49.7s |
| Improvement | +9.9% | +6.6 pp | -49% | -82% | — |

Discussion: The GA solution is qualitatively different from the best
random plan, not just a better-scoring one. Random search's best plan
spent 427.7 s of flight time and incurred risk 68.3 because independent
sampling has no mechanism to reward an ordered sequence of waypoints. The
GA finds an ordered corridor: its UAV 3 route threads between two hazard
blocks, which explains most of the 82% risk reduction. The GA also halves
flight time while keeping energy essentially identical (830.9 vs 830.6),
showing that better spatial arrangement of waypoints comes for free once
crossover preserves route ordering.

## Limitations

- Grid is static; real UAVs face dynamic weather and moving hazards.
- Hazard map is discretized. Sensor radius is fixed at 3 cells and does
  not model line of sight or obstacle shadows.
- Segment safety is checked by sampling at 0.5-cell spacing, so corridors
  narrower than 0.5 may be missed.
- Waypoint repair snaps to the nearest free cell; a future version would
  use A* for the segment between repaired waypoints.
- Results use a single seed (12). A 10-seed confidence-interval study is
  recommended before any real deployment.

## Project Structure
UAV_Genetic_Optimizer/
├── results/
│   ├── baseline_comparison.csv
│   ├── best_mission_summary.txt
│   ├── coverage_detection.png
│   ├── coverage_heatmap.png
│   ├── fitness_convergence.png
│   ├── metrics.json
│   ├── sensitivity_analysis.csv
│   └── uav_paths.png
├── src/
│   ├── environment.py
│   ├── genetic_algorithm.py
│   └── visualization.py
├── .gitignore
├── experiments.py
├── main.py
├── README.md
└── requirements.txt

## Reproducibility Note

The numbers reported above come from running the committed code with the
seeds listed in the source: environment seed = 7, GA seed = 12, random
baseline seed = 99. If you modify any parameter, re-run main.py and
experiments.py --mode all, and use the new CSV values in any report.

## Author

Ali Zain — 03-134242-012 — BSCS-5A
Bahria University, Lahore Campus
Submitted to: Dr. Iram Noreen