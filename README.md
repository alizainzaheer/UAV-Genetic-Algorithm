UAV Genetic Optimizer
A modular Python simulation that uses a custom Genetic Algorithm (GA) to plan routes, speed schedules, and sensor activation for a fleet of three UAVs surveying a 50x50 disaster grid with obstacles, hazard pockets, and priority target regions.

Author: Ali Zain (03-134242-012) - BSCS5A

Headline Results
Best evolved mission from main.py with seed 12:

Metric	Value
Fitness	-1.1318
Priority-weighted coverage	0.8467 (84.7 percent)
Target detection	1.000 (5 of 5)
Energy	830.56
Flight time	217.66 s
Hazard risk	12.49
Constraint violations	7.0
Feasible (zero violations)	No at population 50
GA versus random-search baseline with 500 candidates:

Method	Fitness	Coverage	Time	Risk
Random search	-1.256	0.781	427.7 s	68.3
Genetic Algorithm (50x100)	-1.132	0.847	217.7 s	12.5
GA is +9.9 percent fitness, +6.6 pp coverage, -49 percent flight time, -82 percent risk.

Only population 100 reached feasibility with zero violations in the sensitivity study. See results/sensitivity_analysis.csv.

Project Structure
uav-genetic-optimizer/
|-- README.md
|-- requirements.txt
|-- main.py
|-- experiments.py
|-- src/
|   |-- __init__.py
|   |-- environment.py
|   |-- genetic_algorithm.py
|   `-- visualization.py
|-- results/
|   |-- fitness_convergence.png
|   |-- uav_paths.png
|   |-- coverage_heatmap.png
|   |-- coverage_detection.png
|   |-- best_mission_summary.txt
|   |-- metrics.json
|   |-- baseline_comparison.csv
|   `-- sensitivity_analysis.csv
`-- report/
    `-- UAV_GA_Report.pdf
Features
Environment: 50 x 50 grid with obstacles, hazard pockets, priority regions, and 5 target locations; 3 start and end pairs.

Chromosome: hybrid 3 x 6 x 4 array per drone with real-valued waypoints, integer speed genes, and binary sensor genes.

Genetic operators: tournament selection, segment-swap crossover that preserves waypoint-speed-sensor linkage, Gaussian spatial mutation, and elitism.

Constraint handling: nearest-safe waypoint repair plus additive penalty for obstacle crossings, range excess, and time excess.

Outputs: fitness convergence, UAV route map, coverage heatmap, coverage and detection curves, mission summary, and JSON metrics.

Reproducible: fixed seeds for environment (7), GA (12), and random baseline (99).

Requirements
Python 3.10 or newer is recommended.

numpy
matplotlib
Install with:

powershell
python -m pip install -r requirements.txt
Setup (Windows PowerShell)
powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python main.py
If PowerShell blocks activation, run this once in the current terminal:

powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\venv\Scripts\Activate.ps1
Running the Simulation
powershell
python main.py
This produces 5 plots, metrics.json, and best_mission_summary.txt in the results folder. Runtime is about 8 minutes on a typical laptop for the 50 x 100 GA plus evaluation.

Baseline and Sensitivity Experiments
powershell
python experiments.py --mode baseline
python experiments.py --mode sensitivity
python experiments.py --mode all
The first command runs the GA versus random search baseline in about 10 minutes. The second runs 9 GA sensitivity trials in about 20 to 30 minutes. The third runs both in about 30 to 40 minutes.

Results are saved to results/baseline_comparison.csv and results/sensitivity_analysis.csv.

Implementation Map
Requirement	Location
Chromosome representation	Mission dataclass in src/genetic_algorithm.py
Population initialization	GeneticAlgorithm.random_mission
Selection	GeneticAlgorithm.tournament_select
Crossover (segment-swap)	GeneticAlgorithm.crossover
Mutation (spatial and gene)	GeneticAlgorithm.mutate
Elitism and termination	GeneticAlgorithm.run
Constraint repair	GeneticAlgorithm.repair and UAVEnvironment.repair_waypoint
Penalty function	GeneticAlgorithm.evaluate, penalty = min(2.0, violation) * 0.75
Fitness function	GeneticAlgorithm.evaluate
Baseline comparison	experiments.baseline_random_search
Visualizations	src/visualization.py save_visualizations
Fitness Function
The weighted scalar objective is:

F = 0.45 * coverage + 0.25 * detection
  - 0.15 * normalized_energy
  - 0.10 * normalized_time
  - 0.05 * normalized_risk
  - constraint_penalty
coverage is the priority-weighted fraction of grid cells covered by active sensors.

detection is the fraction of 5 targets detected.

normalized_energy, normalized_time, and normalized_risk are raw costs divided by 900, 180, and 180, each clipped at 2.0.

constraint_penalty is 0.75 times min(2, violations). Because the maximum total reward is 0.70, any single violation makes the plan worse than any valid plan.

The weights are modeling choices. See section 2.2 of the report for justification.

Important Modeling Limitations
This is an educational grid simulation, not a flight-certified planner.

Energy model is simplified: d * (1 + 0.12 * speed^2) + 1.5 * sensor.

Hazard map is discretized. Sensor radius is fixed at 3 cells and does not model line of sight.

Segment safety is checked by sampling at 0.5-cell spacing, so corridors narrower than 0.5 may be missed.

Waypoint repair snaps to the nearest free cell. A future version would use A* for the segment between repaired waypoints.

Results use a single seed (12). A 10-seed confidence-interval study is recommended before any real deployment.

Suggested Experiments
Vary one parameter at a time with the environment seed fixed at 7:

Population size: 20, 50, 100

Mutation rate: 0.03, 0.10, 0.20

Tournament size: 2, 3, 5

Record best fitness, coverage, runtime, and feasibility for each trial. The command experiments.py --mode sensitivity runs exactly these nine configurations and writes the results to results/sensitivity_analysis.csv.

Observed finding: population size 100 is the only setting that reached zero constraint violations. Lower populations trade feasibility for coverage.

Reproducibility Note
The reported numbers come from running the committed code with the seeds listed above. If you modify anything, re-run main.py and experiments.py --mode all, then use the new CSV values in any report. Do not mix old and new outputs.