"""Plots and saved visual outputs for the UAV GA."""
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

def save_visualizations(env, mission, result, history, output_dir="results"):
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # Fitness convergence
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(history["best"], label="Best fitness")
    ax.plot(history["average"], label="Average fitness")
    ax.set(title="Genetic Algorithm Fitness Convergence", xlabel="Generation", ylabel="Fitness")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "fitness_convergence.png", dpi=180)
    plt.close(fig)

    # Environment and evolved paths
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.imshow(env.priorities, origin="lower", cmap="YlGn", alpha=0.65)
    hazard_mask = np.ma.masked_where(env.hazards <= 0, env.hazards)
    ax.imshow(hazard_mask, origin="lower", cmap="Reds", alpha=0.45, vmin=0, vmax=1)
    obstacle_mask = np.ma.masked_where(~env.obstacles, env.obstacles)
    ax.imshow(obstacle_mask, origin="lower", cmap="Greys", alpha=0.95)
    colors = ["tab:blue", "tab:orange", "tab:purple"]
    for d, path in enumerate(result["paths"]):
        arr = np.array(path)
        ax.plot(arr[:, 0], arr[:, 1], "-o", color=colors[d], label=f"UAV {d+1}", markersize=3)
        ax.scatter(*env.starts[d], marker="s", color=colors[d], s=70)
        ax.scatter(*env.ends[d], marker="X", color=colors[d], s=80)
    tx, ty = zip(*env.targets)
    ax.scatter(tx, ty, marker="*", color="cyan", edgecolors="black", s=120, label="Targets")
    ax.set(xlim=(-0.5, env.size-0.5), ylim=(-0.5, env.size-0.5),
           xlabel="X coordinate", ylabel="Y coordinate",
           title="Evolved UAV Routes (priority map, hazards and obstacles)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.15)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "uav_paths.png", dpi=180)
    plt.close(fig)

    # Coverage heatmap from the best plan
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.imshow(env.priorities, origin="lower", cmap="YlGn", alpha=0.35)
    ax.imshow(result["coverage_map"], origin="lower", cmap="Blues", alpha=0.75)
    ax.set(title="Coverage Heatmap of Best Mission", xlabel="X coordinate", ylabel="Y coordinate")
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out / "coverage_heatmap.png", dpi=180)
    plt.close(fig)

    # Metric curves across generations
    fig, ax = plt.subplots(figsize=(9, 5))
    for key, label in [("coverage", "Coverage"), ("detection", "Detection")]:
        ax.plot(history[key], label=label)
    ax.set(title="Surveillance Performance by Generation", xlabel="Generation", ylabel="Score (0–1)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "coverage_detection.png", dpi=180)
    plt.close(fig)

    # Write a readable summary
    with open(out / "best_mission_summary.txt", "w", encoding="utf-8") as f:
        f.write("BEST UAV MISSION SUMMARY\n")
        f.write("========================\n")
        for key in ["fitness", "coverage", "detection", "energy", "time", "risk", "violations", "feasible"]:
            f.write(f"{key}: {result[key]}\n")
        f.write("\nWaypoints by drone (x, y, speed_gene, sensor_on):\n")
        for d, genes in enumerate(mission.genes):
            f.write(f"UAV {d+1}:\n")
            for row in genes:
                f.write(f"  ({row[0]:.2f}, {row[1]:.2f}, speed={int(row[2])}, sensor={int(row[3])})\n")
