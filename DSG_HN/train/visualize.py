"""
visualize.py — produce two PNG images from a DSG/DSC run.

Usage:
    python visualize.py <scene.xml> <init_sets.txt> [mpc_viz.log]

Outputs:
    ../initiation_sets_viz.png   — skill initiation sets scatter plot
    ../mpc_viz.png               — MPC expansion rollouts (only if log provided)

init_sets.txt format:  skill_idx x y
mpc_viz.log format:
    EXPANSION <color_idx> start_x start_y goal_x goal_y
    STEP      <color_idx> x y
"""

import sys
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from collections import defaultdict

COLORS = [
    "#e6194b",  # red
    "#c47800",  # brown-amber
    "#9a34c8",  # purple
    "#e875b0",  # pink
    "#3cb44b",  # green
    "#4363d8",  # blue
    "#f58231",  # orange
    "#42d4f4",  # cyan
    "#ffe119",  # yellow
    "#800000",  # maroon
]


def color_for(idx):
    return COLORS[idx % len(COLORS)]


# ── shared obstacle drawing ───────────────────────────────────────────────────

def draw_obstacles(ax, scene_file):
    tree = ET.parse(scene_file)
    root = tree.getroot()
    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if not any(k in name.lower() for k in ("obs", "wall", "c")):
            continue
        pos  = [float(v) for v in geom.get("pos",  "0 0 0").split()]
        size = [float(v) for v in geom.get("size", "0 0 0").split()]
        x, y = pos[0], pos[1]
        gtype = geom.get("type", "box")
        if gtype == "box":
            sx, sy = size[0], size[1]
            ax.add_patch(patches.Rectangle(
                (x - sx, y - sy), 2 * sx, 2 * sy,
                linewidth=1, edgecolor="black", facecolor="gray", alpha=0.5, zorder=2,
            ))
        elif gtype == "cylinder":
            ax.add_patch(patches.Circle(
                (x, y), size[0],
                linewidth=1, edgecolor="black", facecolor="gray", alpha=0.5, zorder=2,
            ))


# ── plot 1: initiation sets ───────────────────────────────────────────────────

def plot_initiation_sets(scene_file, points_file):
    fig, ax = plt.subplots()
    draw_obstacles(ax, scene_file)

    with open(points_file) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 3:
                continue
            skill = int(parts[0])
            x, y  = float(parts[1]), float(parts[2])
            ax.scatter(x, y, color=color_for(skill), s=5, alpha=0.7, zorder=3)

    ax.set_xlim(-7, 7)
    ax.set_ylim(-7, 7)
    ax.set_aspect("equal")
    ax.set_title("Initiation Sets Visualization")
    plt.savefig("../initiation_sets_viz.png")
    plt.close(fig)
    print("Saved → ../initiation_sets_viz.png")


# ── plot 2: MPC rollouts ──────────────────────────────────────────────────────

def parse_mpc_log(log_file):
    expansions = {}          # color_idx -> (sx, sy, gx, gy)
    steps = defaultdict(list)  # color_idx -> [(x, y), ...]
    with open(log_file) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "EXPANSION" and len(parts) == 6:
                c = int(parts[1])
                expansions[c] = (float(parts[2]), float(parts[3]),
                                  float(parts[4]), float(parts[5]))
            elif parts[0] == "STEP" and len(parts) == 4:
                c = int(parts[1])
                steps[c].append((float(parts[2]), float(parts[3])))
    return expansions, steps


def plot_mpc_rollouts(scene_file, log_file):
    expansions, steps = parse_mpc_log(log_file)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal")
    draw_obstacles(ax, scene_file)

    for color_idx, (sx, sy, gx, gy) in expansions.items():
        c = color_for(color_idx)

        # Robot start position
        ax.scatter(sx, sy, color="black", s=40, zorder=5)

        # MPC trajectory
        traj = steps.get(color_idx, [])
        if traj:
            xs = [sx] + [p[0] for p in traj]
            ys = [sy] + [p[1] for p in traj]
            ax.plot(xs, ys, color=c, linewidth=1.5, alpha=0.85, zorder=3)

        # s_rand goal as star
        ax.scatter(gx, gy, marker="*", s=200, color=c, edgecolors="none", zorder=4)

    # Auto-scale with padding
    all_x = [v for sx, sy, gx, gy in expansions.values() for v in (sx, gx)]
    all_y = [v for sx, sy, gx, gy in expansions.values() for v in (sy, gy)]
    for pts in steps.values():
        all_x += [p[0] for p in pts]
        all_y += [p[1] for p in pts]

    pad = 1.0
    if all_x:
        ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
        ax.set_ylim(min(all_y) - pad, max(all_y) + pad)
    else:
        ax.set_xlim(-7, 7)
        ax.set_ylim(-7, 7)

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("MPC Expansion Rollouts")
    plt.tight_layout()
    plt.savefig("../mpc_viz.png", dpi=150)
    plt.close(fig)
    print("Saved → ../mpc_viz.png")


# ── entry point ───────────────────────────────────────────────────────────────

if len(sys.argv) < 3:
    print(__doc__)
    sys.exit(1)

scene_file  = sys.argv[1]
points_file = sys.argv[2]
mpc_log     = sys.argv[3] if len(sys.argv) > 3 else None

plot_initiation_sets(scene_file, points_file)
if mpc_log:
    plot_mpc_rollouts(scene_file, mpc_log)
