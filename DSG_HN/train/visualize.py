import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.patches as patches
import matplotlib.pyplot as plt

# Toggle for drawing goal regions (GR circles/labels)
SHOW_GOAL_REGIONS = True

def parse_args():
    parser = argparse.ArgumentParser(description="Visualize initiation-set records.")
    parser.add_argument("scene_file", help="Scene XML path.")
    parser.add_argument("points_file", nargs="?", default=None,
                        help="Optional points file (legacy format; also supports GR lines).")
    parser.add_argument("graph_file", nargs="?", default=None,
                        help="Optional DSG log file for fallback GR parsing.")
    parser.add_argument("--models-dir", default=None,
                        help="If provided, plot points from skill_*_classifier.svm_positives.txt "
                             "(same records used by visualize_initiation_set.py).")
    return parser.parse_args()


def load_points_from_models(models_dir: Path):
    points = []
    pat = re.compile(r"skill_(\d+)_classifier\.svm_positives\.txt$")
    if not models_dir.exists():
        return points

    for p in sorted(models_dir.glob("skill_*_classifier.svm_positives.txt")):
        m = pat.match(p.name)
        if not m:
            continue
        skill = int(m.group(1))
        try:
            lines = p.read_text().strip().splitlines()
            if not lines:
                continue
            for line in lines[1:]:
                toks = line.strip().split()
                if len(toks) < 2:
                    continue
                x, y = float(toks[0]), float(toks[1])
                points.append((skill, x, y))
        except Exception:
            continue
    return points


def load_points_and_goal_regions(points_file: str, use_points: bool):
    points = []
    goal_regions = []
    if points_file is None:
        return points, goal_regions

    with open(points_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 3:
                continue
            if parts[0] == "GR" and len(parts) >= 5:
                gr_id = int(parts[1])
                x, y, eps = float(parts[2]), float(parts[3]), float(parts[4])
                goal_regions.append((gr_id, x, y, eps))
                continue
            if use_points:
                skill = int(parts[0])
                x, y = float(parts[1]), float(parts[2])
                points.append((skill, x, y))
    return points, goal_regions


args = parse_args()
scene_file = args.scene_file
points_file = args.points_file
graph_file = args.graph_file

tree = ET.parse(scene_file)
root = tree.getroot()

fig, ax = plt.subplots()

for geom in root.iter('geom'):
    name = geom.get('name', '')
    if 'obs' in name.lower() or 'c' in name.lower() or 'wall' in name.lower():
        pos_str = geom.get('pos', '0 0 0')
        size_str = geom.get('size', '0 0 0')
        type_ = geom.get('type', 'box')
        pos = [float(x) for x in pos_str.split()]
        size = [float(x) for x in size_str.split()]
        x, y = pos[0], pos[1]
        if type_ == 'box':
            sx, sy = size[0], size[1]
            rect = patches.Rectangle((x-sx, y-sy), 2*sx, 2*sy, linewidth=1, edgecolor='black', facecolor='gray', alpha=0.5)
            ax.add_patch(rect)
        elif type_ == 'cylinder':
            r = size[0]
            circle = patches.Circle((x, y), r, linewidth=1, edgecolor='black', facecolor='gray', alpha=0.5)
            ax.add_patch(circle)

colors = ['red', 'green', 'blue', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
goal_regions = []

use_legacy_points = args.models_dir is None
all_points, goal_regions = load_points_and_goal_regions(points_file, use_legacy_points)
if args.models_dir is not None:
    all_points = load_points_from_models(Path(args.models_dir))

for skill, x, y in all_points:
    ax.scatter(x, y, color=colors[skill % len(colors)], s=5, alpha=0.7)

# Optional fallback: parse latest Graph Structure block from a DSG log file.
if graph_file is not None and not goal_regions:
    gr_re = re.compile(r"GR-(\d+).*\| center=\(x=([\-0-9.eE+]+), y=([\-0-9.eE+]+)\) eps=([\-0-9.eE+]+)")
    latest_block = []
    in_graph_block = False

    with open(graph_file, "r") as f:
        for raw in f:
            line = raw.rstrip("\n")
            if line.startswith("=== Graph Structure ==="):
                in_graph_block = True
                latest_block = []
                continue
            if in_graph_block and line.startswith("["):
                in_graph_block = False
                continue
            if in_graph_block:
                m = gr_re.search(line)
                if m:
                    gr_id = int(m.group(1))
                    x = float(m.group(2))
                    y = float(m.group(3))
                    eps = float(m.group(4))
                    latest_block.append((gr_id, x, y, eps))

    goal_regions = latest_block

if SHOW_GOAL_REGIONS:
    for gr_id, x, y, eps in goal_regions:
        circle = patches.Circle((x, y), eps, linewidth=1.5, edgecolor='magenta', facecolor='none', alpha=0.9)
        ax.add_patch(circle)
        ax.text(x, y, f"GR-{gr_id}", color='magenta', fontsize=7, ha='center', va='center')

ax.set_xlim(-7,7)
ax.set_ylim(-7,7)
ax.set_aspect('equal')
ax.set_title('Initiation Sets Visualization')
# if not os.path.exists("../results"):
#     os.makedirs("../results")
plt.savefig(f"../logs/initiation_sets_viz.png")
