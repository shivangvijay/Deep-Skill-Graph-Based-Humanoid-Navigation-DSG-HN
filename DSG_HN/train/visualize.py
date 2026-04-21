import sys
import re
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import os

# Toggle for drawing goal regions (GR circles/labels)
SHOW_GOAL_REGIONS = True

scene_file = sys.argv[1]
points_file = sys.argv[2]
graph_file = sys.argv[3] if len(sys.argv) > 3 else None

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

# Optional direct goal-region format in points file:
# GR <id> <x> <y> <eps>
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
        skill = int(parts[0])
        x, y = float(parts[1]), float(parts[2])
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
if not os.path.exists("../results"):
    os.makedirs("../results")
plt.savefig(f"../results/initiation_sets_viz_{time.time()}.png")
