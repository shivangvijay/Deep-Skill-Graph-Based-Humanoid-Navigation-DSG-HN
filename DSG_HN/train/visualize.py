import sys
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import os

scene_file = sys.argv[1]
points_file = sys.argv[2]

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
with open(points_file, 'r') as f:
    for line in f:
        parts = line.strip().split()
        if len(parts) < 3: continue
        skill = int(parts[0])
        x, y = float(parts[1]), float(parts[2])
        ax.scatter(x, y, color=colors[skill % len(colors)], s=5, alpha=0.7)

ax.set_xlim(-7,7)
ax.set_ylim(-7,7)
ax.set_aspect('equal')
ax.set_title('Initiation Sets Visualization')
if not os.path.exists("../results"):
    os.makedirs("../results")
plt.savefig(f"../results/initiation_sets_viz_{time.time()}.png")