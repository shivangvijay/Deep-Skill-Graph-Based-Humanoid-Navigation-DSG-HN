import argparse
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D


def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize DSG graph using option effect-set medians and goal-region centers."
    )
    parser.add_argument("scene_file", nargs="?", help="Scene XML path.", default="./mac/config/scene/umaze_scene.xml")
    parser.add_argument(
        "models_dir",
        nargs="?",
        help="DSG save directory containing graph_structure.txt and skill files.",
        default="./mac/models/UMaze/dsg_models/run4",
    )
    parser.add_argument("--out", help="Output image path.", default="./mac/logs/UMaze/run4/effect_median_graph.png")
    parser.add_argument("--xmin", type=float, default=-7.0)
    parser.add_argument("--xmax", type=float, default=7.0)
    parser.add_argument("--ymin", type=float, default=-7.0)
    parser.add_argument("--ymax", type=float, default=7.0)
    return parser.parse_args()


def option_color(option_node_id: int):
    return plt.get_cmap("tab20")(option_node_id % 20)


def discover_skills(models_dir: Path):
    out = []
    pat = re.compile(r"skill_(\d+)_classifier\.svm$")
    for p in models_dir.glob("skill_*_classifier.svm"):
        m = pat.match(p.name)
        if m:
            out.append(int(m.group(1)))
    return sorted(set(out))


def parse_graph_option_nodes(models_dir: Path):
    graph_path = models_dir / "graph_structure.txt"
    if not graph_path.exists():
        return []
    lines = graph_path.read_text().splitlines()
    if not lines:
        return []
    n = int(lines[0].strip())
    idx = 1
    nodes = []
    for _ in range(n):
        header = lines[idx].strip().split()
        idx += 1
        node_id = int(header[0])
        is_goal = int(header[1]) != 0
        idx += 3
        if not is_goal:
            nodes.append(node_id)
    return nodes


def build_skill_to_graph_index(skills, models_dir: Path):
    option_nodes = parse_graph_option_nodes(models_dir)
    mapping = {}
    next_node = (max(option_nodes) + 1) if option_nodes else 0
    for i, sid in enumerate(sorted(skills)):
        if i < len(option_nodes):
            mapping[sid] = option_nodes[i]
        else:
            mapping[sid] = next_node
            next_node += 1
    return mapping


def draw_obstacles(ax, scene_file: Path):
    root = ET.parse(scene_file).getroot()
    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if "obs" not in name.lower() and "c" not in name.lower() and "wall" not in name.lower():
            continue
        pos = [float(x) for x in geom.get("pos", "0 0 0").split()]
        size = [float(x) for x in geom.get("size", "0 0 0").split()]
        geom_type = geom.get("type", "box")
        x, y = pos[0], pos[1]
        if geom_type == "box":
            sx, sy = size[0], size[1]
            ax.add_patch(
                patches.Rectangle(
                    (x - sx, y - sy), 2 * sx, 2 * sy, linewidth=1, edgecolor="black", facecolor="gray", alpha=0.45
                )
            )
        elif geom_type == "cylinder":
            r = size[0]
            ax.add_patch(patches.Circle((x, y), r, linewidth=1, edgecolor="black", facecolor="gray", alpha=0.45))


def load_effect_points_2d(path: Path):
    if not path.exists():
        return np.zeros((0, 2), dtype=np.float64)
    lines = path.read_text().strip().splitlines()
    if len(lines) < 2:
        return np.zeros((0, 2), dtype=np.float64)
    pts = []
    for line in lines[1:]:
        toks = line.strip().split()
        if len(toks) < 2:
            continue
        pts.append((float(toks[0]), float(toks[1])))
    if not pts:
        return np.zeros((0, 2), dtype=np.float64)
    return np.array(pts, dtype=np.float64)


def parse_graph_structure(graph_path: Path):
    lines = graph_path.read_text().splitlines()
    n = int(lines[0].strip())
    idx = 1
    nodes = []
    skill_ptr = 0

    for _ in range(n):
        header = lines[idx].strip().split()
        idx += 1
        node_id = int(header[0])
        is_goal = int(header[1]) != 0
        n_children = int(header[2])

        child_tokens = lines[idx].strip().split() if idx < len(lines) else []
        idx += 1
        children = []
        for j in range(0, min(len(child_tokens), n_children * 2), 2):
            children.append(int(child_tokens[j]))

        idx += 1  # skip parent line

        goal_line = lines[idx].strip().split() if idx < len(lines) else []
        idx += 1

        center = None
        epsilon = None
        skill_id = None
        if is_goal and len(goal_line) >= 3:
            epsilon = float(goal_line[0])
            center = (float(goal_line[1]), float(goal_line[2]))
        if not is_goal:
            skill_id = skill_ptr
            skill_ptr += 1

        nodes.append(
            {
                "node_id": node_id,
                "is_goal": is_goal,
                "children": children,
                "center": center,
                "epsilon": epsilon,
                "skill_id": skill_id,
            }
        )
    return nodes


def main():
    args = parse_args()
    scene_file = Path(args.scene_file).resolve()
    models_dir = Path(args.models_dir).resolve()

    # Convenience fallback for common train/mac layout.
    if not models_dir.exists():
        alt_models = (Path.cwd() / "mac/models/UMaze/dsg_models/run4").resolve()
        if alt_models.exists():
            models_dir = alt_models

    graph_path = models_dir / "graph_structure.txt"
    if not graph_path.exists():
        raise FileNotFoundError(f"missing graph structure file: {graph_path}")

    nodes = parse_graph_structure(graph_path)
    skills = discover_skills(models_dir)
    sid_to_opt = build_skill_to_graph_index(skills, models_dir)
    opt_to_sid = {opt: sid for sid, opt in sid_to_opt.items()}

    anchors = {}
    plotted_option_node_ids = []
    fig, ax = plt.subplots(figsize=(8, 8))
    draw_obstacles(ax, scene_file)

    for n in nodes:
        node_id = n["node_id"]
        if n["is_goal"]:
            if n["center"] is None:
                continue
            x, y = n["center"]
            anchors[node_id] = (x, y)
            ax.add_patch(
                patches.Circle((x, y), n["epsilon"], linewidth=1.4, edgecolor="magenta", facecolor="none", alpha=0.9)
            )
            ax.add_patch(
                patches.Circle((x, y), 0.16, linewidth=1.8, edgecolor="magenta", facecolor="white", alpha=0.95)
            )
            ax.text(x, y, f"GR-{node_id}", color="magenta", fontsize=7, ha="center", va="center")
            continue

        sid = opt_to_sid.get(node_id, None)
        if sid is None:
            continue
        effect_path = models_dir / f"skill_{sid}_classifier.svm_effect.txt"
        eff = load_effect_points_2d(effect_path)
        if eff.shape[0] == 0:
            continue
        plotted_option_node_ids.append(node_id)
        color = option_color(node_id)
        mx = float(np.median(eff[:, 0]))
        my = float(np.median(eff[:, 1]))
        anchors[node_id] = (mx, my)
        ax.scatter(eff[:, 0], eff[:, 1], s=6, color=color, alpha=0.2)
        ax.add_patch(
            patches.Circle((mx, my), 0.18, linewidth=1.8, edgecolor=color, facecolor="white", alpha=0.95)
        )
        ax.text(mx, my, f"Opt-{node_id}", color=color, fontsize=7, ha="center", va="center")

    for n in nodes:
        src = n["node_id"]
        if src not in anchors:
            continue
        x0, y0 = anchors[src]
        for dst in n["children"]:
            if dst not in anchors:
                continue
            x1, y1 = anchors[dst]
            ax.plot([x0, x1], [y0, y1], color="black", linewidth=1.0, alpha=0.7)

    legend_handles = [
        Line2D([0], [0], color="black", lw=1.2, label="graph edge"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="white", markeredgecolor="magenta",
               markersize=7, label="goal center"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="white", markeredgecolor="black",
               markersize=7, label="effect-set median"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="gray", alpha=0.4,
               markeredgecolor="none", markersize=6, label="effect samples"),
    ]
    for nid in sorted(set(plotted_option_node_ids)):
        legend_handles.append(
            Line2D([0], [0], color=option_color(nid), lw=2.0, label=f"Opt-{nid}")
        )
    ax.legend(handles=legend_handles, loc="upper right", fontsize=7, frameon=True)

    ax.set_xlim(args.xmin, args.xmax)
    ax.set_ylim(args.ymin, args.ymax)
    ax.set_aspect("equal")
    ax.set_title("DSG Graph: Effect-Set Medians + Goal Centers")
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=170, bbox_inches="tight")
    print(f"saved: {out_path}")


if __name__ == "__main__":
    main()
