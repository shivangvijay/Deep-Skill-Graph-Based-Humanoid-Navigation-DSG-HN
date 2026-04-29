#!/usr/bin/env python3
import argparse
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import xml.etree.ElementTree as ET
from matplotlib.lines import Line2D


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize DSG graph using option effect-set medians and goal-region centers."
    )
    parser.add_argument(
        "scene_file",
        nargs="?",
        default="./mac/config/scene/umaze_scene.xml",
        help="Scene XML path.",
    )
    parser.add_argument(
        "models_dir",
        nargs="?",
        default="./mac/models/UMaze/dsg_models/run2",
        help="DSG save directory containing graph_structure.txt and skill files.",
    )
    parser.add_argument("--out", default="./mac/logs/UMaze/run2/effect_median_graph.png", help="Output image path.")
    parser.add_argument("--xmin", type=float, default=-7.0)
    parser.add_argument("--xmax", type=float, default=7.0)
    parser.add_argument("--ymin", type=float, default=-7.0)
    parser.add_argument("--ymax", type=float, default=7.0)
    parser.add_argument(
        "--points-file",
        default="/tmp/init_sets.txt",
        help="Optional GR overlay file: lines 'GR <id> <x> <y> <eps>'. If present, used for goal-region centers.",
    )
    return parser.parse_args()


def option_color(skill_id: int):
    return plt.get_cmap("tab20")(skill_id % 20)


def discover_skills(models_dir: Path) -> List[int]:
    out = []
    pat = re.compile(r"skill_(\d+)_classifier\.svm$")
    for p in models_dir.glob("skill_*_classifier.svm"):
        m = pat.match(p.name)
        if m:
            out.append(int(m.group(1)))
    return sorted(set(out))


def parse_graph_option_nodes(models_dir: Path) -> List[int]:
    graph_path = models_dir / "graph_structure.txt"
    if not graph_path.exists():
        return []
    lines = graph_path.read_text().splitlines()
    if not lines:
        return []

    n = int(lines[0].strip())
    idx = 1
    nodes: List[int] = []
    for _ in range(n):
        header = lines[idx].strip().split()
        idx += 1
        node_id = int(header[0])
        is_goal = int(header[1]) != 0
        idx += 3  # children, parents, payload lines
        if not is_goal:
            nodes.append(node_id)
    return nodes


def parse_graph_option_skill_mapping(models_dir: Path) -> Dict[int, int]:
    graph_path = models_dir / "graph_structure.txt"
    if not graph_path.exists():
        return {}
    lines = graph_path.read_text().splitlines()
    if not lines:
        return {}

    n = int(lines[0].strip())
    idx = 1
    mapping: Dict[int, int] = {}
    for _ in range(n):
        header = lines[idx].strip().split()
        idx += 1
        node_id = int(header[0])
        is_goal = int(header[1]) != 0
        idx += 2  # children + parents
        payload = lines[idx].strip() if idx < len(lines) else ""
        idx += 1
        if is_goal:
            continue
        toks = payload.split()
        if len(toks) == 2 and toks[0] == "skill_id":
            try:
                skill_id = int(toks[1])
                mapping[skill_id] = node_id
            except ValueError:
                pass
    return mapping


def build_skill_to_graph_index(skills: List[int], models_dir: Path) -> Dict[int, int]:
    explicit = parse_graph_option_skill_mapping(models_dir)
    if explicit:
        return {sid: explicit[sid] for sid in sorted(skills) if sid in explicit}

    option_nodes = parse_graph_option_nodes(models_dir)
    mapping: Dict[int, int] = {}
    for i, sid in enumerate(sorted(skills)):
        if i < len(option_nodes):
            mapping[sid] = option_nodes[i]
    return mapping


def draw_obstacles(ax, scene_path: Path) -> None:
    if scene_path is None or not scene_path.exists():
        return

    root = ET.parse(scene_path).getroot()
    for geom in root.iter("geom"):
        name = geom.get("name", "")
        low = name.lower()
        if "obs" not in low and "wall" not in low and "c" not in low:
            continue
        pos = [float(x) for x in geom.get("pos", "0 0 0").split()]
        size = [float(x) for x in geom.get("size", "0 0 0").split()]
        gtype = geom.get("type", "box")
        x, y = pos[0], pos[1]
        if gtype == "box":
            sx, sy = size[0], size[1]
            ax.add_patch(
                patches.Rectangle(
                    (x - sx, y - sy),
                    2 * sx,
                    2 * sy,
                    linewidth=1,
                    edgecolor="black",
                    facecolor="gray",
                    alpha=0.35,
                )
            )
        elif gtype == "cylinder":
            r = size[0]
            ax.add_patch(
                patches.Circle(
                    (x, y),
                    r,
                    linewidth=1,
                    edgecolor="black",
                    facecolor="gray",
                    alpha=0.35,
                )
            )


def load_effect_points_2d(path: Path) -> np.ndarray:
    if not path.exists():
        return np.zeros((0, 2), dtype=np.float64)
    lines = path.read_text().strip().splitlines()
    if not lines:
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


def parse_graph_nodes(graph_path: Path) -> List[Dict]:
    lines = graph_path.read_text().splitlines()
    n = int(lines[0].strip())
    idx = 1
    nodes: List[Dict] = []

    for _ in range(n):
        header = lines[idx].strip().split()
        idx += 1

        node_id = int(header[0])
        is_goal = int(header[1]) != 0
        n_children = int(header[2])

        child_tokens = lines[idx].strip().split() if idx < len(lines) else []
        idx += 1
        children: List[int] = []
        for j in range(0, min(len(child_tokens), n_children * 2), 2):
            children.append(int(child_tokens[j]))

        idx += 1  # skip parent line

        payload = lines[idx].strip().split() if idx < len(lines) else []
        idx += 1

        center: Optional[Tuple[float, float]] = None
        epsilon: Optional[float] = None
        if is_goal and len(payload) >= 3:
            epsilon = float(payload[0])
            center = (float(payload[1]), float(payload[2]))

        nodes.append(
            {
                "node_id": node_id,
                "is_goal": is_goal,
                "children": children,
                "center": center,
                "epsilon": epsilon,
            }
        )

    return nodes


def parse_goal_regions_from_points(points_file: Optional[Path]) -> List[Tuple[int, float, float, float]]:
    if points_file is None or not points_file.exists():
        return []
    goal_regions: List[Tuple[int, float, float, float]] = []
    with points_file.open("r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 5 and parts[0] == "GR":
                try:
                    gr_id = int(parts[1])
                    x, y, eps = float(parts[2]), float(parts[3]), float(parts[4])
                    goal_regions.append((gr_id, x, y, eps))
                except ValueError:
                    continue
    return goal_regions


def main() -> None:
    args = parse_args()
    scene_file = Path(args.scene_file).resolve()
    models_dir = Path(args.models_dir).resolve()

    graph_path = models_dir / "graph_structure.txt"
    if not graph_path.exists():
        raise FileNotFoundError(f"missing graph structure file: {graph_path}")

    nodes = parse_graph_nodes(graph_path)
    skills = discover_skills(models_dir)
    sid_to_opt = build_skill_to_graph_index(skills, models_dir)
    explicit_map = parse_graph_option_skill_mapping(models_dir)
    points_file = Path(args.points_file).resolve() if args.points_file else None
    goal_regions = parse_goal_regions_from_points(points_file)

    fig, ax = plt.subplots(figsize=(8, 8))
    draw_obstacles(ax, scene_file)

    anchors: Dict[int, Tuple[float, float]] = {}
    plotted_option_node_ids: List[int] = []
    plotted_goal_ids: List[int] = []

    # Plot goal regions. Prefer the same points-file source used by visualize_initiation_set.py.
    if goal_regions:
        for gr_id, x, y, eps in goal_regions:
            anchors[gr_id] = (x, y)
            plotted_goal_ids.append(gr_id)
            ax.add_patch(
                patches.Circle((x, y), eps, linewidth=1.4, edgecolor="magenta", facecolor="none", alpha=0.9)
            )
            ax.add_patch(
                patches.Circle((x, y), 0.14, linewidth=1.6, edgecolor="magenta", facecolor="white", alpha=0.95)
            )
            ax.text(x, y, f"GR-{gr_id}", color="magenta", fontsize=7, ha="center", va="center")
    else:
        for n in nodes:
            if not n["is_goal"] or n["center"] is None:
                continue
            node_id = n["node_id"]
            x, y = n["center"]
            eps = float(n["epsilon"] or 0.0)
            anchors[node_id] = (x, y)
            plotted_goal_ids.append(node_id)
            ax.add_patch(
                patches.Circle((x, y), eps, linewidth=1.4, edgecolor="magenta", facecolor="none", alpha=0.9)
            )
            ax.add_patch(
                patches.Circle((x, y), 0.14, linewidth=1.6, edgecolor="magenta", facecolor="white", alpha=0.95)
            )
            ax.text(x, y, f"GR-{node_id}", color="magenta", fontsize=7, ha="center", va="center")

    # Plot effect-set medians for options (only mature options in the graph).
    mature_skills = [sid for sid in skills if sid in sid_to_opt]
    for sid in mature_skills:
        node_id = sid_to_opt[sid]
        effect_path = models_dir / f"skill_{sid}_classifier.svm_effect.txt"
        eff = load_effect_points_2d(effect_path)
        if eff.shape[0] == 0:
            continue

        color = option_color(node_id)
        mx = float(np.median(eff[:, 0]))
        my = float(np.median(eff[:, 1]))

        anchors[node_id] = (mx, my)
        plotted_option_node_ids.append(node_id)

        ax.scatter(eff[:, 0], eff[:, 1], s=6, color=color, alpha=0.22)
        ax.add_patch(
            patches.Circle((mx, my), 0.18, linewidth=1.8, edgecolor=color, facecolor="white", alpha=0.95)
        )
        ax.text(mx, my, f"Opt-{node_id}", color=color, fontsize=7, ha="center", va="center")

    # Plot graph edges using anchors for whichever nodes are visible.
    edge_count = 0
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
            edge_count += 1

    legend_handles = [
        Line2D([0], [0], color="black", lw=1.2, label="graph edge"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="white", markeredgecolor="magenta", markersize=7, label="goal center"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="white", markeredgecolor="black", markersize=7, label="effect-set median"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="gray", alpha=0.4, markeredgecolor="none", markersize=6, label="effect samples"),
    ]
    for nid in sorted(set(plotted_option_node_ids)):
        legend_handles.append(Line2D([0], [0], color=option_color(nid), lw=2.0, label=f"Opt-{nid}"))
    ax.legend(handles=legend_handles, loc="upper left", bbox_to_anchor=(1.0, 1.0), fontsize=7, frameon=True)

    ax.set_xlim(args.xmin, args.xmax)
    ax.set_ylim(args.ymin, args.ymax)
    ax.set_aspect("equal")
    ax.set_title("DSG Graph: Effect-Set Medians + Goal Centers")
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=170, bbox_inches="tight")

    print(f"discovered skills: {len(skills)}")
    print(f"mature options plotted: {len(mature_skills)}")
    print(f"models_dir: {models_dir}")
    print(f"mapping mode: {'explicit skill_id' if explicit_map else 'legacy positional'}")
    if explicit_map:
        print(f"explicit mapped options: {len(explicit_map)}")
    print(f"goal regions parsed: {sum(1 for n in nodes if n['is_goal'])}")
    print(f"goal-region source: {'points-file' if goal_regions else 'graph_structure'}")
    print(f"goal regions plotted: {len(plotted_goal_ids)}")
    print(f"options plotted: {len(plotted_option_node_ids)}")
    print(f"edges drawn: {edge_count}")
    print(f"saved: {out_path}")


if __name__ == "__main__":
    main()
