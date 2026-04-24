#!/usr/bin/env python3
import argparse
import math
import re
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import matplotlib.pyplot as plt
import numpy as np
import xml.etree.ElementTree as ET
from matplotlib import patches
from matplotlib.colors import TwoSlopeNorm


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize initiation-set classifier boundaries from saved libsvm models."
    )
    parser.add_argument(
        "--models-dir",
        default="mac/models/dsg_models",
        help="Directory containing skill_<id>_classifier.svm files.",
    )
    parser.add_argument(
        "--scene",
        default=None,
        help="Optional scene XML path. If omitted, script tries <models-dir>/scene_file.txt.",
    )
    parser.add_argument(
        "--skills",
        nargs="*",
        type=int,
        default=None,
        help="Optional list of skill ids to plot. Default: all skills with classifier files.",
    )
    parser.add_argument("--xmin", type=float, default=-7.0)
    parser.add_argument("--xmax", type=float, default=7.0)
    parser.add_argument("--ymin", type=float, default=-7.0)
    parser.add_argument("--ymax", type=float, default=7.0)
    parser.add_argument(
        "--grid",
        type=int,
        default=220,
        help="Grid resolution per axis for boundary evaluation.",
    )
    parser.add_argument(
        "--scale-x",
        type=float,
        default=7.0,
        help="World-to-classifier scaling for x feature (feature_x = world_x / scale_x).",
    )
    parser.add_argument(
        "--scale-y",
        type=float,
        default=7.0,
        help="World-to-classifier scaling for y feature (feature_y = world_y / scale_y).",
    )
    parser.add_argument(
        "--show-points",
        action="store_true",
        help="Overlay positive gestation records if available.",
    )
    return parser.parse_args()


def parse_scene_path(models_dir: Path, explicit_scene: Optional[str]) -> Optional[Path]:
    if explicit_scene:
        p = Path(explicit_scene)
        return p if p.exists() else None

    scene_txt = models_dir / "scene_file.txt"
    if not scene_txt.exists():
        return None

    content = scene_txt.read_text().strip()
    if not content:
        return None

    p = Path(content)
    if p.exists():
        return p

    # fallback: resolve relative to script cwd and models dir parent
    p2 = (models_dir.parent / content).resolve()
    if p2.exists():
        return p2
    return None


def draw_obstacles(ax, scene_path: Optional[Path]) -> None:
    if scene_path is None or not scene_path.exists():
        return

    tree = ET.parse(scene_path)
    root = tree.getroot()

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


def parse_sparse_vec(tokens: List[str]) -> np.ndarray:
    max_idx = 0
    pairs: List[Tuple[int, float]] = []
    for tok in tokens:
        if ":" not in tok:
            continue
        i_str, v_str = tok.split(":", 1)
        i = int(i_str)
        v = float(v_str)
        pairs.append((i, v))
        if i > max_idx:
            max_idx = i
    vec = np.zeros(max_idx, dtype=np.float64)
    for i, v in pairs:
        vec[i - 1] = v
    return vec


def parse_libsvm_model(path: Path) -> Dict:
    lines = path.read_text().splitlines()
    header: Dict[str, str] = {}
    sv_start = None
    for i, line in enumerate(lines):
        if line.strip() == "SV":
            sv_start = i + 1
            break
        parts = line.strip().split(maxsplit=1)
        if len(parts) == 2:
            header[parts[0]] = parts[1]

    if sv_start is None:
        raise RuntimeError(f"Invalid model file (no SV section): {path}")

    gamma = float(header.get("gamma", "1.0"))
    rho = float(header.get("rho", "0.0").split()[0])
    svm_type = header.get("svm_type", "")
    kernel_type = header.get("kernel_type", "")
    nr_class = int(header.get("nr_class", "2"))
    labels = [int(x) for x in header.get("label", "").split()] if "label" in header else []

    if kernel_type != "rbf":
        raise RuntimeError(f"Only RBF kernels are supported for plotting. Found: {kernel_type} in {path}")

    coefs: List[float] = []
    svs: List[np.ndarray] = []
    for line in lines[sv_start:]:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        coefs.append(float(parts[0]))
        svs.append(parse_sparse_vec(parts[1:]))

    dim = max((sv.shape[0] for sv in svs), default=0)
    sv_arr = np.zeros((len(svs), dim), dtype=np.float64)
    for i, sv in enumerate(svs):
        sv_arr[i, : sv.shape[0]] = sv

    coef_arr = np.array(coefs, dtype=np.float64)
    return {
        "path": str(path),
        "svm_type": svm_type,
        "nr_class": nr_class,
        "labels": labels,
        "gamma": gamma,
        "rho": rho,
        "sv": sv_arr,
        "coef": coef_arr,
    }


def decision_function(model: Dict, features: np.ndarray) -> np.ndarray:
    # features: [N, D], model["sv"]: [S, D]
    sv = model["sv"]
    coef = model["coef"]
    gamma = model["gamma"]
    rho = model["rho"]

    if sv.shape[0] == 0:
        return np.full((features.shape[0],), -rho, dtype=np.float64)

    # Pairwise squared distances: ||x||^2 + ||sv||^2 - 2 x·sv
    x2 = np.sum(features * features, axis=1, keepdims=True)  # [N,1]
    sv2 = np.sum(sv * sv, axis=1, keepdims=True).T  # [1,S]
    dist2 = x2 + sv2 - 2.0 * (features @ sv.T)  # [N,S]
    dist2 = np.maximum(dist2, 0.0)
    K = np.exp(-gamma * dist2)  # [N,S]
    return K @ coef - rho


def load_points_2d(path: Path) -> np.ndarray:
    if not path.exists():
        return np.zeros((0, 2), dtype=np.float64)
    lines = path.read_text().strip().splitlines()
    if not lines:
        return np.zeros((0, 2), dtype=np.float64)

    # format: first line count, then pos(3)+quat(4)+vel(3)+ang(3)
    pts = []
    for line in lines[1:]:
        toks = line.strip().split()
        if len(toks) < 2:
            continue
        pts.append((float(toks[0]), float(toks[1])))
    if not pts:
        return np.zeros((0, 2), dtype=np.float64)
    return np.array(pts, dtype=np.float64)


def discover_skills(models_dir: Path) -> List[int]:
    out = []
    pat = re.compile(r"skill_(\d+)_classifier\.svm$")
    for p in models_dir.glob("skill_*_classifier.svm"):
        m = pat.match(p.name)
        if m:
            out.append(int(m.group(1)))
    return sorted(set(out))


def main() -> None:
    args = parse_args()
    models_dir = Path(args.models_dir).resolve()
    if not models_dir.exists():
        raise FileNotFoundError(f"models dir not found: {models_dir}")

    scene_path = parse_scene_path(models_dir, args.scene)
    skills = sorted(set(args.skills)) if args.skills else discover_skills(models_dir)
    if not skills:
        print(f"No classifier files found in {models_dir}")
        return

    out_dir = models_dir / "initiation_set_viz"
    out_dir.mkdir(parents=True, exist_ok=True)

    xs = np.linspace(args.xmin, args.xmax, args.grid)
    ys = np.linspace(args.ymin, args.ymax, args.grid)
    xx, yy = np.meshgrid(xs, ys)
    world_pts = np.stack([xx.ravel(), yy.ravel()], axis=1)
    feat_pts = np.stack([world_pts[:, 0] / args.scale_x, world_pts[:, 1] / args.scale_y], axis=1)

    for sid in skills:
        opt_path = models_dir / f"skill_{sid}_classifier.svm"
        pess_path = models_dir / f"skill_{sid}_classifier.svm_pessimistic"
        if not opt_path.exists() and not pess_path.exists():
            continue

        fig, axes = plt.subplots(1, 2, figsize=(13, 6), sharex=True, sharey=True)
        ax_opt, ax_pess = axes

        for ax in axes:
            draw_obstacles(ax, scene_path)
            ax.set_xlim(args.xmin, args.xmax)
            ax.set_ylim(args.ymin, args.ymax)
            ax.set_aspect("equal")
            ax.set_xlabel("x")
        ax_opt.set_ylabel("y")

        if opt_path.exists():
            opt_model = parse_libsvm_model(opt_path)
            opt_dec = decision_function(opt_model, feat_pts).reshape(xx.shape)
            vmax = max(1e-6, float(np.percentile(np.abs(opt_dec), 98)))
            opt_norm = TwoSlopeNorm(vmin=-vmax, vcenter=0.0, vmax=vmax)
            cf_opt = ax_opt.contourf(
                xx, yy, opt_dec,
                levels=61, cmap="coolwarm", norm=opt_norm, alpha=0.9
            )
            ax_opt.contour(
                xx,
                yy,
                opt_dec,
                levels=[0.0],
                colors=["tab:blue"],
                linewidths=1.7,
                linestyles="--",
            )
            ax_opt.set_title("Optimistic decision value")
            cbar_opt = fig.colorbar(cf_opt, ax=ax_opt, fraction=0.046, pad=0.04)
            cbar_opt.set_label("decision value")
        else:
            ax_opt.set_title("Optimistic classifier (missing)")
            ax_opt.text(0.5, 0.5, "No optimistic model", transform=ax_opt.transAxes, ha="center", va="center")

        if pess_path.exists():
            pess_model = parse_libsvm_model(pess_path)
            pess_dec = decision_function(pess_model, feat_pts).reshape(xx.shape)
            vmax = max(1e-6, float(np.percentile(np.abs(pess_dec), 98)))
            pess_norm = TwoSlopeNorm(vmin=-vmax, vcenter=0.0, vmax=vmax)
            cf_pess = ax_pess.contourf(
                xx, yy, pess_dec,
                levels=61, cmap="coolwarm", norm=pess_norm, alpha=0.9
            )
            ax_pess.contour(
                xx,
                yy,
                pess_dec,
                levels=[0.0],
                colors=["tab:red"],
                linewidths=1.8,
                linestyles="-",
            )
            ax_pess.set_title("Pessimistic decision value")
            cbar_pess = fig.colorbar(cf_pess, ax=ax_pess, fraction=0.046, pad=0.04)
            cbar_pess.set_label("decision value")
        else:
            ax_pess.set_title("Pessimistic classifier (missing)")
            ax_pess.text(0.5, 0.5, "No pessimistic model", transform=ax_pess.transAxes, ha="center", va="center")

        if args.show_points:
            pos_path = models_dir / f"skill_{sid}_classifier.svm_positives.txt"
            eff_path = models_dir / f"skill_{sid}_classifier.svm_effect.txt"
            pos = load_points_2d(pos_path)
            eff = load_points_2d(eff_path)
            if pos.shape[0] > 0:
                ax_opt.scatter(pos[:, 0], pos[:, 1], s=6, color="black", alpha=0.45)
                ax_pess.scatter(pos[:, 0], pos[:, 1], s=6, color="black", alpha=0.45)
            if eff.shape[0] > 0:
                ax_opt.scatter(eff[:, 0], eff[:, 1], s=8, color="tab:green", alpha=0.55)
                ax_pess.scatter(eff[:, 0], eff[:, 1], s=8, color="tab:green", alpha=0.55)

        # stable legend entries
        handles = [
            plt.Line2D([0], [0], color="tab:blue", linestyle="--", lw=1.7, label="optimistic boundary"),
            plt.Line2D([0], [0], color="tab:red", linestyle="-", lw=1.8, label="pessimistic boundary"),
        ]
        if args.show_points:
            handles.append(
                plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="black", markersize=5, label="positives")
            )
            handles.append(
                plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="tab:green", markersize=5, label="effect")
            )
        fig.legend(handles=handles, loc="upper center", ncol=4, fontsize=8, frameon=True)
        fig.suptitle(f"Skill {sid} initiation-set decision-value maps", fontsize=12)

        out_path = out_dir / f"skill_{sid}_boundaries.png"
        fig.tight_layout(rect=[0, 0, 1, 0.93])
        fig.savefig(out_path, dpi=160)
        plt.close(fig)
        print(f"[saved] {out_path}")

    print(f"Done. Output directory: {out_dir}")


if __name__ == "__main__":
    main()
