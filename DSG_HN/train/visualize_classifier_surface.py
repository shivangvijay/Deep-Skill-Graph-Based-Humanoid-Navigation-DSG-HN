import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import xml.etree.ElementTree as ET
import os
import argparse

def draw_obstacles(ax, scene_file):
    """Parses MuJoCo XML and draws obstacles on the provided axis."""
    if not os.path.exists(scene_file):
        print(f"[Warning] Scene file not found: {scene_file}")
        return

    tree = ET.parse(scene_file)
    root = tree.getroot()
    obs_count = 0

    for geom in root.iter('geom'):
        name = geom.get('name', '')
        # Filter for obstacles, walls, or cylinders
        if any(keyword in name.lower() for keyword in ['obs', 'c', 'wall']):
            pos = [float(x) for x in geom.get('pos', '0 0 0').split()]
            size = [float(x) for x in geom.get('size', '0 0 0').split()]
            gtype = geom.get('type', 'box')
            x, y = pos[0], pos[1]

            if gtype == 'box':
                sx, sy = size[0], size[1]
                rect = patches.Rectangle((x-sx, y-sy), 2*sx, 2*sy, 
                                        linewidth=1, edgecolor='black', 
                                        facecolor='gray', alpha=0.4, zorder=1)
                ax.add_patch(rect)
                obs_count += 1
            elif gtype == 'cylinder':
                r = size[0]
                circle = patches.Circle((x, y), r, linewidth=1, edgecolor='black', 
                                        facecolor='gray', alpha=0.4, zorder=1)
                ax.add_patch(circle)
                obs_count += 1
    print(f"Drew {obs_count} obstacles from scene.")

def main():
    parser = argparse.ArgumentParser(description="Visualize initiation sets.")
    parser.add_argument("scene_file", help="Path to scene file")
    parser.add_argument("points_file", help="Path to points file")
    parser.add_argument("surface_file", help="Path to surface file")
    parser.add_argument("--combined", action="store_true", 
                        help="Plot all pessimistic boundaries on one figure")
    
    args = parser.parse_args()

    try:
        pts = np.loadtxt(args.points_file)
        surf = np.loadtxt(args.surface_file)
    except Exception as e:
        print(f"Error loading data: {e}")
        return

    num_skills = (surf.shape[1] - 2) // 2
    res = int(np.sqrt(surf.shape[0]))
    X = surf[:, 0].reshape((res, res)).T
    Y = surf[:, 1].reshape((res, res)).T
    
    if args.combined:
        fig, ax = plt.subplots(figsize=(8, 8))
        cmap = plt.get_cmap('tab10') 
        
        # DRAW OBSTACLES FIRST (zorder=1)
        draw_obstacles(ax, args.scene_file)
        
        for i in range(num_skills):
            color = cmap(i % 10) 
            pess_vals = surf[:, 3 + 2*i].reshape((res, res)).T
            
            if np.max(pess_vals) > 0:
                ax.contourf(X, Y, pess_vals, levels=[0, 1e9], colors=[color], alpha=0.3, zorder=2)
                ax.contour(X, Y, pess_vals, levels=[0], colors=[color], linewidths=2, zorder=3)
                ax.fill_between([], [], color=color, alpha=0.3, label=f"Skill {i}")

        ax.set_title("Pessimistic Initiation Sets")
        ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), fontsize='small', borderaxespad=0.)
    else:
        fig, axes = plt.subplots(1, num_skills, figsize=(6 * num_skills, 6), squeeze=False)
        for i in range(num_skills):
            ax = axes[0, i]
            draw_obstacles(ax, args.scene_file)
            
            opt_vals = surf[:, 2 + 2*i].reshape((res, res)).T
            pess_vals = surf[:, 3 + 2*i].reshape((res, res)).T

            mesh = ax.pcolormesh(X, Y, opt_vals, shading='nearest', cmap='RdBu_r', vmin=-1, vmax=1, zorder=0)
            if np.max(pess_vals) > 0:
                ax.contour(X, Y, pess_vals, levels=[0], colors=['yellow'], linewidths=2, zorder=3)

            if pts.ndim == 1: pts = pts.reshape(1, -1)
            skill_pts = pts[pts[:, 0] == i]
            ax.scatter(skill_pts[:, 1], skill_pts[:, 2], color='black', s=10, alpha=0.6, zorder=4)
            plt.colorbar(mesh, ax=ax)

    # Add Goal and Start Markers
    goal_pos, start_pos = (-4.5, 4.1), (-5.3, -4.5)
    target_axes = [ax] if args.combined else axes[0]

    for a in target_axes:
        a.scatter(*goal_pos, color='red', marker='*', s=150, edgecolors='black', zorder=10)
        a.text(goal_pos[0], goal_pos[1] + 0.3, "Goal", color='red', fontweight='bold', ha='center', zorder=11)
        a.scatter(*start_pos, color='lime', marker='o', s=100, edgecolors='black', zorder=10)
        a.text(start_pos[0], start_pos[1] - 0.7, "Start", color='green', fontweight='bold', ha='center', zorder=11)

    # Final Formatting
    for a in fig.get_axes():
        if isinstance(a, plt.Axes):
            a.set_xlim(-7.0, 7.0)
            a.set_ylim(-7.0, 7.0)
            a.set_aspect('equal')

    plt.tight_layout()
    if not os.path.exists("../logs"): os.makedirs("../logs")
    
    filename = "combined_pessimistic.png" if args.combined else "initiation_sets_surface.png"
    plt.savefig(f"../logs/{filename}", bbox_inches='tight')
    print(f"Saved plot to ../logs/{filename}")

if __name__ == "__main__":
    main()