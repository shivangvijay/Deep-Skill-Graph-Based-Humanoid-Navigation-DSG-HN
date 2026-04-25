import sys
import numpy as np
import matplotlib.pyplot as plt
import os
import time

def main():
    if len(sys.argv) < 4:
        print("Usage: python visualize_classifier_surface.py <scene_file> <points_file> <surface_file>")
        return

    points_path = sys.argv[2]
    surface_path = sys.argv[3]

    try:
        pts = np.loadtxt(points_path)
        surf = np.loadtxt(surface_path)
    except Exception as e:
        print(f"Error: {e}")
        return

    num_skills = (surf.shape[1] - 2) // 2
    res = int(np.sqrt(surf.shape[0]))
    
    # Reshape and Transpose to match X-outer, Y-inner C++ loop
    # This aligns Matrix[x][y] to the plot axes correctly
    X = surf[:, 0].reshape((res, res)).T
    Y = surf[:, 1].reshape((res, res)).T
    
    # We will create a subplot for each skill to show the RAW values clearly
    # because overlaying multiple heatmaps makes the values unreadable.
    fig, axes = plt.subplots(1, num_skills, figsize=(6 * num_skills, 6), squeeze=False)

    for i in range(num_skills):
        ax = axes[0, i]
        
        # Extract raw decision values
        # opt_vals is the raw distance from the SVM hyperplane
        opt_vals = surf[:, 2 + 2*i].reshape((res, res)).T
        pess_vals = surf[:, 3 + 2*i].reshape((res, res)).T

        # Show the RAW values as a heatmap
        # Shading 'nearest' ensures no interpolation (no fake gradients)
        mesh = ax.pcolormesh(X, Y, opt_vals, shading='nearest', cmap='RdBu_r', vmin=-1, vmax=1)
        
        # Add the 'Pessimistic' boundary as a single solid contour at value 0
        if np.max(pess_vals) > 0:
            ax.contour(X, Y, pess_vals, levels=[0], colors=['yellow'], linewidths=2)

        # Plot the training points (actual world coords)
        if pts.ndim == 1: pts = pts.reshape(1, -1)
        skill_pts = pts[pts[:, 0] == i]
        ax.scatter(skill_pts[:, 1], skill_pts[:, 2], color='black', s=10, alpha=0.6)

        # FORCE STRICT BOUNDS (-3 to 3)
        ax.set_xlim(-7.0, 7.0)
        ax.set_ylim(-7.0, 7.0)
        ax.set_aspect('equal')
        ax.set_title(f"Skill {i} Raw Decision Values")
        plt.colorbar(mesh, ax=ax, label='SVM Decision Value f(x)')

    plt.tight_layout()
    if not os.path.exists("../results"):
        os.makedirs("../results")
    plt.savefig(f"../results/initiation_sets_surface_{time.time()}.png")

if __name__ == "__main__":
    main()