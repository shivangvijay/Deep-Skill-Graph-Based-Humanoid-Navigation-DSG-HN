#pragma once
// Torch-side declarations for sandbox_mpc.
// This header is included by BOTH translation units but <torch/torch.h> is
// only included in sandbox_mpc_torch.cpp (compiled with torch include paths).
// sandbox_mpc.cpp (the main loop) sees only opaque pointers / free functions.

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "robot_bridge.h"  // Obstacle struct

struct RolloutState {
    double x=0, y=0, yaw=0, vx=0, vy=0, oz=0;
    std::vector<double> joint_pos, joint_vel;
};

struct ActionCmd { double vx, vy, yaw; };

struct MpcConfig {
    double w_pos       = 1.0;
    double w_heading   = 1.0;
    double w_terminal  = 3.0;
    double w_smooth    = 0.1;
    double w_backward  = 3.0;
    double w_collision = 200.0;  // soft penetration cost (linear, 0 at safe boundary)
    double base_radius = 0.35;   // robot footprint radius (metres)
    double clearance   = 0.05;   // extra safety margin around obstacles
    int cem_rounds     = 3;
    int cem_elites     = 32;
    int candidates     = 256;
    int horizon        = 10;
};

struct StallNode {
    double x, y, yaw;
    double goal_x, goal_y;
    int steps_taken;
};

struct MpcContext;   // opaque — defined in sandbox_mpc_torch.cpp

struct MpcContextDeleter { void operator()(MpcContext *p) const; };
using MpcContextPtr = std::unique_ptr<MpcContext, MpcContextDeleter>;

MpcContextPtr mpc_create(
    const std::string &ckpt_path,
    const std::string &normaliser_path,
    int history, int d_model, int n_heads, int n_layers,
    const MpcConfig &cfg = MpcConfig{});

ActionCmd mpc_plan(MpcContext &ctx,
                   const RolloutState &state,
                   double goal_x, double goal_y,
                   unsigned seed_offset);

void mpc_update_history(MpcContext &ctx,
                        const RolloutState &state,
                        const ActionCmd &cmd);

void mpc_clear_history(MpcContext &ctx);

// Update the obstacle list used by the cost function.
// Call once per _runMPC() invocation (obstacles are static within one expansion step).
void mpc_set_obstacles(MpcContext &ctx, const std::vector<Obstacle> &obstacles);

// Returns the number of obstacles currently stored in the context (for testing).
int mpc_obstacle_count(const MpcContext &ctx);
