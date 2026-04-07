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

struct RolloutState {
    double x=0, y=0, yaw=0, vx=0, vy=0, oz=0;
    std::vector<double> joint_pos, joint_vel;
};

struct ActionCmd { double vx, vy, yaw; };

struct MpcContext;   // opaque — defined in sandbox_mpc_torch.cpp

struct MpcContextDeleter { void operator()(MpcContext *p) const; };
using MpcContextPtr = std::unique_ptr<MpcContext, MpcContextDeleter>;

MpcContextPtr mpc_create(
    const std::string &ckpt_path,
    const std::string &normaliser_path,
    int history, int d_model, int n_heads, int n_layers);

ActionCmd mpc_plan(MpcContext &ctx,
                   const RolloutState &state,
                   double goal_x, double goal_y,
                   unsigned seed_offset);

void mpc_update_history(MpcContext &ctx,
                        const RolloutState &state,
                        const ActionCmd &cmd);

void mpc_clear_history(MpcContext &ctx);
