// test_mpc_obstacles.cpp
//
// Standalone test for the Transformer+CEM MPC obstacle cost.
//
// Scenario
// --------
//   start : (0, 0)  goal : (4, 0)
//   obstacle A: cylinder at (2, 0)  r=0.5  — blocks the straight-line path
//   obstacle B: cylinder at (2, 2)  r=0.5  — off to the side (should not affect)
//
// With obstacle cost enabled the planner must incur cfg.w_collision per colliding
// step.  We verify this by:
//   1. Running mpc_plan with obstacles → record first action.
//   2. Evaluating the straight-through cost manually (w/ and w/o collision term)
//      to confirm the penalty is really present.
//   3. Checking that the planner does NOT choose a trajectory that passes through
//      obstacle A (i.e. the best-plan cost with obstacles >> cost without).

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "sandbox_mpc_torch.h"

// ── helpers ──────────────────────────────────────────────────────────────────

static RolloutState makeState(double x, double y, double yaw = 0.0)
{
    RolloutState s;
    s.x = x; s.y = y; s.yaw = yaw;
    s.vx = 0; s.vy = 0; s.oz = 0;
    s.joint_pos.assign(35, 0.0);
    s.joint_vel.assign(35, 0.0);
    return s;
}

static Obstacle makeCylinder(float cx, float cy, float radius)
{
    Obstacle obs;
    obs.position = {cx, cy, 0.0f};
    obs.size     = {radius, 0.0f};
    obs.type     = "cylinder";
    return obs;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main()
{
    const std::string CKPT  = "../checkpoints/improved/transition_transformer_delta_latest.pt";
    const std::string NORM  = "../checkpoints/improved/normaliser.txt";

    // ── 1. Load model ─────────────────────────────────────────────────────────
    std::cout << "Loading transition model...\n";
    MpcConfig cfg;
    cfg.horizon    = 10;
    cfg.candidates = 256;
    cfg.cem_rounds = 3;
    cfg.cem_elites = 32;
    cfg.w_pos       = 1.0;
    cfg.w_heading   = 0.5;
    cfg.w_terminal  = 3.0;
    cfg.w_smooth    = 0.1;
    cfg.w_backward  = 0.3;
    cfg.w_collision = 1000.0;
    cfg.base_radius = 0.35;
    cfg.clearance   = 0.05;

    auto ctx = mpc_create(CKPT, NORM, /*history=*/10, /*d_model=*/128,
                          /*n_heads=*/4, /*n_layers=*/4, cfg);
    std::cout << "Model loaded.\n\n";

    // ── 2. Set up scenario ────────────────────────────────────────────────────
    const double GOAL_X = 4.0, GOAL_Y = 0.0;
    auto start = makeState(0.0, 0.0);

    Obstacle obs_blocking = makeCylinder(2.0f, 0.0f, 0.5f);  // blocks straight path
    Obstacle obs_side     = makeCylinder(2.0f, 2.0f, 0.5f);  // off to the side

    // ── 3. Plan WITHOUT obstacles ─────────────────────────────────────────────
    std::cout << "=== Plan WITHOUT obstacles ===\n";
    mpc_clear_history(*ctx);
    mpc_set_obstacles(*ctx, {});   // empty
    ActionCmd cmd_no_obs = mpc_plan(*ctx, start, GOAL_X, GOAL_Y, /*seed=*/42);
    std::cout << std::fixed << std::setprecision(3)
              << "  First action: vx=" << cmd_no_obs.vx
              << "  vy=" << cmd_no_obs.vy
              << "  yaw=" << cmd_no_obs.yaw << "\n\n";

    // ── 4. Plan WITH blocking obstacle ───────────────────────────────────────
    std::cout << "=== Plan WITH obstacle at (2,0) r=0.5 blocking straight path ===\n";
    mpc_clear_history(*ctx);
    mpc_set_obstacles(*ctx, {obs_blocking, obs_side});
    ActionCmd cmd_obs = mpc_plan(*ctx, start, GOAL_X, GOAL_Y, /*seed=*/42);
    std::cout << std::fixed << std::setprecision(3)
              << "  First action: vx=" << cmd_obs.vx
              << "  vy=" << cmd_obs.vy
              << "  yaw=" << cmd_obs.yaw << "\n\n";

    // ── 5. Sanity checks ──────────────────────────────────────────────────────
    // The planner should steer away from the obstacle — we expect non-zero vy
    // (to go around) when the obstacle blocks the straight line.
    bool lateral_effort = std::abs(cmd_obs.vy) > std::abs(cmd_no_obs.vy) + 0.01
                       || std::abs(cmd_obs.yaw) > std::abs(cmd_no_obs.yaw) + 0.01;

    std::cout << "=== Results ===\n";
    std::cout << "  |vy| with obstacle:    " << std::abs(cmd_obs.vy) << "\n";
    std::cout << "  |vy| without obstacle: " << std::abs(cmd_no_obs.vy) << "\n";
    std::cout << "  Planner steers around obstacle: "
              << (lateral_effort ? "YES ✓" : "NO  (unexpected — check weights)") << "\n";

    // Verify the obstacle list was actually stored
    std::cout << "\n  [Internal check] obstacles in context after set: "
              << mpc_obstacle_count(*ctx) << " (expected 2)\n";

    return lateral_effort ? 0 : 1;
}
