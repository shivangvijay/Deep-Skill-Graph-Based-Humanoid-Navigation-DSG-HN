// test_mpc_expansion.cpp
//
// Validates that the Transformer+CEM MPC is wired into the expansion phase
// correctly — i.e., that removing the `|| true` bypass in _runMPC means the
// planner runs and produces goal-directed actions.
//
// WHAT IS TESTED (standalone, no MuJoCo env)
// -------------------------------------------
// 1. The model loads — _mpc_ctx will be non-null, so _runMPC will take the
//    Transformer+CEM branch, not the global-option fallback.
// 2. First action is goal-directed (cosine similarity with direction-to-goal > 0)
//    for the cases where the goal is reachable by forward locomotion.
// 3. The receding-horizon loop (mpc_plan → mpc_update_history) runs without
//    crashing for N steps.
// 4. Over a short horizon the planner makes net forward progress toward the goal.
//
// WHY MULTI-STEP DISTANCE IS ONLY CHECKED FOR A SHORT HORIZON
// ------------------------------------------------------------
// The Transformer uses a 10-step history. In the real environment joint states
// evolve realistically, keeping inputs in-distribution. In this standalone test
// we propagate state with a simple kinematic model and static joint states, so
// the history drifts out-of-distribution after roughly 8–10 steps. We therefore
// only assert distance progress over the first NEAR_HORIZON steps.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "sandbox_mpc_torch.h"

static constexpr const char *CKPT = "../checkpoints/improved/transition_transformer_delta_latest.pt";
static constexpr const char *NORM = "../checkpoints/improved/normaliser.txt";

static constexpr int    NEAR_HORIZON       = 8;    // steps before OOD drift kicks in
static constexpr int    FULL_STEPS         = 20;   // total steps to run (for API test)
static constexpr double DT                 = 0.1;  // seconds per step
static constexpr double NEAR_PROGRESS_MIN  = 0.1;  // metres gained within NEAR_HORIZON

static RolloutState makeState(double x, double y, double yaw = 0.0)
{
    RolloutState s;
    s.x = x; s.y = y; s.yaw = yaw;
    s.vx = 0; s.vy = 0; s.oz = 0;
    s.joint_pos.assign(35, 0.0);
    s.joint_vel.assign(35, 0.0);
    return s;
}

// Body-frame velocity → global-frame position/velocity (pure kinematics).
// In the real env, joint states would also evolve — here they stay frozen.
// This is valid for short horizons but drifts OOD as history accumulates.
static RolloutState applyKinematics(const RolloutState &s, const ActionCmd &cmd, double dt)
{
    const double cy = std::cos(s.yaw), sy = std::sin(s.yaw);
    RolloutState next = s;
    next.x   += (cmd.vx * cy - cmd.vy * sy) * dt;
    next.y   += (cmd.vx * sy + cmd.vy * cy) * dt;
    next.yaw += cmd.yaw * dt;
    next.vx   = cmd.vx * cy - cmd.vy * sy;  // global frame, matches training data
    next.vy   = cmd.vx * sy + cmd.vy * cy;
    return next;
}

static double dist2d(const RolloutState &s, double gx, double gy)
{
    const double dx = s.x - gx, dy = s.y - gy;
    return std::sqrt(dx * dx + dy * dy);
}

// Cosine similarity between the planned action (projected to global frame) and
// the direction toward the goal.
static double goalCos(const RolloutState &s, const ActionCmd &cmd, double gx, double gy)
{
    const double cy = std::cos(s.yaw), sy = std::sin(s.yaw);
    const double ax = cmd.vx * cy - cmd.vy * sy;
    const double ay = cmd.vx * sy + cmd.vy * cy;
    const double dx = gx - s.x, dy = gy - s.y;
    const double a  = std::sqrt(ax*ax + ay*ay);
    const double d  = std::sqrt(dx*dx + dy*dy);
    if (a < 1e-6 || d < 1e-6) return 1.0;
    return (ax*dx + ay*dy) / (a*d);
}

struct Case { double sx, sy, syaw, gx, gy; const char *name; bool expect_fwd; };

int main()
{
    std::cout << "Loading transition model...\n";
    MpcConfig cfg;
    cfg.horizon    = 7;   // matches _dsg_cfg.mpc_horizon
    cfg.candidates = 256;
    cfg.cem_rounds = 3;
    cfg.cem_elites = 32;
    cfg.w_pos       = 1.0;
    cfg.w_heading   = 0.5;
    cfg.w_terminal  = 3.0;
    cfg.w_smooth    = 0.1;
    cfg.w_backward  = 0.3;
    cfg.w_collision = 200.0;
    cfg.base_radius = 0.35;
    cfg.clearance   = 0.05;

    auto ctx = mpc_create(CKPT, NORM, 10, 128, 4, 4, cfg);
    std::cout << "Model loaded — _mpc_ctx will be non-null, MPC branch will be taken.\n\n";

    // Cases where the model should clearly prefer forward motion.
    // expect_fwd=false marks goals that require significant turning first,
    // where the first action may legitimately have negative forward component.
    const std::vector<Case> cases = {
        { 0.0,  0.0,  0.0,   5.0,  0.0,  "straight ahead (+x)",            true  },
        { 0.0,  0.0,  0.0,   3.0,  1.0,  "forward-right diagonal",         true  },
        { 0.0,  0.0,  1.57,  0.0,  4.0,  "facing +y, goal ahead",          true  },
        { 2.0, -2.0,  0.0,  -1.0,  2.0,  "backward-left goal (turning ok)", false },
        { 0.0,  0.0,  0.0,   0.0,  3.0,  "pure lateral (model may turn)",  false },
    };

    int fwd_pass = 0, fwd_total = 0;
    int near_pass = 0, near_total = 0;
    bool api_ok = true;

    for (const auto &tc : cases)
    {
        std::cout << "──────────────────────────────────────────\n";
        std::cout << "Case: " << tc.name << "\n";
        std::cout << std::fixed << std::setprecision(3);

        // ── Check 1: first action goal-directedness ───────────────────────
        mpc_clear_history(*ctx);
        mpc_set_obstacles(*ctx, {});
        RolloutState s0 = makeState(tc.sx, tc.sy, tc.syaw);
        ActionCmd first = mpc_plan(*ctx, s0, tc.gx, tc.gy, 0u);
        double cs0 = goalCos(s0, first, tc.gx, tc.gy);

        std::cout << "  first action: vx=" << first.vx
                  << " vy=" << first.vy << " yaw=" << first.yaw
                  << "  cos=" << cs0 << "\n";

        if (tc.expect_fwd)
        {
            ++fwd_total;
            if (cs0 > 0.0) { ++fwd_pass; std::cout << "  Check 1 (goal-directed): PASS\n"; }
            else             std::cout << "  Check 1 (goal-directed): FAIL (cos <= 0)\n";
        }
        else
        {
            std::cout << "  Check 1 (goal-directed): SKIP (turning expected first)\n";
        }

        // ── Check 2: net progress over short horizon ──────────────────────
        mpc_clear_history(*ctx);
        RolloutState state = makeState(tc.sx, tc.sy, tc.syaw);
        double d_start = dist2d(state, tc.gx, tc.gy);

        for (int step = 0; step < FULL_STEPS; ++step)
        {
            ActionCmd cmd = mpc_plan(*ctx, state, tc.gx, tc.gy, static_cast<unsigned>(step));
            mpc_update_history(*ctx, state, cmd);  // API must not crash

            double d   = dist2d(state, tc.gx, tc.gy);
            double cos = goalCos(state, cmd, tc.gx, tc.gy);
            std::cout << "    step " << std::setw(2) << (step+1)
                      << "  d=" << std::setw(5) << d
                      << "  cos=" << std::setw(6) << cos
                      << "  vx=" << std::setw(6) << cmd.vx
                      << "  vy=" << std::setw(6) << cmd.vy << "\n";

            state = applyKinematics(state, cmd, DT);
            if (step == NEAR_HORIZON - 1)
            {
                // Evaluate progress before OOD drift becomes severe
                double d_near = dist2d(state, tc.gx, tc.gy);
                double prog   = d_start - d_near;
                ++near_total;
                if (prog > NEAR_PROGRESS_MIN)
                {
                    ++near_pass;
                    std::cout << "  Check 2 (near progress=" << prog << "m): PASS\n";
                }
                else
                {
                    std::cout << "  Check 2 (near progress=" << prog
                              << "m, need " << NEAR_PROGRESS_MIN << "m): FAIL\n";
                }
            }
            if (d < 0.15) { std::cout << "  [reached goal early]\n"; break; }
        }
    }

    // ── Summary ───────────────────────────────────────────────────────────────
    std::cout << "\n══════════════════════════════════════════\n";
    std::cout << "Check 1 (first action forward): " << fwd_pass  << "/" << fwd_total  << "\n";
    std::cout << "Check 2 (near-horizon progress): " << near_pass << "/" << near_total << "\n";
    std::cout << "Check 3 (API no crash):          " << (api_ok ? "PASS" : "FAIL") << "\n";

    // Require at least 2/3 of forward-expected cases to pass, and most near-progress.
    bool ok = (fwd_pass  >= (fwd_total  * 2 / 3))
           && (near_pass >= (near_total * 2 / 3))
           && api_ok;

    std::cout << "\nOverall: " << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
