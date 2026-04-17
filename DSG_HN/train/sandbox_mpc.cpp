// sandbox_mpc.cpp — live MuJoCo demo of Transformer+CEM MPC with obstacle cost.
// Robot-bridge / MuJoCo side only — NO torch headers (see sandbox_mpc_torch.cpp).
//
// Controls (keyboard):
//   G           — enter a new goal (x y)
//   R           — reset robot to a random pose
//   0 / Ctrl+C  — quit
//
// Usage (run from train/build):
//   ./sandbox_mpc [--checkpoint PATH] [--goal-x X] [--goal-y Y] [--w-collision W] ...

#include "sandbox_mpc_torch.h"

#include "robot_bridge_train.h"
#include "param.h"
#include "joystick/joystick.h"
#include "isaaclab/devices/keyboard/keyboard.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

// ── Scene with obstacles for testing obstacle avoidance ───────────────────────
#define SCENE_FILE          "../config/scene/umaze_scene.xml"
#define CONTROL_HZ          10
#define GOAL_REACHED_DIST   0.3
#define STALL_WINDOW        200
#define STALL_THRESHOLD     0.98

static std::atomic<bool> running{true};
static void sigint_handler(int) { running = false; }

static RolloutState robot_state_to_rollout(const RobotState &s)
{
    RolloutState r;
    r.x   = s.position[0];
    r.y   = s.position[1];
    r.vx  = s.velocity[0];
    r.vy  = s.velocity[1];
    r.oz  = s.angular_velocity[2];
    float qw = s.orientation[0], qx = s.orientation[1];
    float qy = s.orientation[2], qz = s.orientation[3];
    r.yaw = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    r.joint_pos.assign(s.q.begin(),  s.q.end());
    r.joint_vel.assign(s.dq.begin(), s.dq.end());
    return r;
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, sigint_handler);

    namespace po = boost::program_options;
    po::options_description desc("sandbox_mpc — live Transformer MPC (run from train/build)");
    desc.add_options()
        ("help,h",        "show help")
        ("checkpoint",    po::value<std::string>()->default_value(
                              "../checkpoints/improved/transition_transformer_delta_latest.pt"),
                          "path to .pt checkpoint")
        ("normaliser",    po::value<std::string>()->default_value(""), "normaliser.txt path")
        ("goal-x",        po::value<double>()->default_value(5.0),  "initial goal x")
        ("goal-y",        po::value<double>()->default_value(-2.0), "initial goal y")
        ("max-steps",     po::value<int>()->default_value(300),     "steps before stall reset")
        ("horizon",       po::value<int>()->default_value(10),      "MPC planning horizon")
        ("candidates",    po::value<int>()->default_value(256),     "CEM candidates")
        ("cem-rounds",    po::value<int>()->default_value(3),       "CEM refinement rounds")
        ("cem-elites",    po::value<int>()->default_value(32),      "CEM elite count")
        ("w-pos",         po::value<double>()->default_value(1.0),  "per-step distance weight")
        ("w-heading",     po::value<double>()->default_value(1.0),  "heading weight")
        ("w-terminal",    po::value<double>()->default_value(3.0),  "terminal distance weight")
        ("w-smooth",      po::value<double>()->default_value(0.1),  "smoothness weight")
        ("w-backward",    po::value<double>()->default_value(3.0),  "backward penalty")
        ("w-collision",   po::value<double>()->default_value(200.0), "obstacle collision cost (soft linear penetration)")
        ("base-radius",   po::value<double>()->default_value(0.35), "robot footprint radius (m)")
        ("clearance",     po::value<double>()->default_value(0.05), "safety margin around obstacles (m)");

    param::bin_path = param::get_bin_path();
    param::load_config_file();

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) { std::cout << desc; return 0; }
        po::notify(vm);
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n' << desc; return 1;
    }

    // ── MPC context ───────────────────────────────────────────────────────────
    MpcConfig mpc_cfg;
    mpc_cfg.horizon     = vm["horizon"].as<int>();
    mpc_cfg.candidates  = vm["candidates"].as<int>();
    mpc_cfg.cem_rounds  = vm["cem-rounds"].as<int>();
    mpc_cfg.cem_elites  = vm["cem-elites"].as<int>();
    mpc_cfg.w_pos       = vm["w-pos"].as<double>();
    mpc_cfg.w_heading   = vm["w-heading"].as<double>();
    mpc_cfg.w_terminal  = vm["w-terminal"].as<double>();
    mpc_cfg.w_smooth    = vm["w-smooth"].as<double>();
    mpc_cfg.w_backward  = vm["w-backward"].as<double>();
    mpc_cfg.w_collision = vm["w-collision"].as<double>();
    mpc_cfg.base_radius = vm["base-radius"].as<double>();
    mpc_cfg.clearance   = vm["clearance"].as<double>();

    const std::string ckpt_path = vm["checkpoint"].as<std::string>();
    std::string norm_path       = vm["normaliser"].as<std::string>();
    if (norm_path.empty()) {
        auto sl = ckpt_path.find_last_of('/');
        norm_path = (sl == std::string::npos ? "." : ckpt_path.substr(0, sl)) + "/normaliser.txt";
    }

    std::cout << "Loading Transformer checkpoint: " << ckpt_path << "\n";
    auto ctx = mpc_create(ckpt_path, norm_path,
                          /*history=*/10, /*d_model=*/128, /*n_heads=*/4, /*n_layers=*/4,
                          mpc_cfg);
    std::cout << "Model loaded. w_collision=" << mpc_cfg.w_collision
              << "  base_radius=" << mpc_cfg.base_radius << "\n";

    // ── Robot bridge ──────────────────────────────────────────────────────────
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);
    auto robot_bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, -5.0f, 5.0f, -5.0f, 5.0f, policy_dir, /*render=*/true);

    // Wire obstacle cost — obstacles are static for this scene so set once.
    auto obstacles = robot_bridge->getObstacles();
    mpc_set_obstacles(*ctx, obstacles);
    std::cout << "Scene obstacles loaded: " << mpc_obstacle_count(*ctx) << "\n";

    // Visualise obstacle safety margins in the viewer
    auto eng = robot_bridge->getEngine();
    {
        std::vector<MuJoCoEngine::ObstacleMarker> vis_markers;
        for (const auto &obs : obstacles)
            vis_markers.push_back({obs.position[0], obs.position[1],
                                   (float)mpc_cfg.base_radius});
        eng->setObstacleMarkers(vis_markers, (float)(mpc_cfg.base_radius + mpc_cfg.clearance));
    }

    // ── Input ─────────────────────────────────────────────────────────────────
    Joystick js("/dev/input/js0");
    bool has_joystick = js.isFound();
    if (!has_joystick) std::cout << "No joystick found — keyboard only.\n";
    Keyboard kb;

    double goal_x = vm["goal-x"].as<double>();
    double goal_y = vm["goal-y"].as<double>();
    const int max_steps = vm["max-steps"].as<int>();

    eng->setGoalMarker((float)goal_x, (float)goal_y, 0.4f);

    auto [pos, quat] = robot_bridge->generateRandomPose();
    robot_bridge->resetRobot(pos, quat);
    mpc_clear_history(*ctx);

    std::cout << "\nsandbox_mpc ready  [CEM " << mpc_cfg.cem_rounds << "×"
              << mpc_cfg.candidates << ", " << mpc_cfg.cem_elites << " elites, H="
              << mpc_cfg.horizon << "]\n"
              << "  Goal: (" << goal_x << ", " << goal_y << ")\n"
              << "  G / Ctrl+C → new goal / quit\n"
              << "  R          → reset robot\n"
              << "  0          → quit\n\n";

    int step = 0;
    unsigned seed = 42;
    std::deque<double> dist_hist;

    while (running)
    {
        kb.update();
        if (has_joystick) js.getState();

        if (kb.key() == "0" || (has_joystick && js.button_[7])) break;

        if (kb.key() == "r" || (has_joystick && js.button_[6])) {
            auto [p, q] = robot_bridge->generateRandomPose();
            robot_bridge->resetRobot(p, q);
            mpc_clear_history(*ctx);
            step = 0; dist_hist.clear();
            std::cout << "Robot reset.\n";
            continue;
        }

        if (kb.key() == "g") {
            std::cout << "Enter new goal (x y): "; std::cout.flush();
            std::cin >> goal_x >> goal_y;
            eng->setGoalMarker((float)goal_x, (float)goal_y, 0.4f);
            mpc_clear_history(*ctx);
            step = 0; dist_hist.clear();
            std::cout << "New goal: (" << goal_x << ", " << goal_y << ")\n";
            continue;
        }

        auto t0 = std::chrono::steady_clock::now();

        auto rs          = robot_bridge->getRobotState();
        auto rollout_st  = robot_state_to_rollout(rs);
        double dist      = std::hypot(rollout_st.x - goal_x, rollout_st.y - goal_y);

        // Goal reached
        if (dist < GOAL_REACHED_DIST) {
            robot_bridge->publishVelCommand({0, 0, 0});
            robot_bridge->update();
            std::cout << "GOAL REACHED at step=" << step
                      << "  dist=" << std::fixed << std::setprecision(3) << dist << " m\n";
            mpc_clear_history(*ctx);
            step = 0; dist_hist.clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        // Stall detection
        dist_hist.push_back(dist);
        if ((int)dist_hist.size() > STALL_WINDOW) dist_hist.pop_front();
        if (step >= max_steps ||
            ((int)dist_hist.size() == STALL_WINDOW &&
             dist >= dist_hist.front() * STALL_THRESHOLD)) {
            robot_bridge->publishVelCommand({0, 0, 0});
            robot_bridge->update();
            std::cout << "STALL at step=" << step
                      << "  pos=(" << std::fixed << std::setprecision(2)
                      << rollout_st.x << ", " << rollout_st.y << ")\n";
            auto [p, q] = robot_bridge->generateRandomPose();
            robot_bridge->resetRobot(p, q);
            mpc_clear_history(*ctx);
            step = 0; dist_hist.clear();
            continue;
        }

        // ── MPC step ──────────────────────────────────────────────────────────
        ActionCmd cmd = mpc_plan(*ctx, rollout_st, goal_x, goal_y, seed + step);
        robot_bridge->publishVelCommand({(float)cmd.vx, (float)cmd.vy, (float)cmd.yaw});
        robot_bridge->update();
        mpc_update_history(*ctx, rollout_st, cmd);

        if (step % 10 == 0) {
            std::cout << "step " << std::setw(4) << step
                      << "  pos=(" << std::fixed << std::setprecision(2)
                      << rollout_st.x << ", " << rollout_st.y << ")"
                      << "  dist=" << std::setprecision(3) << dist << " m"
                      << "  cmd=(" << std::setprecision(2) << cmd.vx
                      << ", " << cmd.vy << ", " << cmd.yaw << ")\n";
        }
        ++step;

        // Pace to CONTROL_HZ
        auto elapsed   = std::chrono::steady_clock::now() - t0;
        auto remaining = std::chrono::milliseconds(1000 / CONTROL_HZ) - elapsed;
        if (remaining > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remaining);
    }

    robot_bridge->publishVelCommand({0, 0, 0});
    return 0;
}
