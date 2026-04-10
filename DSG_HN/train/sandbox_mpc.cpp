// sandbox_mpc.cpp — main loop (robot bridge + MuJoCo side, NO torch headers).
// Torch model code lives in sandbox_mpc_torch.cpp via the sandbox_mpc_torch.h API.

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
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#define SCENE_FILE          "../config/scene/umaze_scene.xml"
#define USE_WALL_CLOCK_TIME true
#define CONTROL_HZ          10
#define GOAL_REACHED_DIST   0.15
#define STALL_WINDOW        50
#define STALL_THRESHOLD     0.95

#define JS_DEVICE     "/dev/input/js0"
#define JS_MAX_AXIS   32767.0f
#define DEADZONE      0.05f

static std::atomic<bool> running{true};
static void sigint_handler(int) { running = false; }

static RolloutState robot_state_to_rollout(const RobotState &s) {
    RolloutState r;
    r.x  = s.position[0]; r.y  = s.position[1];
    r.vx = s.velocity[0]; r.vy = s.velocity[1];
    r.oz = s.angular_velocity[2];
    float qw=s.orientation[0], qx=s.orientation[1], qy=s.orientation[2], qz=s.orientation[3];
    r.yaw = std::atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));
    r.joint_pos.assign(s.q.begin(),  s.q.end());
    r.joint_vel.assign(s.dq.begin(), s.dq.end());
    return r;
}

static void save_stall_node(const StallNode &sn, const std::string &path) {
    bool exists = std::ifstream(path).good();
    std::ofstream ofs(path, std::ios::app);
    if (!exists)
        ofs << "x,y,yaw,goal_x,goal_y,steps_taken\n";
    ofs << std::fixed << std::setprecision(4)
        << sn.x << ',' << sn.y << ',' << sn.yaw << ','
        << sn.goal_x << ',' << sn.goal_y << ',' << sn.steps_taken << '\n';
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, sigint_handler);

    namespace po = boost::program_options;
    po::options_description desc("sandbox_mpc — live Transformer MPC (run from train/build)");
    desc.add_options()
        ("help,h",      "show help")
        ("checkpoint",  po::value<std::string>()->required(), "path to .pt checkpoint")
        ("normaliser",  po::value<std::string>()->default_value(""), "normaliser.txt")
        ("goal-x",      po::value<double>()->default_value(3.0), "initial goal x")
        ("goal-y",      po::value<double>()->default_value(0.0), "initial goal y")
        ("history",     po::value<int>()->default_value(10), "context window N")
        ("d-model",     po::value<int>()->default_value(128), "transformer d_model")
        ("n-heads",     po::value<int>()->default_value(4),   "attention heads")
        ("n-layers",    po::value<int>()->default_value(4),   "transformer layers")
        ("seed",        po::value<int>()->default_value(42),  "random seed")
        ("max-steps",   po::value<int>()->default_value(200), "max MPC steps before stall")
        ("horizon",     po::value<int>()->default_value(10),  "MPC planning horizon")
        ("cem-rounds",  po::value<int>()->default_value(3),   "CEM refinement rounds")
        ("cem-elites",  po::value<int>()->default_value(32),  "CEM elite count")
        ("candidates",  po::value<int>()->default_value(256), "CEM candidates per round")
        ("w-pos",       po::value<double>()->default_value(1.0),  "per-step distance weight")
        ("w-heading",   po::value<double>()->default_value(0.5),  "heading alignment weight")
        ("w-terminal",  po::value<double>()->default_value(3.0),  "terminal position weight")
        ("w-smooth",    po::value<double>()->default_value(0.1),  "action smoothness weight")
        ("w-backward",  po::value<double>()->default_value(0.3),  "backward velocity penalty");

    // param::helper does not tolerate unknown options, so initialise it
    // manually (bin_path, config, log level) without its parse_command_line.
    param::bin_path = param::get_bin_path();
    param::load_config_file();
#ifdef NDEBUG
    spdlog::set_level(spdlog::level::info);
#else
    spdlog::set_level(spdlog::level::debug);
#endif

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) { std::cout << desc; return 0; }
        po::notify(vm);
    } catch (const std::exception &e) { std::cerr << e.what() << '\n' << desc; return 1; }

    const auto ckpt_path = vm["checkpoint"].as<std::string>();
    const int  history   = vm["history"].as<int>();
    const int  d_model   = vm["d-model"].as<int>();
    const int  n_heads   = vm["n-heads"].as<int>();
    const int  n_layers  = vm["n-layers"].as<int>();
    const int  max_steps = vm["max-steps"].as<int>();
    double goal_x = vm["goal-x"].as<double>();
    double goal_y = vm["goal-y"].as<double>();

    MpcConfig mpc_cfg;
    mpc_cfg.candidates  = vm["candidates"].as<int>();
    mpc_cfg.horizon     = vm["horizon"].as<int>();
    mpc_cfg.cem_rounds  = vm["cem-rounds"].as<int>();
    mpc_cfg.cem_elites  = vm["cem-elites"].as<int>();
    mpc_cfg.w_pos       = vm["w-pos"].as<double>();
    mpc_cfg.w_heading   = vm["w-heading"].as<double>();
    mpc_cfg.w_terminal  = vm["w-terminal"].as<double>();
    mpc_cfg.w_smooth    = vm["w-smooth"].as<double>();
    mpc_cfg.w_backward  = vm["w-backward"].as<double>();

    std::string norm_path = vm["normaliser"].as<std::string>();
    if (norm_path.empty()) {
        auto sl = ckpt_path.find_last_of('/');
        norm_path = (sl==std::string::npos ? "." : ckpt_path.substr(0,sl)) + "/normaliser.txt";
    }

    auto ctx = mpc_create(ckpt_path, norm_path, history, d_model, n_heads, n_layers, mpc_cfg);
    std::cout << "Loaded Transformer from " << ckpt_path << '\n';

    // MuJoCo robot
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);
    auto robot_bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, -5.0f, 5.0f, -5.0f, 5.0f, policy_dir, true);

    Joystick js(JS_DEVICE);
    bool has_joystick = js.isFound();
    if (!has_joystick) std::cout << "No joystick — keyboard override disabled.\n";
    Keyboard kb;

    std::cout << "\nsandbox_mpc ready  [CEM " << mpc_cfg.cem_rounds << "x"
              << mpc_cfg.candidates << ", " << mpc_cfg.cem_elites << " elites, H="
              << mpc_cfg.horizon << ", max=" << max_steps << "]\n"
              << "  Goal: (" << goal_x << ", " << goal_y << ")\n"
              << "  G           -> enter new goal\n"
              << "  R           -> reset robot\n"
              << "  0 / Ctrl+C  -> quit\n\n";

    auto [pos, quat] = robot_bridge->generateRandomPose();
    robot_bridge->resetRobot(pos, quat);

    unsigned seed_base = (unsigned)vm["seed"].as<int>();
    int step = 0;
    std::deque<double> dist_history;

    auto handle_stall = [&](const RolloutState &rs, const char *reason) {
        robot_bridge->publishVelCommand({0.f, 0.f, 0.f});
        robot_bridge->update();
        StallNode sn{rs.x, rs.y, rs.yaw, goal_x, goal_y, step};
        save_stall_node(sn, "stall_nodes.csv");
        std::cout << "STALL [" << reason << "] at step=" << step
                  << "  pos=(" << std::fixed << std::setprecision(2)
                  << rs.x << ", " << rs.y << ")  saved to stall_nodes.csv\n";
    };

    while (running)
    {
        kb.update();
        if (has_joystick) js.getState();

        if (kb.key() == "0" || (has_joystick && js.button_[7])) break;

        if (kb.key() == "r" || (has_joystick && js.button_[6])) {
            auto [p, q] = robot_bridge->generateRandomPose();
            robot_bridge->resetRobot(p, q);
            mpc_clear_history(*ctx);
            step = 0;
            dist_history.clear();
            std::cout << "Robot reset.\n";
            continue;
        }

        if (kb.key() == "g") {
            std::cout << "Enter goal x y: "; std::cout.flush();
            std::cin >> goal_x >> goal_y;
            std::cout << "New goal: (" << goal_x << ", " << goal_y << ")\n";
            mpc_clear_history(*ctx);
            step = 0;
            dist_history.clear();
            continue;
        }

        auto t0 = std::chrono::steady_clock::now();

        auto rs = robot_bridge->getRobotState();
        auto rollout_state = robot_state_to_rollout(rs);

        double dist = std::sqrt(std::pow(rollout_state.x - goal_x, 2) +
                                std::pow(rollout_state.y - goal_y, 2));

        if (dist < GOAL_REACHED_DIST) {
            std::cout << "GOAL REACHED! dist=" << std::fixed << std::setprecision(3)
                      << dist << "m  at step=" << step << "\n";
            robot_bridge->publishVelCommand({0.f, 0.f, 0.f});
            robot_bridge->update();
            dist_history.clear();
            step = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (step >= max_steps) {
            handle_stall(rollout_state, "timeout");
            break;
        }

        dist_history.push_back(dist);
        if ((int)dist_history.size() > STALL_WINDOW)
            dist_history.pop_front();

        if ((int)dist_history.size() == STALL_WINDOW &&
            dist >= dist_history.front() * STALL_THRESHOLD) {
            handle_stall(rollout_state, "no progress");
            break;
        }

        auto best = mpc_plan(*ctx, rollout_state, goal_x, goal_y, seed_base + step);

        robot_bridge->publishVelCommand({(float)best.vx, (float)best.vy, (float)best.yaw});
        robot_bridge->update();

        mpc_update_history(*ctx, rollout_state, best);

        if (step % 5 == 0) {
            std::cout << "Step " << std::setw(4) << step
                      << "  pos=(" << std::fixed << std::setprecision(2)
                      << rollout_state.x << ", " << rollout_state.y << ")"
                      << "  dist=" << std::setprecision(3) << dist << "m"
                      << "  cmd=(" << std::setprecision(2) << best.vx
                      << ", " << best.vy << ", " << best.yaw << ")\n";
        }
        ++step;

        auto elapsed = std::chrono::steady_clock::now() - t0;
        auto remaining = std::chrono::milliseconds(1000/CONTROL_HZ) - elapsed;
        if (remaining > std::chrono::milliseconds(0) && USE_WALL_CLOCK_TIME)
            std::this_thread::sleep_for(remaining);
    }

    robot_bridge->publishVelCommand({0.f, 0.f, 0.f});
    return 0;
}
