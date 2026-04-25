#include "robot_bridge_train.h"
#include "param.h"
#include "joystick/joystick.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <cmath>
#include <csignal>
#include <atomic>
#include <iomanip>
#include <sstream>
#include "environment.h"
#include <torch/torch.h>
#include <memory>
#include <filesystem>
namespace fs = std::filesystem;

// #define SCENE_FILE "umaze_scene.xml"
static constexpr const char *DEFAULT_OUTPUT = "../transitions_narrow_random.csv";

#define SCENE_FILE "test_scene_narrow.xml"
// Match policy training ranges from deploy.yaml
// commands.base_velocity.ranges: lin_vel_x[-0.5,1.0], lin_vel_y[-0.3,0.3], ang_vel_z[-0.2,0.2]
#define VX_MAX 1.0f
#define VX_MIN -0.5f
#define VY_MAX 0.3f
#define VY_MIN -0.3f
#define YAW_MAX 1.0f
#define YAW_MIN -1.0f

#define X_MIN -3.0f
#define X_MAX 3.0f
#define Y_MIN -3.0f
#define Y_MAX 3.0f

// Joystick device
#define JS_DEVICE "/dev/input/js0"
#define JS_MAX_AXIS 32767.0f
#define DEADZONE 0.05f

#define COLLECT_TRAIN_ENV_TRANSITIONS true
std::unique_ptr<TrainEnvironment> train_env = nullptr;

static std::atomic<bool> running{true};

static void sigint_handler(int) { running = false; }

static std::string make_csv_header(bool train_env_transitions)
{
    std::ostringstream header;
    if (train_env_transitions)
    {
        header << "timestamp_s,";

        int s_dim = train_env->state_dim;
        int a_dim = train_env->action_dim;

        for (int i = 0; i < s_dim; ++i)
            header << "state_" << i << ",";

        for (int i = 0; i < a_dim; ++i)
            header << "action_" << i << ",";

        for (int i = 0; i < s_dim; ++i)
            header << "next_state_" << i << ",";

        header << "reward,done\n";
    }
    else
    {
        header << "timestamp_s,"
               << "x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,";

        for (int i = 0; i < DOF; ++i)
        {
            header << "joint_pos_" << std::setw(2) << std::setfill('0') << i << ',';
        }

        for (int i = 0; i < DOF; ++i)
        {
            header << "joint_vel_" << std::setw(2) << std::setfill('0') << i << ',';
        }

        header << "cmd_vx,cmd_vy,cmd_yaw,"
               << "next_x,next_y,next_z,next_qw,next_qx,next_qy,next_qz,"
               << "next_vx,next_vy,next_vz,next_omega_x,next_omega_y,next_omega_z,";

        for (int i = 0; i < DOF; ++i)
        {
            header << "next_joint_pos_" << std::setw(2) << std::setfill('0') << i << ',';
        }

        for (int i = 0; i < DOF; ++i)
        {
            header << "next_joint_vel_" << std::setw(2) << std::setfill('0') << i;
            if (i + 1 < DOF)
                header << ',';
        }

        header << '\n';
    }
    return header.str();
}

static void write_state_row(std::ostream &csv, const RobotState &state)
{
    csv << state.position[0] << ',' << state.position[1] << ',' << state.position[2] << ','
        << state.orientation[0] << ',' << state.orientation[1] << ',' << state.orientation[2] << ',' << state.orientation[3] << ','
        << state.velocity[0] << ',' << state.velocity[1] << ',' << state.velocity[2] << ','
        << state.angular_velocity[0] << ',' << state.angular_velocity[1] << ',' << state.angular_velocity[2] << ',';

    for (int i = 0; i < DOF; ++i)
    {
        csv << state.q[i] << ',';
    }

    for (int i = 0; i < DOF; ++i)
    {
        csv << state.dq[i];
        if (i + 1 < DOF)
            csv << ',';
    }
}

static void write_train_env_row(std::ostream &csv, const torch::Tensor &state, const torch::Tensor &action, const torch::Tensor &next_state, const torch::Tensor &reward, const torch::Tensor &done)
{
    auto s = state.flatten();
    auto a = action.flatten();
    auto ns = next_state.flatten();

    for (int i = 0; i < s.size(0); i++)  csv << s.accessor<float, 1>()[i] << ",";
    for (int i = 0; i < a.size(0); i++)  csv << a.accessor<float, 1>()[i] << ",";
    for (int i = 0; i < ns.size(0); i++) csv << ns.accessor<float, 1>()[i] << ",";

    csv << reward.reshape({-1})[0].item<float>() << ",";
    csv << done.reshape({-1})[0].item<float>() << "\n";
}

// Normalise raw joystick axis to [-1, 1] with deadzone
static float norm(int raw)
{
    float v = float(raw) / JS_MAX_AXIS;
    return (std::fabs(v) < DEADZONE) ? 0.0f : v;
}

// Scale normalised joystick value asymmetrically to match policy range
static float scale_vx(float n) { return n >= 0.0f ? n * VX_MAX : n * -VX_MIN; }
static float scale_vy(float n) { return n >= 0.0f ? n * VY_MAX : n * -VY_MIN; }
static float scale_yaw(float n) { return n >= 0.0f ? n * YAW_MAX : n * -YAW_MIN; }

int main(int argc, char **argv)
{
    std::signal(SIGINT, sigint_handler);

    [[maybe_unused]] auto vm = param::helper(argc, argv);

    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    auto robot_bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, true /* render */);


    AbstractedState global_goal = {{2.0, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    AbstractedState global_start = {{-2.0, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    torch::Tensor state;
    if (COLLECT_TRAIN_ENV_TRANSITIONS)
    {
        train_env = std::make_unique<TrainEnvironment>(robot_bridge, 500);
        train_env->clearGoal();
        // train_env->setGoal(global_goal);
        train_env->setGoalDistance(std::sqrt((X_MAX - X_MIN) * (X_MAX - X_MIN) + (Y_MAX - Y_MIN) * (Y_MAX - Y_MIN)));
        state = train_env->reset(); //To(global_start);
        train_env->updateGoalMarker();
    }
    {
        robot_bridge->resetRobot(global_start.position, global_start.orientation);
    }

    // Joystick (optional — graceful fallback to keyboard-only)
    Joystick js(JS_DEVICE);
    bool has_joystick = js.isFound();
    if (!has_joystick)
        std::cout << "No joystick found at " << JS_DEVICE << " — keyboard only.\n";

    // Keyboard (same mapping as deploy/robots/g1_29dof)
    Keyboard kb;

    bool file_exists = fs::exists(DEFAULT_OUTPUT);

    // Open with std::ios::app to append to the end of the file
    std::ofstream csv(DEFAULT_OUTPUT, std::ios::app);

    if (!csv.is_open())
    {
        std::cerr << "Failed to open output file: " << DEFAULT_OUTPUT << '\n';
        return 1;
    }

    csv << std::fixed << std::setprecision(6);

    if (!file_exists)
    {
        csv << make_csv_header(COLLECT_TRAIN_ENV_TRANSITIONS);
    }

    std::cout << "\nSandbox ready.\n"
              << "Keyboard:\n"
              << "  w / s       → forward / backward (vx)\n"
              << "  a / d       → strafe left / right (vy)\n"
              << "  q / e       → rotate left / right (ωz)\n"
              << "  r           → reset robot\n"
              << "  0           → quit\n"
              << "Joystick:\n"
              << "  Left  stick → vx / vy\n"
              << "  Right stick → ωz\n"
              << "  Back        → reset robot\n"
              << "  Start       → quit\n"
              << "  Ctrl+C      → quit\n\n";

    std::cout << "Recording transitions to " << DEFAULT_OUTPUT << "...\n";

    size_t count = 0;

    while (running)
    {
        kb.update();

        if (has_joystick)
            js.getState();

        // ── Quit ──────────────────────────────────────────────────────────
        if (kb.key() == "0" || (has_joystick && js.button_[7]))
            break;

        // ── Reset ─────────────────────────────────────────────────────────
        if (kb.key() == "r" || (has_joystick && js.button_[6]))
        {
            auto [pos, quat] = robot_bridge->generateRandomPose();
            robot_bridge->resetRobot(pos, quat);
            continue;
        }

        // ── Velocity command ──────────────────────────────────────────────
        float vx = 0.0f, vy = 0.0f, oz = 0.0f;
        float vx_unscaled = 0.0f, vy_unscaled = 0.0f, oz_unscaled = 0.0f;

        // Keyboard takes priority when a movement key is held
        auto k = kb.key();
        if (k == "w")
        {
            vx = VX_MAX;
            vx_unscaled = 1;
        }
        else if (k == "s")
        {
            vx = VX_MIN;
            vx_unscaled = -1;
        }
        else if (k == "a")
        {
            vy = VY_MAX;
            vy_unscaled = 1;
        }
        else if (k == "d")
        {
            vy = VY_MIN;
            vy_unscaled = -1;
        }
        else if (k == "q")
        {
            oz = YAW_MAX;
            oz_unscaled = 1;
        }
        else if (k == "e")
        {
            oz = YAW_MIN;
            oz_unscaled = 1;

        }
        else if (has_joystick)
        {
            // Fallback to joystick axes; negate Y so stick-up = forward
            vx_unscaled = -norm(js.axis_[1]);
            vy_unscaled = -norm(js.axis_[0]);
            oz_unscaled = -norm(js.axis_[3]);
            vx = scale_vx(vx_unscaled);
            vy = scale_vy(vy_unscaled);
            oz = scale_yaw(oz_unscaled);
        }

        auto epoch = std::chrono::system_clock::now().time_since_epoch();
        double timestamp = std::chrono::duration<double>(epoch).count();

        if (COLLECT_TRAIN_ENV_TRANSITIONS)
        {
            torch::Tensor scaled_action = torch::tensor({vx, vy, oz}, torch::kFloat32);
            torch::Tensor action = torch::tensor({vx_unscaled, vy_unscaled, oz_unscaled});
            auto [next_state, reward, done] = train_env->step(scaled_action);
            csv << timestamp << ',';
            write_train_env_row(csv, state, action, next_state, reward, done);

            if (done.accessor<float, 1>()[0] > 0.5)
            {
                if (reward.accessor<float, 1>()[0] > 45)
                    std::cout << "Successful trajectory, resetting..." << std::endl;
                else
                    std::cout << "Unsuccessful trajectory, resetting..." << std::endl;
                state = train_env->reset(); //To(global_start);
                train_env->updateGoalMarker();
            }
            else
            {
                state = next_state;
            }
        }
        else
        {
            auto s0 = robot_bridge->getRobotState();
            robot_bridge->publishVelCommand({vx, vy, oz});
            robot_bridge->update();
            auto s1 = robot_bridge->getRobotState();

            csv << timestamp << ',';
            write_state_row(csv, s0);
            csv << ',' << vx << ',' << vy << ',' << oz << ',';
            write_state_row(csv, s1);
            csv << '\n';
        }

        ++count;
        if (count % 20 == 0)
            std::cout << "Recorded " << count << " transitions\r" << std::flush;
    }

    std::cout << "\nSaved " << count << " transitions to " << DEFAULT_OUTPUT << '\n';

    return 0;
}