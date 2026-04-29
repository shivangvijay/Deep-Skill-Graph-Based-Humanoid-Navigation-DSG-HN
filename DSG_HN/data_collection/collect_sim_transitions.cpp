#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "mujoco_utils/mujoco_articulation.h"
#include "mujoco_utils/mujoco_engine.h"
#include "robot_bridge_train.h"
#include "param.h"
#include "joystick/joystick.h"
#include "isaaclab/devices/keyboard/keyboard.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <csignal>
#include <atomic>
#include <thread>

#define SCENE_FILE "../config/scene/umaze_scene.xml"
#define OUTPUT_FILE "../../sandbox/build/transitions_umaze.csv"
#define STABILIZE_STEPS 10

#define X_MIN -9.0f
#define X_MAX  9.0f
#define Y_MIN -9.0f
#define Y_MAX  9.0f

#define VX_MAX   1.0f
#define VX_MIN  -0.5f
#define VY_MAX   0.3f
#define VY_MIN  -0.3f
#define YAW_MAX  1.0f
#define YAW_MIN -1.0f

#define JS_DEVICE "/dev/input/js0"
#define JS_MAX_AXIS 32767.0f
#define DEADZONE 0.05f

static std::atomic<bool> g_running{true};
static void handle_signal(int) { g_running = false; }

static float norm_axis(int raw)
{
    float v = float(raw) / JS_MAX_AXIS;
    return (std::fabs(v) < DEADZONE) ? 0.0f : v;
}

static float scale_vx(float n)  { return n >= 0.0f ? n * VX_MAX  : n * -VX_MIN; }
static float scale_vy(float n)  { return n >= 0.0f ? n * VY_MAX  : n * -VY_MIN; }
static float scale_yaw(float n) { return n >= 0.0f ? n * YAW_MAX : n * -YAW_MIN; }

class TransitionWriter
{
public:
    TransitionWriter(const std::string &path)
        : file(path, std::ios::app), t0(std::chrono::steady_clock::now())
    {
        if (!file.is_open())
        {
            std::cerr << "Cannot open output file: " << path << "\n";
            std::exit(1);
        }
        file << std::fixed << std::setprecision(6);
        file.seekp(0, std::ios::end);
        if (file.tellp() == 0)
        {
        file << "timestamp_s,"
             << "x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,";
        for (int i = 0; i < DOF; ++i) file << "joint_pos_" << std::setw(2) << std::setfill('0') << i << ",";
        for (int i = 0; i < DOF; ++i) file << "joint_vel_" << std::setw(2) << std::setfill('0') << i << ",";
        file << "cmd_vx,cmd_vy,cmd_yaw,"
             << "next_x,next_y,next_z,next_qw,next_qx,next_qy,next_qz,"
             << "next_vx,next_vy,next_vz,next_omega_x,next_omega_y,next_omega_z,";
        for (int i = 0; i < DOF; ++i) file << "next_joint_pos_" << std::setw(2) << std::setfill('0') << i << ",";
        for (int i = 0; i < DOF; ++i) file << "next_joint_vel_" << std::setw(2) << std::setfill('0') << i << ((i < DOF-1) ? "," : "");
        file << "\n";
        }
    }

    void write(const RobotState &s0, const std::vector<float> &cmd,
               const RobotState &s1)
    {
        auto now = std::chrono::steady_clock::now();
        double ts = std::chrono::duration<double>(now - t0).count();

        file << ts << ","
             << s0.position[0]         << "," << s0.position[1]         << "," << s0.position[2]         << ","
             << s0.orientation[0]      << "," << s0.orientation[1]      << "," << s0.orientation[2]      << "," << s0.orientation[3] << ","
             << s0.velocity[0]         << "," << s0.velocity[1]         << "," << s0.velocity[2]         << ","
             << s0.angular_velocity[0] << "," << s0.angular_velocity[1] << "," << s0.angular_velocity[2] << ",";
        for (int i = 0; i < DOF; ++i) file << s0.q[i] << ",";
        for (int i = 0; i < DOF; ++i) file << s0.dq[i] << ",";
        file << cmd[0] << "," << cmd[1] << "," << cmd[2] << ","
             << s1.position[0]         << "," << s1.position[1]         << "," << s1.position[2]         << ","
             << s1.orientation[0]      << "," << s1.orientation[1]      << "," << s1.orientation[2]      << "," << s1.orientation[3] << ","
             << s1.velocity[0]         << "," << s1.velocity[1]         << "," << s1.velocity[2]         << ","
             << s1.angular_velocity[0] << "," << s1.angular_velocity[1] << "," << s1.angular_velocity[2] << ",";
        for (int i = 0; i < DOF; ++i) file << s1.q[i] << ",";
        for (int i = 0; i < DOF; ++i) file << s1.dq[i] << ((i < DOF-1) ? "," : "");
        file << "\n";
        count++;
    }

    size_t rows() const { return count; }
    void flush() { file.flush(); }
    ~TransitionWriter() { file.close(); }

private:
    std::ofstream file;
    std::chrono::steady_clock::time_point t0;
    size_t count = 0;
};

static void stabilize(std::shared_ptr<RobotBridgeTrain> bridge, int n_steps)
{
    std::vector<float> zero_cmd = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < n_steps; i++)
    {
        bridge->publishVelCommand(zero_cmd);
        bridge->update();
    }
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, handle_signal);

    auto vm = param::helper(argc, argv);
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    std::cout << "Initializing MuJoCo simulation...\n";
    auto bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, true);

    Joystick js(JS_DEVICE);
    bool has_joystick = js.isFound();
    if (!has_joystick)
        std::cout << "No joystick at " << JS_DEVICE << " — keyboard only.\n";

    Keyboard kb;
    TransitionWriter writer(OUTPUT_FILE);

    std::cout << "\n=== JOYSTICK + KEYBOARD DATA COLLECTION ===\n"
              << "  Writing to: " << OUTPUT_FILE << "\n"
              << "  Deploy ranges:  vx [" << VX_MIN << ", " << VX_MAX << "]"
              << "  vy [" << VY_MIN << ", " << VY_MAX << "]"
              << "  yaw [" << YAW_MIN << ", " << YAW_MAX << "]\n"
              << "Keyboard:\n"
              << "  W / S       = forward / backward (vx)\n"
              << "  A / D       = strafe left / right (vy)\n"
              << "  Q / E       = rotate left / right (yaw)\n"
              << "  R           = reset robot\n"
              << "  0 / Ctrl+C  = quit & save\n"
              << "Joystick:\n"
              << "  Left stick  = vx / vy\n"
              << "  Right stick = yaw\n"
              << "  Back        = reset robot\n"
              << "  Start       = quit\n\n";

    auto [pos, quat, vel, ang_vel] = bridge->generateRandomPoseWithVel();
    bridge->resetRobot(pos, quat, vel, ang_vel);
    stabilize(bridge, STABILIZE_STEPS);

    int ep = 0;

    while (g_running)
    {
        kb.update();
        if (has_joystick) js.getState();

        if (kb.key() == "0" || (has_joystick && js.button_[7]))
            break;

        if (kb.key() == "r" || (has_joystick && js.button_[6]))
        {
            auto [p, q, v, a] = bridge->generateRandomPoseWithVel();
            bridge->resetRobot(p, q, v, a);
            stabilize(bridge, STABILIZE_STEPS);
            writer.flush();
            ep++;
            std::cout << "RESET  ep=" << ep << "  total rows=" << writer.rows() << "\n";
            continue;
        }

        float vx = 0.0f, vy = 0.0f, yaw = 0.0f;
        auto k = kb.key();

        if (k == "w")       vx = VX_MAX;
        else if (k == "s")  vx = VX_MIN;
        else if (k == "a")  vy = VY_MAX;
        else if (k == "d")  vy = VY_MIN;
        else if (k == "q")  yaw = YAW_MAX;
        else if (k == "e")  yaw = YAW_MIN;
        else if (has_joystick)
        {
            vx  = scale_vx(-norm_axis(js.axis_[1]));
            vy  = scale_vy(norm_axis(js.axis_[0]));
            yaw = scale_yaw(norm_axis(js.axis_[3]));
        }

        auto loop_t0 = std::chrono::steady_clock::now();

        RobotState s0 = bridge->getRobotState();
        bridge->publishVelCommand({vx, vy, yaw});
        bridge->update();
        RobotState s1 = bridge->getRobotState();

        writer.write(s0, {vx, vy, yaw}, s1);

        if (writer.rows() % 50 == 0) {
            writer.flush();
            std::cout << "ep=" << ep
                      << "  rows=" << writer.rows()
                      << "  cmd=(" << std::fixed << std::setprecision(2)
                      << vx << ", " << vy << ", " << yaw << ")"
                      << "  pos=(" << s0.position[0] << ", " << s0.position[1] << ")\n";
        }

        auto elapsed = std::chrono::steady_clock::now() - loop_t0;
        auto remaining = std::chrono::milliseconds(100) - elapsed;
        if (remaining > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remaining);
    }

    writer.flush();
    std::cout << "\nSaved " << writer.rows() << " transitions to " << OUTPUT_FILE << "\n";
    return 0;
}
