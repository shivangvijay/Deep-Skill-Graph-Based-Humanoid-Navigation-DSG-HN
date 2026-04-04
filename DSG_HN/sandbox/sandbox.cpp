#include "robot_bridge_train.h"
#include "param.h"
#include "joystick/joystick.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include <iostream>
#include <fstream>
#include <deque>
#include <cmath>
#include <csignal>
#include <atomic>
#include <string>

#define USE_WALL_CLOCK_TIME true
#define RECORD_TRANSITIONS true
#define TRANSITION_FILE "transitions.csv"
#define RESET_TRIM_SECONDS 5.0

#define SCENE_FILE "umaze_scene.xml"
// Match policy training ranges from deploy.yaml
// commands.base_velocity.ranges: lin_vel_x[-0.5,1.0], lin_vel_y[-0.3,0.3], ang_vel_z[-0.2,0.2]
#define VX_MAX 1.0f
#define VX_MIN -0.5f
#define VY_MAX 0.3f
#define VY_MIN -0.3f
#define YAW_MAX 0.2f
#define YAW_MIN -0.2f

// Joystick device
#define JS_DEVICE "/dev/input/js0"
#define JS_MAX_AXIS 32767.0f
#define DEADZONE 0.05f

static std::atomic<bool> running{true};

static void sigint_handler(int) { running = false; }

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

    auto vm = param::helper(argc, argv);

    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    auto robot_bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, -5.0f, 5.0f, -5.0f, 5.0f, policy_dir, true /* render */);

    // Joystick (optional — graceful fallback to keyboard-only)
    Joystick js(JS_DEVICE);
    bool has_joystick = js.isFound();
    if (!has_joystick)
        std::cout << "No joystick found at " << JS_DEVICE << " — keyboard only.\n";

    // Keyboard (same mapping as deploy/robots/g1_29dof)
    Keyboard kb;

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

    auto [pos, quat] = robot_bridge->generateRandomPose();
    robot_bridge->resetRobot(pos, quat);

    // ── Transition recording ────────────────────────────────────────────
    std::ofstream csv;
    struct RowMeta { double timestamp; std::streampos pos; };
    std::deque<RowMeta> row_meta;
    size_t transition_count = 0;
    size_t reset_count = 0;
    int episode = 0;
    int step_in_ep = 0;

    if (RECORD_TRANSITIONS)
    {
        csv.open(TRANSITION_FILE);
        csv << "episode,step,"
            << "x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,"
            << "cmd_vx,cmd_vy,cmd_yaw,"
            << "next_x,next_y,next_z,next_qw,next_qx,next_qy,next_qz,"
            << "next_vx,next_vy,next_vz,next_omega_x,next_omega_y,next_omega_z,"
            << "collision\n";
        std::cout << "Recording transitions to " << TRANSITION_FILE << "\n\n";
    }

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
            if (RECORD_TRANSITIONS && !row_meta.empty())
            {
                auto now = std::chrono::steady_clock::now().time_since_epoch();
                double now_ts = std::chrono::duration<double>(now).count();
                double cutoff = now_ts - RESET_TRIM_SECONDS;

                auto trim_it = row_meta.begin();
                for (auto it = row_meta.begin(); it != row_meta.end(); ++it)
                {
                    if (it->timestamp >= cutoff) { trim_it = it; break; }
                }

                size_t trim_count = std::distance(trim_it, row_meta.end());
                std::streampos trunc_pos = trim_it->pos;
                row_meta.erase(trim_it, row_meta.end());

                csv.flush();
                csv.close();
                truncate(TRANSITION_FILE, static_cast<off_t>(trunc_pos));
                csv.open(TRANSITION_FILE, std::ios::app);
                transition_count -= trim_count;

                std::cout << "Reset: trimmed " << trim_count << " rows. ";
            }

            auto [pos, quat] = robot_bridge->generateRandomPose();
            robot_bridge->resetRobot(pos, quat);
            episode++;
            step_in_ep = 0;
            reset_count++;
            std::cout << "Robot reset (#" << reset_count << "). Transitions so far: " << transition_count << "\n";
            continue;
        }

        // ── Velocity command ──────────────────────────────────────────────
        float vx = 0.0f, vy = 0.0f, oz = 0.0f;

        // Keyboard takes priority when a movement key is held
        auto k = kb.key();
        if (k == "w")
            vx = VX_MAX;
        else if (k == "s")
            vx = VX_MIN;
        else if (k == "a")
            vy = VY_MAX;
        else if (k == "d")
            vy = VY_MIN;
        else if (k == "q")
            oz = YAW_MAX;
        else if (k == "e")
            oz = YAW_MIN;
        else if (has_joystick)
        {
            // Fallback to joystick axes; negate Y so stick-up = forward
            vx = scale_vx(-norm(js.axis_[1]));
            vy = scale_vy(norm(js.axis_[0]));
            oz = scale_yaw(norm(js.axis_[3]));
        }
        auto t0 = std::chrono::steady_clock::now();

        RobotState s0 = robot_bridge->getRobotState();

        robot_bridge->publishVelCommand({vx, vy, oz});
        robot_bridge->update();

        RobotState s1 = robot_bridge->getRobotState();
        bool collision = robot_bridge->inCollision();

        // ── Record transition ───────────────────────────────────────────
        if (RECORD_TRANSITIONS && csv.is_open())
        {
            double ts = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            std::streampos row_start = csv.tellp();

            csv << episode << "," << step_in_ep << ","
                << s0.position[0]         << "," << s0.position[1]         << "," << s0.position[2]         << ","
                << s0.orientation[0]      << "," << s0.orientation[1]      << "," << s0.orientation[2]      << "," << s0.orientation[3] << ","
                << s0.velocity[0]         << "," << s0.velocity[1]         << "," << s0.velocity[2]         << ","
                << s0.angular_velocity[0] << "," << s0.angular_velocity[1] << "," << s0.angular_velocity[2] << ","
                << vx << "," << vy << "," << oz << ","
                << s1.position[0]         << "," << s1.position[1]         << "," << s1.position[2]         << ","
                << s1.orientation[0]      << "," << s1.orientation[1]      << "," << s1.orientation[2]      << "," << s1.orientation[3] << ","
                << s1.velocity[0]         << "," << s1.velocity[1]         << "," << s1.velocity[2]         << ","
                << s1.angular_velocity[0] << "," << s1.angular_velocity[1] << "," << s1.angular_velocity[2] << ","
                << (collision ? 1 : 0)    << "\n";

            row_meta.push_back({ts, row_start});
            transition_count++;
            step_in_ep++;

            if (transition_count % 50 == 0)
                csv.flush();
        }

        std::cout << "\rCmd: [" << vx << ", " << vy << ", " << oz
                  << "]  Vel: [" << s1.velocity[0] << ", " << s1.velocity[1] << "]"
                  << "  Transitions: " << transition_count << std::flush;

        auto elapsed = std::chrono::steady_clock::now() - t0;
        auto remaining = std::chrono::milliseconds(100) - elapsed;
        if (remaining > std::chrono::milliseconds(0) && USE_WALL_CLOCK_TIME)
            std::this_thread::sleep_for(remaining);
    }

    if (RECORD_TRANSITIONS && csv.is_open())
    {
        csv.flush();
        csv.close();
        std::cout << "\n\nSaved " << transition_count << " transitions to "
                  << TRANSITION_FILE << " (resets: " << reset_count << ")\n";
    }

    return 0;
}