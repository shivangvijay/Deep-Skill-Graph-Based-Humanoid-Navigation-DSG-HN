#include "robot_bridge_train.h"
#include "param.h"
#include "joystick/joystick.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include <iostream>
#include <cmath>
#include <csignal>
#include <atomic>

#define SCENE_FILE "test_scene.xml"

// Match policy training ranges from deploy.yaml
// commands.base_velocity.ranges: lin_vel_x[-0.5,1.0], lin_vel_y[-0.3,0.3], ang_vel_z[-0.2,0.2]
#define VX_MAX   1.0f
#define VX_MIN  -0.5f
#define VY_MAX   0.3f
#define VY_MIN  -0.3f
#define YAW_MAX  0.2f
#define YAW_MIN -0.2f

// Joystick device
#define JS_DEVICE    "/dev/input/js0"
#define JS_MAX_AXIS  32767.0f
#define DEADZONE     0.05f

static std::atomic<bool> running{true};

static void sigint_handler(int) { running = false; }

// Normalise raw joystick axis to [-1, 1] with deadzone
static float norm(int raw)
{
    float v = float(raw) / JS_MAX_AXIS;
    return (std::fabs(v) < DEADZONE) ? 0.0f : v;
}

// Scale normalised joystick value asymmetrically to match policy range
static float scale_vx(float n)  { return n >= 0.0f ? n * VX_MAX  : n * -VX_MIN;  }
static float scale_vy(float n)  { return n >= 0.0f ? n * VY_MAX  : n * -VY_MIN;  }
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

        // Keyboard takes priority when a movement key is held
        auto k = kb.key();
        if      (k == "w") vx = VX_MAX;
        else if (k == "s") vx = VX_MIN;
        else if (k == "a") vy = VY_MAX;
        else if (k == "d") vy = VY_MIN;
        else if (k == "q") oz = YAW_MAX;
        else if (k == "e") oz = YAW_MIN;
        else if (has_joystick)
        {
            // Fallback to joystick axes; negate Y so stick-up = forward
            vx = scale_vx ( -norm(js.axis_[1]));
            vy = scale_vy (  norm(js.axis_[0]));
            oz = scale_yaw(  norm(js.axis_[3]));
        }

        robot_bridge->publishVelCommand({vx, vy, oz});
        robot_bridge->update();
        auto state = robot_bridge->getRobotState();

        std::cout << "Command: " << vx << ", " << vy << ", " << oz << std::endl;
        std::cout << "Velocity: " << state.velocity[0] << ", " << state.velocity[1] << ", " << state.velocity[2] << std::endl;
    }

    return 0;
}
