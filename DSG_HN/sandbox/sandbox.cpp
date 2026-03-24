#define private public
#include "glfw_adapter.h"
#undef private
#include "simulate.h"
#include "array_safety.h"

namespace mj = ::mujoco;

#include "robot_bridge_train.h"
#include "param.h"
#include "joystick/joystick.h"
#include <iostream>
#include <cmath>
#include <csignal>
#include <atomic>
#include <thread>

#define SCENE_FILE "umaze_scene.xml"

// Fixed spawn: bottom-left corner of the arena, clear of all obstacles and walls.
// Position (-7, -7) is >3m from the nearest cylinder and well south of the maze wall (y=1.5).
// Quaternion (w=1,x=0,y=0,z=0) = identity orientation, facing +x.
static const std::array<float, 3> SPAWN_POS  = {-7.0f, -7.0f, 0.0f};
static const std::array<float, 4> SPAWN_QUAT = {1.0f, 0.0f, 0.0f, 0.0f};

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

// Velocity command shared between key callback and physics thread
static std::atomic<float> g_vx{0.0f};
static std::atomic<float> g_vy{0.0f};
static std::atomic<float> g_oz{0.0f};
static std::atomic<bool>  g_reset{false};
static std::atomic<bool>  g_quit{false};

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

// GLFW key callback — runs on the main thread inside RenderLoop
void key_cb(GLFWwindow* /*window*/, int key, int /*scancode*/, int action, int /*mods*/)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key)
        {
            case GLFW_KEY_W: g_vx = VX_MAX;  g_vy = 0.0f;    g_oz = 0.0f;    break;
            case GLFW_KEY_S: g_vx = VX_MIN;  g_vy = 0.0f;    g_oz = 0.0f;    break;
            case GLFW_KEY_A: g_vx = 0.0f;    g_vy = VY_MAX;  g_oz = 0.0f;    break;
            case GLFW_KEY_D: g_vx = 0.0f;    g_vy = VY_MIN;  g_oz = 0.0f;    break;
            case GLFW_KEY_Q: g_vx = 0.0f;    g_vy = 0.0f;    g_oz = YAW_MAX; break;
            case GLFW_KEY_E: g_vx = 0.0f;    g_vy = 0.0f;    g_oz = YAW_MIN; break;
            case GLFW_KEY_R: g_reset = true; break;
            case GLFW_KEY_0: g_quit  = true; break;
            default: break;
        }
    }
    if (action == GLFW_RELEASE)
    {
        switch (key)
        {
            case GLFW_KEY_W: case GLFW_KEY_S: g_vx = 0.0f; break;
            case GLFW_KEY_A: case GLFW_KEY_D: g_vy = 0.0f; break;
            case GLFW_KEY_Q: case GLFW_KEY_E: g_oz = 0.0f; break;
            default: break;
        }
    }
}

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);

    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    // Build robot bridge with render=false — mj::Simulate owns the window
    auto robot_bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, -10.0f, 10.0f, -10.0f, 10.0f, policy_dir, false);

    // Set up interactive viewer
    mjvCamera cam;  mjv_defaultCamera(&cam);
    mjvOption  opt; mjv_defaultOption(&opt);
    mjvPerturb pert; mjv_defaultPerturb(&pert);

    auto sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    // Joystick (optional)
    Joystick js(JS_DEVICE);
    bool has_joystick = js.isFound();
    if (!has_joystick)
        std::cout << "No joystick found at " << JS_DEVICE << " — keyboard only.\n";

    std::cout << "\nSandbox ready (interactive MuJoCo viewer).\n"
              << "Keyboard:\n"
              << "  w / s       -> forward / backward (vx)\n"
              << "  a / d       -> strafe left / right (vy)\n"
              << "  q / e       -> rotate left / right (oz)\n"
              << "  r           -> reset robot\n"
              << "  0           -> quit\n"
              << "Mouse:        -> orbit / pan / zoom (MuJoCo viewer)\n\n";

    // Physics thread: load model into sim, then run control loop
    // sim->Load() MUST be called from this thread, not from main, so that
    // RenderLoop() (which starts after this thread) sees the model correctly.
    std::thread physics([&]()
    {
        // Hand model and data to the viewer from the physics thread
        sim->LoadMessage(SCENE_FILE);
        sim->Load(robot_bridge->getModel(), robot_bridge->getData(), SCENE_FILE);

        // Spawn robot at fixed bottom-left position, clear of all obstacles
        {
            const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
            robot_bridge->resetRobot(SPAWN_POS, SPAWN_QUAT);
        }

        while (!sim->exitrequest.load() && !g_quit.load())
        {
            if (has_joystick)
                js.getState();

            float vx = g_vx.load();
            float vy = g_vy.load();
            float oz = g_oz.load();

            if (has_joystick && vx == 0.0f && vy == 0.0f && oz == 0.0f)
            {
                vx = scale_vx ( -norm(js.axis_[1]));
                vy = scale_vy (  norm(js.axis_[0]));
                oz = scale_yaw(  norm(js.axis_[3]));

                if (js.button_[6]) g_reset = true;
                if (js.button_[7]) g_quit  = true;
            }

            if (g_reset.exchange(false))
            {
                const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
                robot_bridge->resetRobot(SPAWN_POS, SPAWN_QUAT);
                continue;
            }

            {
                auto t0 = std::chrono::steady_clock::now();

                {
                    const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
                    robot_bridge->publishVelCommand({vx, vy, oz});
                    robot_bridge->update();
                }

                // update() advances velocity_policy_dt (0.1s) of sim time with no wall-clock
                // pacing when render=false. Sleep the remainder so the robot moves at real-time.
                auto elapsed = std::chrono::steady_clock::now() - t0;
                auto remaining = std::chrono::milliseconds(100) - elapsed;
                if (remaining > std::chrono::milliseconds(0))
                    std::this_thread::sleep_for(remaining);
            }
        }
        sim->exitrequest.store(1);
    });

    // Register key callback on the sim window (window exists after Simulate constructor)
    glfwSetKeyCallback(
        static_cast<mj::GlfwAdapter*>(sim->platform_ui.get())->window_,
        key_cb);

    // Blocking: runs the MuJoCo interactive viewer on the main thread (required by GLFW/macOS)
    sim->RenderLoop();

    physics.join();
    return 0;
}
