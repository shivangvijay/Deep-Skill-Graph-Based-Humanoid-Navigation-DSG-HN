#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <csignal>
#include <boost/program_options.hpp>
#include "unitree/robot/channel/channel_factory.hpp"
#include "robot_bridge.h"
#include "dds/vel_subscriber.h"

// ---- Configurable defaults (also overridable via --dt and --output CLI flags) ----
static constexpr double      DEFAULT_DT_SECONDS = 0.5;
static constexpr const char* DEFAULT_OUTPUT     = "transitions.csv";
// ---------------------------------------------------------------------------------

#define X_MIN -3.0f
#define X_MAX  3.0f
#define Y_MIN -3.0f
#define Y_MAX  3.0f

#define SCENE_FILE "ai_maker_space_scene.xml"

static bool g_running = true;
void handle_signal(int) { g_running = false; }

namespace po = boost::program_options;

int main(int argc, char** argv)
{
    po::options_description desc("Transition data collector");
    desc.add_options()
        ("help,h",    "show help")
        ("network,n", po::value<std::string>()->default_value("lo"),           "DDS network interface")
        ("output,o",  po::value<std::string>()->default_value(DEFAULT_OUTPUT), "output CSV file path")
        ("dt,d",      po::value<double>()->default_value(DEFAULT_DT_SECONDS),  "step duration in seconds");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) { std::cout << desc; return 0; }

    const std::string network = vm["network"].as<std::string>();
    const std::string output  = vm["output"].as<std::string>();
    const double      dt      = vm["dt"].as<double>();
    const auto        dt_ms   = std::chrono::milliseconds(static_cast<int>(dt * 1000));

    std::cout << "Initializing DDS on network: " << network << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, network);

    RobotBridge robot_bridge(network, SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX);

    VelSubscriber vel_sub;
    vel_sub.Init();

    // Give subscribers time to receive first messages
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Resetting robot to safe spawn position..." << std::endl;
    robot_bridge.resetRobot();

    // Wait for the policy to stabilize after spawn
    std::cout << "Waiting for robot to stabilize..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::signal(SIGINT, handle_signal);

    std::ofstream csv(output);
    if (!csv.is_open()) {
        std::cerr << "Failed to open output file: " << output << std::endl;
        return 1;
    }

    csv << "timestamp_s,"
        << "x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,"
        << "cmd_vx,cmd_vy,cmd_yaw,"
        << "next_x,next_y,next_z,next_qw,next_qx,next_qy,next_qz,"
        << "next_vx,next_vy,next_vz,next_omega_x,next_omega_y,next_omega_z\n";

    std::cout << "Recording transitions (dt=" << dt << "s). Pilot with w/s/a/d/q/e. Ctrl+C to stop.\n";

    size_t count = 0;
    while (g_running)
    {
        auto epoch = std::chrono::system_clock::now().time_since_epoch();
        double timestamp = std::chrono::duration<double>(epoch).count();

        RobotState s0 = robot_bridge.getRobotState();
        std::vector<float> cmd = vel_sub.getVelCmd();

        std::this_thread::sleep_for(dt_ms);

        RobotState s1 = robot_bridge.getRobotState();

        csv << timestamp << ","
            // s0 position + orientation
            << s0.position[0]      << "," << s0.position[1]      << "," << s0.position[2]      << ","
            << s0.orientation[0]   << "," << s0.orientation[1]   << "," << s0.orientation[2]   << "," << s0.orientation[3] << ","
            // s0 velocity + angular velocity
            << s0.velocity[0]      << "," << s0.velocity[1]      << "," << s0.velocity[2]      << ","
            << s0.angular_velocity[0] << "," << s0.angular_velocity[1] << "," << s0.angular_velocity[2] << ","
            // command
            << cmd[0] << "," << cmd[1] << "," << cmd[2] << ","
            // s1 position + orientation
            << s1.position[0]      << "," << s1.position[1]      << "," << s1.position[2]      << ","
            << s1.orientation[0]   << "," << s1.orientation[1]   << "," << s1.orientation[2]   << "," << s1.orientation[3] << ","
            // s1 velocity + angular velocity
            << s1.velocity[0]      << "," << s1.velocity[1]      << "," << s1.velocity[2]      << ","
            << s1.angular_velocity[0] << "," << s1.angular_velocity[1] << "," << s1.angular_velocity[2] << "\n";

        count++;
        if (count % 20 == 0) {
            std::cout << "Recorded " << count << " transitions\r" << std::flush;
        }
    }

    csv.close();
    std::cout << "\nSaved " << count << " transitions to " << output << std::endl;
    return 0;
}
