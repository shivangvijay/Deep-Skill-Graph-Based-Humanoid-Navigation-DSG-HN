#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <deque>
#include <atomic>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <boost/program_options.hpp>
#include "unitree/robot/channel/channel_factory.hpp"
#include "robot_bridge.h"
#include "dds/vel_subscriber.h"

// ---- Configurable defaults (also overridable via --dt and --output CLI flags) ----
static constexpr double      DEFAULT_DT_SECONDS  = 0.05;
static constexpr const char* DEFAULT_OUTPUT      = "transitions.csv";
static constexpr double      RESET_TRIM_SECONDS  = 5.0;
// ---------------------------------------------------------------------------------

#define X_MIN -3.0f
#define X_MAX  3.0f
#define Y_MIN -3.0f
#define Y_MAX  3.0f

#define SCENE_FILE "ai_maker_space_scene.xml"

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_reset{false};

void handle_signal(int) { g_running = false; }

// ---- Raw terminal helpers ----
static termios g_orig_termios;

static void restore_terminal() { tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios); }

static void set_raw_terminal()
{
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    std::atexit(restore_terminal);
    termios raw = g_orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

// Background thread: watch stdin for 'r' keypress
static void stdin_watcher()
{
    while (g_running)
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv = {0, 100000}; // 100 ms
        if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0)
        {
            char c;
            if (read(STDIN_FILENO, &c, 1) == 1 && (c == 'r' || c == 'R'))
                g_reset = true;
        }
    }
}

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
    set_raw_terminal();
    std::thread watcher(stdin_watcher);

    std::ofstream csv(output);
    if (!csv.is_open()) {
        std::cerr << "Failed to open output file: " << output << std::endl;
        g_running = false;
        watcher.join();
        return 1;
    }

    const std::string header =
        "timestamp_s,"
        "x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,"
        "cmd_vx,cmd_vy,cmd_yaw,"
        "next_x,next_y,next_z,next_qw,next_qx,next_qy,next_qz,"
        "next_vx,next_vy,next_vz,next_omega_x,next_omega_y,next_omega_z\n";
    csv << header;

    // Track per-row file positions for trim-on-reset
    struct RowMeta { double timestamp; std::streampos start_pos; };
    std::deque<RowMeta> row_meta;

    std::cout << "Recording transitions (dt=" << dt << "s).\n"
              << "  Pilot with w/s/a/d/q/e in the g1_ctrl terminal.\n"
              << "  Press 'r' here to reset robot and trim last "
              << RESET_TRIM_SECONDS << "s of data.\n"
              << "  Ctrl+C to stop.\n";

    size_t count = 0;
    size_t resets = 0;

    while (g_running)
    {
        // ---- Handle reset request ----
        if (g_reset)
        {
            g_reset = false;

            if (row_meta.empty())
            {
                std::cout << "\nReset: no data collected yet, just repositioning robot...\n";
            }
            else
            {
                auto now = std::chrono::system_clock::now().time_since_epoch();
                double now_ts = std::chrono::duration<double>(now).count();
                double cutoff = now_ts - RESET_TRIM_SECONDS;

                // Find the first row within the trim window.
                // If all rows fall within the window (< RESET_TRIM_SECONDS collected),
                // trim_it points to row_meta.begin() and we clear everything back to the header.
                auto trim_it = row_meta.begin();
                for (auto it = row_meta.begin(); it != row_meta.end(); ++it)
                {
                    if (it->timestamp >= cutoff) { trim_it = it; break; }
                }

                std::streampos trunc_pos = trim_it->start_pos;
                size_t trim_count = std::distance(trim_it, row_meta.end());
                double trimmed_secs = row_meta.back().timestamp - trim_it->timestamp;
                row_meta.erase(trim_it, row_meta.end());

                csv.flush();
                csv.close();
                if (truncate(output.c_str(), static_cast<off_t>(trunc_pos)) != 0)
                    std::cerr << "Warning: truncate() failed\n";
                csv.open(output, std::ios::app);
                count -= trim_count;

                std::cout << "\nReset: trimmed " << trim_count << " rows ("
                          << trimmed_secs << "s). Repositioning robot...\n";
            }

            robot_bridge.resetRobot();
            std::cout << "Waiting for robot to stabilize...\n";
            std::this_thread::sleep_for(std::chrono::seconds(2));
            ++resets;
            std::cout << "Recording resumed. Total resets: " << resets << "\n";
            continue;
        }

        // ---- Record one transition ----
        auto epoch = std::chrono::system_clock::now().time_since_epoch();
        double timestamp = std::chrono::duration<double>(epoch).count();

        RobotState s0 = robot_bridge.getRobotState();
        std::vector<float> cmd = vel_sub.getVelCmd();

        std::this_thread::sleep_for(dt_ms);

        RobotState s1 = robot_bridge.getRobotState();

        std::streampos row_start = csv.tellp();
        csv << timestamp << ","
            << s0.position[0]         << "," << s0.position[1]         << "," << s0.position[2]         << ","
            << s0.orientation[0]      << "," << s0.orientation[1]      << "," << s0.orientation[2]      << "," << s0.orientation[3] << ","
            << s0.velocity[0]         << "," << s0.velocity[1]         << "," << s0.velocity[2]         << ","
            << s0.angular_velocity[0] << "," << s0.angular_velocity[1] << "," << s0.angular_velocity[2] << ","
            << cmd[0] << "," << cmd[1] << "," << cmd[2] << ","
            << s1.position[0]         << "," << s1.position[1]         << "," << s1.position[2]         << ","
            << s1.orientation[0]      << "," << s1.orientation[1]      << "," << s1.orientation[2]      << "," << s1.orientation[3] << ","
            << s1.velocity[0]         << "," << s1.velocity[1]         << "," << s1.velocity[2]         << ","
            << s1.angular_velocity[0] << "," << s1.angular_velocity[1] << "," << s1.angular_velocity[2] << "\n";

        row_meta.push_back({timestamp, row_start});
        ++count;

        if (count % 20 == 0)
            std::cout << "Recorded " << count << " transitions\r" << std::flush;
    }

    g_running = false;
    watcher.join();
    csv.close();
    std::cout << "\nSaved " << count << " transitions to " << output
              << " (resets: " << resets << ")\n";
    return 0;
}
