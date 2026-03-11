#include "build_dsg.h"

int main(int argc, char** argv)
{
    std::cout << "Starting DSG build..." << std::endl;
    auto vm = param::helper(argc, argv);

    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    RobotBridge robot_bridge(vm["network"].as<std::string>());

    int steps = 0;
    robot_bridge.resetRobot();
    while (true)
    {
        // Example velocity command to publish
        std::vector<float> cmd = {1.0f, 0.0f, 0.0f}; // Move forward at 1.0 m/s
        robot_bridge.publishVelCommand(cmd);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (steps % 10 == 0)
        {
            robot_bridge.resetRobot(); // Reset the robot every 10 steps
        }
        steps++;
    }
}