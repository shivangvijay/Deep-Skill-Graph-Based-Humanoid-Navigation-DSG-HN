#include "build_dsg.h"

int main(int argc, char** argv)
{
    std::cout << "Starting DSG build..." << std::endl;
    auto vm = param::helper(argc, argv);

    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    RobotBridge robot_bridge(vm["network"].as<std::string>());

    while (true)
    {
        // Example velocity command to publish
        std::vector<float> cmd = {2.0f, 0.0f, 0.0f}; // Move forward at 0.5 m/s
        robot_bridge.publishVelCommand(cmd);
        //std::cout << "Published velocity command: " << cmd[0] << " " << cmd[1] << " " << cmd[2] << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Publish every second
    }
}