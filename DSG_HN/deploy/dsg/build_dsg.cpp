#include "build_dsg.h"
#include <thread>
#include <chrono>
#include <iostream>
#include "unitree/robot/channel/channel_factory.hpp"
#include "param.h"
#include <string>

int main(int argc, char** argv)
{
    std::cout << "Starting DSG build..." << std::endl;
    auto vm = param::helper(argc, argv);

    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    VelPublisher vel_publisher;
    vel_publisher.Init();

    while (true)
    {
        // Example velocity command to publish
        std::vector<float> cmd = {0.5f, 0.0f, 0.0f}; // Move forward at 0.5 m/s
        vel_publisher.publishVelCommand(cmd);
        std::cout << "Published velocity command: " << cmd[0] << " " << cmd[1] << " " << cmd[2] << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Publish every second
    }
}