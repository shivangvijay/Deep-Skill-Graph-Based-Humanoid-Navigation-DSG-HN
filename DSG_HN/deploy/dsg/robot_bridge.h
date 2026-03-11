// This file contains to abstract away sending commands to the robot, and receiving the state from the simulation/robot
#pragma once

#include "dds/vel_publisher.h"
#include <vector>
#include "dds/vel_publisher.h"
#include "unitree/robot/channel/channel_factory.hpp"
#include <string>
#include "dds/state_subscriber.h"

class RobotBridge
{
public:
    RobotBridge(std::string network);
    ~RobotBridge() = default;

    void publishVelCommand(const std::vector<float> &cmd);
    RobotState getRobotState();

private:
    VelPublisher vel_publisher;
    StateSubscriber state_subscriber;
};