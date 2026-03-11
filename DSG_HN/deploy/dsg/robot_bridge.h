// This file contains to abstract away sending commands to the robot, and receiving the state from the simulation/robot
#pragma once

#include "dds/vel_publisher.h"
#include <vector>
#include "dds/vel_publisher.h"
#include "unitree/robot/channel/channel_factory.hpp"
#include <string>
#include "dds/state_subscriber.h"
#include "dds/reset_publisher.h"

// TODO: Add function to reset the robot, and ensure that the robot is stable before sending commands
class RobotBridge
{
public:
    RobotBridge(std::string network);
    ~RobotBridge() = default;

    void publishVelCommand(const std::vector<float> &cmd);
    void resetRobot(); // random reset
    void resetRobot(const std::vector<float> &position, const std::vector<float> &orientation); // reset to a specific pose
    RobotState getRobotState();

private:
    VelPublisher vel_publisher;
    StateSubscriber state_subscriber;
    ResetPublisher reset_publisher;
};