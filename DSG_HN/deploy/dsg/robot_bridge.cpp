// This file contains to abstract away sending commands to the robot, and receiving the state from the simulation/robot

#include "robot_bridge.h"

RobotBridge::RobotBridge(std::string network)
{
    unitree::robot::ChannelFactory::Instance()->Init(0, network);
    vel_publisher.Init();
    state_subscriber.Init();
    reset_publisher.Init();
}

void RobotBridge::publishVelCommand(const std::vector<float> &cmd)
{
    vel_publisher.publishVelCommand(cmd);
}

RobotState RobotBridge::getRobotState()
{
    return state_subscriber.GetRobotState();
}

void RobotBridge::resetRobot()
{
    std::vector<float> random_position = {0.0f, 0.0f}; // You can modify this to generate a random position
    std::vector<float> random_orientation = {1.0f, 0.0f, 0.0f, 0.0f}; // You can modify this to generate a random orientation
    reset_publisher.publishResetCommand(random_position, random_orientation);
}

void RobotBridge::resetRobot(const std::vector<float> &position, const std::vector<float> &orientation)
{
    reset_publisher.publishResetCommand(position, orientation);
}