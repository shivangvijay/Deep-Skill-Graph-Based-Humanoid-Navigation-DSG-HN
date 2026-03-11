// This file contains to abstract away sending commands to the robot, and receiving the state from the simulation/robot

#include "robot_bridge.h"

RobotBridge::RobotBridge(std::string network)
{
    unitree::robot::ChannelFactory::Instance()->Init(0, network);
    vel_publisher.Init();
    state_subscriber.Init();
}

void RobotBridge::publishVelCommand(const std::vector<float> &cmd)
{
    vel_publisher.publishVelCommand(cmd);
}

RobotState RobotBridge::getRobotState()
{
    return state_subscriber.GetRobotState();
}