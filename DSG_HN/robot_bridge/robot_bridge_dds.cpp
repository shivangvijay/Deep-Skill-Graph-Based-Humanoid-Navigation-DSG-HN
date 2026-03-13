#include "robot_bridge_dds.h"

RobotBridgeDDS::RobotBridgeDDS(std::string scene_file, float x_min, float x_max, float y_min, float y_max, std::string network)
    : RobotBridge(scene_file, x_min, x_max, y_min, y_max)
{
    unitree::robot::ChannelFactory::Instance()->Init(0, network);

    vel_publisher.Init();
    state_subscriber.Init();
    reset_publisher.Init();
}

void RobotBridgeDDS::publishVelCommand(const std::vector<float> &cmd)
{
    current_cmd = cmd;
    vel_publisher.publishVelCommand(cmd);
}

void RobotBridgeDDS::update()
{
    // TODO
    // Need to make frequency match that of our trained policy, so we add a sleep here
    // to let some time pass. Right now set to 5Hz, but in the future need to make this updatable
    std::this_thread::sleep_for(std::chrono::milliseconds(int(VELOCITY_POLICY_DT * 1000)));
}

void RobotBridgeDDS::resetRobot(const std::array<float, 3> &pos, const std::array<float, 4> &quat)
{
    std::vector<float> p(pos.begin(), pos.end());
    std::vector<float> q(quat.begin(), quat.end());
    reset_publisher.publishResetCommand(p, q);
}

RobotState RobotBridgeDDS::getRobotState()
{
    return state_subscriber.GetRobotState();
}