#pragma once
#include "robot_bridge.h"
#include "dds/vel_publisher.h"
#include "dds/state_subscriber.h"
#include "dds/reset_publisher.h"
#include <unitree/robot/channel/channel_factory.hpp>
#include <thread>

class RobotBridgeDDS : public RobotBridge
{
public:
    RobotBridgeDDS(std::string scene_file,
                   float x_min, float x_max, float y_min, float y_max, std::string network);

    virtual ~RobotBridgeDDS() = default;

    void publishVelCommand(const std::vector<float> &cmd) override;
    void resetRobot() override;
    void resetRobot(const std::array<float, 3> &pos, const std::array<float, 4> &quat) override;
    RobotState getRobotState() override;
    void update() override;

private:
    VelPublisher vel_publisher;
    StateSubscriber state_subscriber;
    ResetPublisher reset_publisher;
};