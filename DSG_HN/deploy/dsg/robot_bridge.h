// This file contains to abstract away sending commands to the robot, and receiving the state from the simulation/robot
#pragma once

#include "dds/vel_publisher.h"
#include <vector>
#include "dds/vel_publisher.h"
#include "unitree/robot/channel/channel_factory.hpp"
#include <string>
#include "dds/state_subscriber.h"
#include "dds/reset_publisher.h"
#include <fstream>
#include <stdexcept>

#define SCENE_FILE "ai_maker_space_scene.xml"

struct Obstacle
{
    std::array<float, 3> position;
    std::array<float, 3> size;
    std::string name;
};

class RobotBridge
{
public:
    RobotBridge(std::string network);
    ~RobotBridge() = default;

    void publishVelCommand(const std::vector<float> &cmd);
    void resetRobot(); // random reset
    void resetRobot(const std::vector<float> &position, const std::vector<float> &orientation); // reset to a specific pose
    RobotState getRobotState();
    std::vector<Obstacle> getObstacles() const;

private:
    VelPublisher vel_publisher;
    StateSubscriber state_subscriber;
    ResetPublisher reset_publisher;
    std::string scene_file = SCENE_FILE;
    std::vector<Obstacle> obstacles;

    void readScene();
};