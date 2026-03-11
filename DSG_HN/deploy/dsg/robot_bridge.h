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
#include <math.h>
#include <random>
#include <utility>
#include <sstream>

struct Obstacle
{
    std::array<float, 3> position;
    std::array<float, 3> size;
    std::string name;
};

class RobotBridge
{
public:
    RobotBridge(std::string network, std::string scene_file, float x_min, float x_max, float y_min, float y_max);
    ~RobotBridge() = default;

    void publishVelCommand(const std::vector<float> &cmd);
    void resetRobot();                                                                          // random reset
    void resetRobot(const std::array<float, 3> &position, const std::array<float, 4> &orientation); // reset to a specific pose
    RobotState getRobotState();
    std::vector<Obstacle> getObstacles() const;
    float distanceToNearestObstacle(const std::array<float, 3> &position, const std::array<float, 4> &orientation) const; // TODO: implement this function to calculate the distance from the given position to the nearest obstacle
    float distanceToNearestObstacle();                                                                                    // uses curent robot state

private:
    VelPublisher vel_publisher;
    StateSubscriber state_subscriber;
    ResetPublisher reset_publisher;
    std::string scene_file;
    std::vector<Obstacle> obstacles;
    float x_min, x_max, y_min, y_max;

    std::pair<std::array<float, 3>, std::array<float, 4>> generateRandomPos() const;
    void readScene();
};