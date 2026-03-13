#pragma once
#include <vector>
#include <string>
#include <array>
#include <memory>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <math.h>

#define DOF 35 // note that the actual DOF is 29, but the msg has 35 motors, so just going to read all 35
#define VELOCITY_POLICY_DT 0.2 // 5 Hz, this we can change later on though

struct RobotState
{
    std::array<float, DOF> q;
    std::array<float, DOF> dq;

    std::array<float, 3> position;
    std::array<float, 3> velocity;
    std::array<float, 3> accel;

    std::array<float, 4> orientation;      // quaternion
    std::array<float, 3> angular_velocity; // from the IMU
};

struct Obstacle
{
    std::array<float, 3> position;
    std::array<float, 3> size;
    std::string name;
};

class RobotBridge
{
public:
    virtual ~RobotBridge() = default;

    // Shared API used by both training and test robot bridge instances
    virtual void publishVelCommand(const std::vector<float> &cmd) = 0;
    virtual void resetRobot(const std::array<float, 3> &pos, const std::array<float, 4> &quat) = 0;
    virtual RobotState getRobotState() = 0;
    virtual void update() = 0;

    // common logic
    float distanceToNearestObstacle();
    float distanceToNearestObstacle(const std::array<float, 3> &pos, const std::array<float, 4> &quat) const;
    std::vector<Obstacle> getObstacles() const { return obstacles; }
    std::vector<float> GetCurrentCmd() const { return current_cmd; }
    void resetRobot();
    void printState(const RobotState &s) const;

protected:
    RobotBridge(std::string scene_file, float x_min, float x_max, float y_min, float y_max);

    void readScene();
    std::pair<std::array<float, 3>, std::array<float, 4>> generateRandomPos() const;

    std::string scene_file;
    std::vector<Obstacle> obstacles;
    std::vector<float> current_cmd = {0.0, 0.0, 0.0};
    float x_min = -5.0, x_max = 5.0, y_min = -5.0, y_max = 5.0;
    float velocity_policy_dt = VELOCITY_POLICY_DT;
};