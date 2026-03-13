#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include "param.h"
#include "robot_bridge_dds.h"

#define X_MIN -10.0f
#define X_MAX 10.0f
#define Y_MIN -10.0f
#define Y_MAX 10.0f

#define SCENE_FILE "ai_maker_space_scene.xml"

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);

    // TODO: pass vm in as a param both here and in the train version, and then you can just directly choose whether to use training
    // or not with a factory archetype
    RobotBridgeDDS robot_bridge(SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, vm["network"].as<std::string>());

    int steps = 0;
    robot_bridge.resetRobot();
    while (true)
    {
        // Example velocity command to publish
        std::vector<float> cmd = {1.0f, 0.0f, 1.0f}; 
        robot_bridge.publishVelCommand(cmd);
        robot_bridge.update(); // sleeps so vel published at 50 Hz, matching low level control. In future, need to decide what to set this at
        if (steps % 10 == 0)
        {
            robot_bridge.resetRobot();
        }
        steps++;
    }
}