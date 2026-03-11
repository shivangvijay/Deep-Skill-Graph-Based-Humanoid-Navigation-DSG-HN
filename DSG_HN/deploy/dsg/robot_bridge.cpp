// This file contains to abstract away sending commands to the robot, and receiving the state from the simulation/robot

#include "robot_bridge.h"

RobotBridge::RobotBridge(std::string network)
{
    unitree::robot::ChannelFactory::Instance()->Init(0, network);
    vel_publisher.Init();
    state_subscriber.Init();
    reset_publisher.Init();
    readScene();
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
    std::vector<float> random_position = {0.0f, 0.0f};                // You can modify this to generate a random position
    std::vector<float> random_orientation = {1.0f, 0.0f, 0.0f, 0.0f}; // You can modify this to generate a random orientation
    reset_publisher.publishResetCommand(random_position, random_orientation);
}

void RobotBridge::resetRobot(const std::vector<float> &position, const std::vector<float> &orientation)
{
    reset_publisher.publishResetCommand(position, orientation);
}

std::vector<Obstacle> RobotBridge::getObstacles() const
{
    return obstacles;
}

void RobotBridge::readScene()
{
    std::ifstream file(scene_file);

    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open scene file: " + scene_file);
    }

    std::string line;
    while (std::getline(file, line))
    {

        if (line.find("<geom") != std::string::npos && line.find("layout_box") != std::string::npos)
        {
            Obstacle box;

            // extract the na,e
            size_t name_pos = line.find("name=\"");
            if (name_pos != std::string::npos)
            {
                size_t start = name_pos + 6; // move past name="
                size_t end = line.find("\"", start);
                box.name = line.substr(start, end - start);
            }

            // extract the pos
            size_t pos_attr = line.find("pos=\"");
            if (pos_attr != std::string::npos)
            {
                size_t start = pos_attr + 5; // move past pos="
                size_t end = line.find("\"", start);
                std::string pos_str = line.substr(start, end - start);

                // parse the string into floats using stringstream
                std::stringstream ss(pos_str);
                ss >> box.position[0] >> box.position[1] >> box.position[2]; // the >> operator moves you to the next item in the stringstream, which you can save by putting the var there

                obstacles.push_back(box);
            }
        }
    }

    file.close();
    std::cout << "Successfully parsed " << obstacles.size() << " obstacles." << std::endl;
}

// TODO next: add function that determines distance to the nearest obstacle, which can be used for spawning the robot and for the MPC cost function/constraints later on