// This file contains to abstract away sending commands to the robot, and receiving the state from the simulation/robot

#include "robot_bridge.h"

RobotBridge::RobotBridge(std::string network, std::string scene_file, float x_min, float x_max, float y_min, float y_max) : scene_file(scene_file), x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max)
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

std::pair<std::array<float, 3>, std::array<float, 4>> RobotBridge::generateRandomPos() const
{
    std::array<float, 3> random_position = {0, 0, 0};
    random_position[0] = x_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (x_max - x_min)));
    random_position[1] = y_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (y_max - y_min)));

    std::array<float, 4> random_orientation = {0, 0, 0, 0};
    float random_yaw = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 * M_PI; // generate random yaw between 0 and 2 pi
    random_orientation[0] = cos(random_yaw / 2);                                             // convert yaw to quaternion
    random_orientation[3] = sin(random_yaw / 2);
    return {random_position, random_orientation};
}

void RobotBridge::resetRobot()
{
    auto [random_position, random_orientation] = generateRandomPos();

    // TODO: need a finer grain way to check distance, since robot is not a point mass
    // to start, this 
    while (distanceToNearestObstacle(random_position, random_orientation) < 2.0f)
    {
        std::tie(random_position, random_orientation) = generateRandomPos();
    }
    std::cout << "Distance to nearest obstacle: " << distanceToNearestObstacle(random_position, random_orientation) << std::endl;
    std::vector<float> random_position_vec(random_position.begin(), random_position.end());
    std::vector<float> random_orientation_vec(random_orientation.begin(), random_orientation.end());
    reset_publisher.publishResetCommand(random_position_vec, random_orientation_vec);
}

void RobotBridge::resetRobot(const std::array<float, 3> &position, const std::array<float, 4> &orientation)
{
    std::vector<float> random_position_vec(position.begin(), position.end());
    std::vector<float> random_orientation_vec(orientation.begin(), orientation.end());
    reset_publisher.publishResetCommand(random_position_vec, random_orientation_vec);
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
            bool found_name, found_pos, found_size  = false;
            Obstacle box;

            size_t name_pos = line.find("name=\"");
            if (name_pos != std::string::npos)
            {
                size_t start = name_pos + 6; // move past name="
                size_t end = line.find("\"", start);
                box.name = line.substr(start, end - start);
                found_name = true;
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
                found_pos = true;
            }

            // extract the size
            size_t size_attr = line.find("size=\"");
            if (size_attr != std::string::npos)
            {
                size_t start = size_attr + 6; // move past size="
                size_t end = line.find("\"", start);
                std::string size_str = line.substr(start, end - start);

                // parse the string into floats using stringstream
                std::stringstream ss(size_str);
                ss >> box.size[0] >> box.size[1] >> box.size[2];
                found_size = true;
            }

            if (found_name && found_pos && found_size)
            {
                obstacles.push_back(box);
            }
        }
    }

    file.close();
    std::cout << "Successfully parsed " << obstacles.size() << " obstacles." << std::endl;
}

float RobotBridge::distanceToNearestObstacle()
{
    RobotState state = getRobotState();
    return distanceToNearestObstacle(state.position, state.orientation);
}

float RobotBridge::distanceToNearestObstacle(const std::array<float, 3> &position, const std::array<float, 4> &orientation) const
{
    float min_distance = std::numeric_limits<float>::max();
    for (const auto &obstacle : obstacles)
    {
        float dx = std::max(std::abs(position[0] - obstacle.position[0]) - obstacle.size[0], 0.0f);
        float dy = std::max(std::abs(position[1] - obstacle.position[1]) - obstacle.size[1], 0.0f);
        float distance = std::sqrt(dx * dx + dy * dy);
        min_distance = std::min(min_distance, distance);
    }
    return min_distance;
}