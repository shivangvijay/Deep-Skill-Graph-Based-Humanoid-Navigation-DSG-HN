#include "robot_bridge.h"

RobotBridge::RobotBridge(std::string scene_file, float x_min, float x_max, float y_min, float y_max)
    : scene_file(scene_file), x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max)
{
    readScene();
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
            bool found_name, found_pos, found_size = false;
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

float RobotBridge::distanceToNearestObstacle(const std::array<float, 3> &pos, const std::array<float, 4> &quat) const
{
    float min_dist = std::numeric_limits<float>::max();
    for (const auto &obs : obstacles)
    {
        float dx = std::max(std::abs(pos[0] - obs.position[0]) - obs.size[0], 0.0f);
        float dy = std::max(std::abs(pos[1] - obs.position[1]) - obs.size[1], 0.0f);
        min_dist = std::min(min_dist, std::sqrt(dx * dx + dy * dy));
    }
    return min_dist;
}

std::pair<std::array<float, 3>, std::array<float, 4>> RobotBridge::generateRandomPos() const
{
    std::array<float, 3> pos = {
        x_min + static_cast<float>(rand()) / (RAND_MAX / (x_max - x_min)),
        y_min + static_cast<float>(rand()) / (RAND_MAX / (y_max - y_min)),
        0.0f};
    float yaw = (static_cast<float>(rand()) / RAND_MAX) * 2 * M_PI;
    return {pos, {cosf(yaw / 2), 0, 0, sinf(yaw / 2)}};
}