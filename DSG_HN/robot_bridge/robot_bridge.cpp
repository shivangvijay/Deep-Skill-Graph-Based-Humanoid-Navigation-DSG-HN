#include "robot_bridge.h"

RobotBridge::RobotBridge(std::string scene_file, float x_min, float x_max, float y_min, float y_max)
    : scene_file(scene_file), x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max)
{
    readScene();
}
void RobotBridge::readScene() // assuming cylindrical obstacles!
{
    std::ifstream file(scene_file);

    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open scene file: " + scene_file);
    }

    std::string line;
    while (std::getline(file, line))
    {

        if (line.find("<geom") != std::string::npos && line.find("layout_") != std::string::npos)
        {
            bool found_type, found_pos, found_size = false;
            Obstacle obs;

            size_t type_pos = line.find("type=\"");
            if (type_pos != std::string::npos)
            {
                size_t start = type_pos + 6; // move past name="
                size_t end = line.find("\"", start);
                obs.type = line.substr(start, end - start);
                found_type = true;
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
                ss >> obs.position[0] >> obs.position[1] >> obs.position[2]; // the >> operator moves you to the next item in the stringstream, which you can save by putting the var there
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
                ss >> obs.size[0] >> obs.size[1];
                found_size = true;
            }

            if (found_type && found_pos && found_size)
            {
                obstacles.push_back(obs);
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
        float dx = pos[0] - obs.position[0];
        float dy = pos[1] - obs.position[1];
        float center_dist = std::sqrt(dx * dx + dy * dy);

        float actual_dist = center_dist - obs.size[0];

        min_dist = std::min(min_dist, actual_dist);
    }
    return min_dist;
}

// scalar w is first element
std::pair<std::array<float, 3>, std::array<float, 4>> RobotBridge::generateRandomPose() const
{
    auto [pos, quat, _, __] = generateRandomPoseWithVel();

    return {pos, quat};
}

std::tuple<std::array<float, 3>, std::array<float, 4>, std::array<float, 3>, std::array<float, 3>> RobotBridge::generateRandomPoseWithVel() const
{
    std::array<float, 3> pos;
    std::array<float, 4> quat;
    std::array<float, 3> vel;
    std::array<float, 3> ang_vel;
    int attempts = 0;
    do
    {
        if (attempts++ > 1000)
            throw std::runtime_error("Could Not Respawn Robot");
        pos = {
            x_min + static_cast<float>(rand()) / (RAND_MAX / (x_max - x_min)),
            y_min + static_cast<float>(rand()) / (RAND_MAX / (y_max - y_min)),
            0.0f};

        float yaw = (static_cast<float>(rand()) / RAND_MAX) * 2 * M_PI;
        quat = {cosf(yaw / 2), 0, 0, sinf(yaw / 2)};

        vel = {
            -vel_limits[0] + static_cast<float>(rand()) / (RAND_MAX / (2 * vel_limits[0])),
            -vel_limits[1] + static_cast<float>(rand()) / (RAND_MAX / (2 * vel_limits[1])),
            0.0f};

        float ang_vel_yaw = -vel_limits[2] + static_cast<float>(rand()) / (RAND_MAX / (2 * vel_limits[2]));
        ang_vel = {0.0, 0.0, ang_vel_yaw};
    } while (distanceToNearestObstacle(pos, quat) < 0.5);

    return {pos, quat, vel, ang_vel};
}

void RobotBridge::printState(const RobotState &s) const // just a lil utility function to help with debuffing
{
    std::cout << "--- Robot State ---" << std::endl;

    std::cout << "Joint Positions: " << std::endl;
    for (size_t i = 0; i < s.q.size(); i++)
    {
        std::cout << s.q[i] << (i == s.q.size() - 1 ? "" : ", ");
    }
    std::cout << std::endl;

    std::cout << "Joint Velocities: " << std::endl;
    for (size_t i = 0; i < s.dq.size(); i++)
    {
        std::cout << s.dq[i] << (i == s.dq.size() - 1 ? "" : ", ");
    }
    std::cout << std::endl;

    std::cout << "Base Position: [" << s.position[0] << ", " << s.position[1] << ", " << s.position[2] << "]" << std::endl;
    std::cout << "Base Velocity: [" << s.velocity[0] << ", " << s.velocity[1] << ", " << s.velocity[2] << "]" << std::endl;

    std::cout << "Orientation (Quat): [" << s.orientation[0] << ", " << s.orientation[1] << ", "
              << s.orientation[2] << ", " << s.orientation[3] << "]" << std::endl;

    std::cout << "Angular Velocity: [" << s.angular_velocity[0] << ", " << s.angular_velocity[1] << ", "
              << s.angular_velocity[2] << "]" << std::endl;

    std::cout << "Acceleration: [" << s.accel[0] << ", " << s.accel[1] << ", " << s.accel[2] << "]" << std::endl;
    std::cout << "-------------------" << std::endl;
}