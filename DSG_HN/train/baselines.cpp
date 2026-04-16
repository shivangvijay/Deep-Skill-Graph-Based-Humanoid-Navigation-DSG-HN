#include "baselines.h"

// Public
PurePursuitBaseline::PurePursuitBaseline(std::shared_ptr<RobotBridgeTrain> robot_bridge, float discretization, float robot_radius, float lookahead_distance)
    : _robot_bridge(robot_bridge), _discretization(discretization), _robot_radius(robot_radius), _lookahead_distance(lookahead_distance),
      _env(std::make_shared<TrainEnvironment>(robot_bridge, 1000))

{
    _readScene(robot_radius, discretization);
}

float PurePursuitBaseline::execute(const AbstractedState &start, const AbstractedState &goal)
{
    _env->resetTo(start);
    _env->setGoal(goal);
    auto waypoints = _generateWaypoint(start, goal);
    float cum_reward = 0.0f;
    bool env_done = false;

    // simple pure pursuit controller to follow waypoints
    for (int i = 0; i < waypoints.size(); i++)
    {
        if (env_done)
            break;
        auto [wx, wy] = waypoints[i];
        AbstractedState waypoint_state;
        waypoint_state.position = {wx, wy, 0};
        waypoint_state.orientation = {1, 0, 0, 0};
        _env->updateGoalMarker(waypoint_state);
               
        while (true)
        {
            RobotState state = _robot_bridge->getRobotState();
            float dx = wx - state.position[0];
            float dy = wy - state.position[1];
            float distance = std::sqrt(dx * dx + dy * dy);

            if ((distance < _lookahead_distance && i != waypoints.size() - 1) || env_done)
            {
                break;
            }

            float current_yaw = 2.0f * std::atan2(state.orientation[3], state.orientation[0]);

            float target_angle = std::atan2(dy, dx);

            float angle_diff = target_angle - current_yaw;
            while (angle_diff > M_PI)
                angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI)
                angle_diff += 2 * M_PI;

            // if performance is off, this is another place to look regarding tuning
            float v_scale = std::max(0.0f, std::cos(angle_diff));
            float linear_vel = std::min(0.75f, distance) * v_scale;

            float angular_vel = std::clamp(angle_diff * 2.0f, -1.0f, 1.0f);

            auto [next_state, reward, done] = _env->step(torch::tensor({linear_vel, 0.0f, angular_vel}));
            cum_reward += reward.item<float>();
            env_done = done.item<bool>();
        }
    }
    return cum_reward;
}

// Private
float PurePursuitBaseline::_h(const std::pair<float, float> pos, const std::pair<float, float> goal)
{
    return std::sqrt(std::pow(goal.first - pos.first, 2) + std::pow(goal.second - pos.second, 2));
}

std::vector<std::pair<float, float>> PurePursuitBaseline::_generateWaypoint(const AbstractedState &start, const AbstractedState &goal)
{
    int start_x = static_cast<int>((start.position[0] - _robot_bridge->x_min) / _discretization);
    int start_y = static_cast<int>((start.position[1] - _robot_bridge->y_min) / _discretization);
    int goal_x = static_cast<int>((goal.position[0] - _robot_bridge->x_min) / _discretization);
    int goal_y = static_cast<int>((goal.position[1] - _robot_bridge->y_min) / _discretization);

    std::vector<std::pair<float, float>> waypoints;

    std::vector<std::vector<bool>> closed_set(_grid.size(), std::vector<bool>(_grid[0].size(), false));
    std::priority_queue<std::tuple<float, int, int>, std::vector<std::tuple<float, int, int>>, std::greater<>> open_list;

    std::vector<std::vector<float>> g_values(_grid.size(), std::vector<float>(_grid[0].size(), std::numeric_limits<float>::infinity()));

    open_list.push({0.0f, start_x, start_y});
    g_values[start_y][start_x] = 0.0f;

    std::vector<std::pair<int, int>> directions = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

    while (!open_list.empty())
    {
        auto [f, x, y] = open_list.top();
        open_list.pop();

        if (closed_set[y][x])
            continue;

        closed_set[y][x] = true;

        if (x == goal_x && y == goal_y)
        {
            // backtrack
            std::vector<std::pair<int, int>> path;
            path.emplace_back(x, y);
            while (!(x == start_x && y == start_y))
            {
                float min_g = std::numeric_limits<float>::infinity();
                std::pair<int, int> best_prev;
                for (const auto &dir : directions)
                {
                    int prev_x = x + dir.first;
                    int prev_y = y + dir.second;
                    if (prev_x >= 0 && prev_x < _grid[0].size() && prev_y >= 0 && prev_y < _grid.size())
                    {
                        if (g_values[prev_y][prev_x] < min_g)
                        {
                            min_g = g_values[prev_y][prev_x];
                            best_prev = {prev_x, prev_y};
                        }
                    }
                }
                path.push_back(best_prev);
                x = best_prev.first;
                y = best_prev.second;
            }
            std::reverse(path.begin(), path.end());
            for (const auto &[px, py] : path)
            {
                waypoints.push_back(_convertGridToWorld({px, py}));
            }
            break;
        }

        for (const auto &dir : directions)
        {
            int new_x = x + dir.first;
            int new_y = y + dir.second;

            if (new_x >= 0 && new_x < _grid[0].size() && new_y >= 0 && new_y < _grid.size() && _grid[new_y][new_x] == 0 && !closed_set[new_y][new_x])
            {
                float tentative_g = g_values[y][x] + std::sqrt(dir.first * dir.first + dir.second * dir.second) * _discretization; // move cost is sqrt 2 for diag, 1 for straight
                if (tentative_g < g_values[new_y][new_x])
                {
                    g_values[new_y][new_x] = tentative_g;
                    open_list.push({tentative_g + _h({new_x, new_y}, {goal_x, goal_y}), new_x, new_y});
                }
            }
        }
    }
    if (waypoints.empty())
    {
        std::cerr << "WARNING: PurePursuitBaseline failed to find a path from start to goal. Returning direct waypoint.\n";
        waypoints.push_back({goal.position[0], goal.position[1]});
    }

    return waypoints;
}

std::pair<float, float> PurePursuitBaseline::_convertGridToWorld(const std::pair<int, int> grid_pos)
{
    float x_min = _robot_bridge->x_min;
    float y_min = _robot_bridge->y_min;

    float world_x = x_min + grid_pos.first * _discretization + _discretization / 2.0f;
    float world_y = y_min + grid_pos.second * _discretization + _discretization / 2.0f;

    return {world_x, world_y};
}

void PurePursuitBaseline::_readScene(float robot_radius, float discretization)
{
    float x_min = _robot_bridge->x_min;
    float x_max = _robot_bridge->x_max;
    float y_min = _robot_bridge->y_min;
    float y_max = _robot_bridge->y_max;

    int grid_width = static_cast<int>(std::ceil((x_max - x_min) / discretization));
    int grid_height = static_cast<int>(std::ceil((y_max - y_min) / discretization));

    _grid.assign(grid_height, std::vector<int>(grid_width, 0));

    const auto &obstacles = _robot_bridge->getObstacles();

    for (const auto &obs : obstacles)
    {
        float effective_radius = obs.size[0] + robot_radius;

        // Check all cells for collision
        for (int j = 0; j < grid_height; ++j)
        {
            for (int i = 0; i < grid_width; ++i)
            {
                float cell_x = x_min + i * discretization + discretization / 2.0f;
                float cell_y = y_min + j * discretization + discretization / 2.0f;

                float dx = cell_x - obs.position[0];
                float dy = cell_y - obs.position[1];
                float dist_to_center = std::sqrt(dx * dx + dy * dy);

                // check if any corner or the cell center is within the expanded obstacle
                // if the closest point of the cell to the obstacle center is within effective_radius, mark as obstacle
                float cell_half_diag = (discretization / 2.0f) * std::sqrt(2.0f);

                if (dist_to_center <= effective_radius + cell_half_diag)
                {
                    // additional precise check: minimum distance from cell to obstacle center
                    float cell_x_min = cell_x - discretization / 2.0f;
                    float cell_x_max = cell_x + discretization / 2.0f;
                    float cell_y_min = cell_y - discretization / 2.0f;
                    float cell_y_max = cell_y + discretization / 2.0f;

                    float closest_x = std::max(cell_x_min, std::min(obs.position[0], cell_x_max));
                    float closest_y = std::max(cell_y_min, std::min(obs.position[1], cell_y_max));

                    float closest_dx = closest_x - obs.position[0];
                    float closest_dy = closest_y - obs.position[1];
                    float closest_dist = std::sqrt(closest_dx * closest_dx + closest_dy * closest_dy);

                    if (closest_dist <= effective_radius)
                    {
                        _grid[j][i] = 1;
                    }
                }
            }
        }
    }
}

#define X_MIN -7.0f
#define X_MAX 7.0f
#define Y_MIN -7.0f
#define Y_MAX 7.0f
#define SCENE_FILE "../config/scene/umaze_scene.xml"

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    torch::Device device(torch::kCPU);
    if (torch::cuda::is_available())
    {
        std::cout << "CUDA available — training on GPU." << std::endl;
        device = torch::Device(torch::kCUDA);
    }
    else if (torch::mps::is_available())
    {
        std::cout << "MPS is available! Training on Apple GPU." << std::endl;
        device = torch::Device(torch::kMPS);
    }

    auto robot_bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, /*render=*/true);

    PurePursuitBaseline baseline(robot_bridge, 0.1f, 0.2f);

    AbstractedState global_goal = {{-4.5, 4.1, 0.}, {0, 0, 0, -1}, {0, 0, 0}, {0, 0, 0}};
    AbstractedState global_start = {{-5.3, -4.5, 0.}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    float reward = baseline.execute(global_start, global_goal);
    std::cout << "Total reward: " << reward << std::endl;
}