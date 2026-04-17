#include "agent.h"
#include "environment.h"
#include "robot_bridge_train.h"
#include <cmath>
#include <queue>
#include "param.h"
#include <torch/torch.h>
#include <memory>

class Baseline
{
public:
    Baseline() {}
    virtual float execute(const AbstractedState &start, const AbstractedState &goal) = 0;
};

// 2d navigation of discretized space, and then follow with pure pursuit
class PurePursuitBaseline : public Baseline
{
public:
    PurePursuitBaseline(std::shared_ptr<RobotBridgeTrain> robot_bridge, float discretization = 0.1f, float robot_radius = 0.3f, float lookahead_distance = 0.5f);

    float execute(const AbstractedState &start, const AbstractedState &goal) override;

private:
    std::shared_ptr<RobotBridgeTrain> _robot_bridge;
    std::shared_ptr<TrainEnvironment> _env;
    std::vector<std::vector<int>> _grid; // 0 for free, 1 for obstacle
    float _discretization;
    float _robot_radius;
    float _lookahead_distance;

    void _readScene(float robot_radius, float discretization);

    float _h(const std::pair<float, float> pos, const std::pair<float, float> goal);

    std::pair<float, float> _convertGridToWorld(std::pair<int, int> grid_pos);

    // A* search to find path from start to goal, converting grid coord waypoint to real world waypoints
    std::vector<std::pair<float, float>> _generateWaypoint(const AbstractedState &start, const AbstractedState &goal);
};

class TD3Baseline : public Baseline
{
public:
    TD3Baseline(std::shared_ptr<RobotBridgeTrain> robot_bridge,
                const std::string &actor_path, const std::string &critic_1_path, const std::string &critic_2_path, const std::vector<int>& actor_layer_sizes, const std::vector<int>& critic_layer_sizes, torch::Device device);
    float execute(const AbstractedState &start, const AbstractedState &goal) override;

private:
    std::shared_ptr<RobotBridgeTrain> _robot_bridge;
    std::shared_ptr<TrainEnvironment> _env;
    TD3Agent _agent;
};