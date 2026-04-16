#include "agent.h"
#include "environment.h"
#include "robot_bridge_train.h"
#include <cmath>
#include <queue>
#include "param.h"
#include <torch/torch.h>


// 2d navigation of discretized space, and then follow with pure pursuit
class PurePursuitBaseline
{
public:
    PurePursuitBaseline(std::shared_ptr<RobotBridgeTrain> robot_bridge, float discretization=0.1f, float robot_radius=0.3f, float lookahead_distance=0.5f);

    float execute(const AbstractedState& start, const AbstractedState& goal);
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
    std::vector<std::pair<float, float>> _generateWaypoint(const AbstractedState& start, const AbstractedState& goal);
};