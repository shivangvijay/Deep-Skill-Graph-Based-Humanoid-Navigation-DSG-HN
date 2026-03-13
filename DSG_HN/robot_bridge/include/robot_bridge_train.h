#include "robot_bridge.h"
#include "mujoco_articulation.h"
#include "mujoco_engine.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

class RobotBridgeTrain : public RobotBridge
{
public:
    // Pass in your existing env and engine pointers. TODO: unify this and other RobotBridge by just passing in vm
    RobotBridgeTrain(std::string scene_file, float x_min, float x_max, float y_min, float y_max, std::shared_ptr<MuJoCoEngine> eng, std::unique_ptr<isaaclab::ManagerBasedRLEnv> env);

private:
    std::shared_ptr<MuJoCoEngine> eng;
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::vector<float> current_cmd;
};