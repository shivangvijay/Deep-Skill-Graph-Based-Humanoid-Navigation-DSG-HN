#include "robot_bridge_train.h"

RobotBridgeTrain::RobotBridgeTrain(std::string scene_file, float x_min, float x_max, float y_min, float y_max, std::shared_ptr<MuJoCoEngine> eng_, std::unique_ptr<isaaclab::ManagerBasedRLEnv> env_)
    : RobotBridge(scene_file, x_min, x_max, y_min, y_max), eng(eng_), env(std::move(env_))
{
}