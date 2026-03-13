#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "mujoco_utils/mujoco_articulation.h"
#include "mujoco_utils/mujoco_engine.h"
#include <string>
#include "param.h"
#include <algorithm>
#include "robot_bridge_train.h"

#define SCENE_FILE "ai_maker_space_scene.xml"
#define POLICY_DIR "config/policy/velocity"
#define CONFIG_PATH "config/config.yaml"

#define X_MIN -2.0f
#define X_MAX 2.0f
#define Y_MIN -2.0f
#define Y_MAX 2.0f

std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;


int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    bool render = true;
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);
    auto eng = std::make_shared<MuJoCoEngine>(render);
    eng->initialize(SCENE_FILE);

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::MuJoCoArticulation>(eng));
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    RobotBridgeTrain robot_bridge(SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, eng, std::move(env), true);

    std::vector<float> cmd = {0.0, 1.0, 0.0};
    int step = 0;

    while (true)
    {
        if (step % 100 == 0) 
        {
            robot_bridge.resetRobot();
            robot_bridge.update();
        }
        // if (step % 10)
        // {
        //     RobotState s = robot_bridge.getRobotState();
        //     robot_bridge.printState(s);
        // }
        // if (robot_bridge.inCollision())
        // {
        //     std::cout << "In Collision" << std::endl;
        // }
        robot_bridge.publishVelCommand(cmd);
        robot_bridge.update();
        step++;
    }
}