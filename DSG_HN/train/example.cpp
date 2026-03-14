#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "mujoco_utils/mujoco_articulation.h"
#include "mujoco_utils/mujoco_engine.h"
#include <string>
#include "param.h"
#include <algorithm>
#include "robot_bridge_train.h"
#include "environment.h"
#include <torch/torch.h>

#define SCENE_FILE "ai_maker_space_scene.xml"
#define POLICY_DIR "config/policy/velocity"
#define CONFIG_PATH "config/config.yaml"

#define X_MIN -2.0f
#define X_MAX 2.0f
#define Y_MIN -2.0f
#define Y_MAX 2.0f

std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;

/*
TODO: TONIGHT OR TOMORROW MORNING: WRITE KINDA GYM WRAPPER AROUND THIS.
THEN, FIND EXISTING TD3 IMPLEMENTATION https://github.com/hrshl212/TD3-libtorch/blob/main/agent.cpp LIKE THIS, AND MAKE LOOP IN LIBTORCH
*/

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    bool render = false;
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);
    auto eng = std::make_shared<MuJoCoEngine>(render);
    eng->initialize(SCENE_FILE);

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::MuJoCoArticulation>(eng));
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    std::shared_ptr<RobotBridgeTrain> robot_bridge = std::make_shared<RobotBridgeTrain>(SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, eng, std::move(env), render);

    TrainEnvironment train_env(robot_bridge);

    torch::Tensor action = torch::zeros({3});
    train_env.reset();
    int num_frames = 0;
    // get start time

    auto start_time = std::chrono::high_resolution_clock::now();

    while (true)
    {
        auto [state, reward, terminated] = train_env.step(action);
        if (terminated)
        {
            train_env.reset();
        }
        num_frames++;
        auto current_time = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() >= 10)
        {
            std::cout << "FPS: " << num_frames / 10.0 << std::endl;
            num_frames = 0;
            start_time = current_time;
        }
    }
}