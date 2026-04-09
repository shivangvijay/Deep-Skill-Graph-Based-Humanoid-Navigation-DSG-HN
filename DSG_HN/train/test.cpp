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
#include "agent.h"
#include <torch/torch.h>

#define SCENE_FILE "../config/scene/umaze_scene.xml"
#define POLICY_DIR "config/policy/velocity"
#define CONFIG_PATH "config/config.yaml"

#define X_MIN -7.0f
#define X_MAX 7.0f
#define Y_MIN -7.0f
#define Y_MAX 7.0f

#define CRITIC_LR 1e-3
#define ACTOR_LR 1e-3
#define TAU 0.005
#define GAMMA 0.99
#define BATCH_SIZE 16
#define ACTOR_UPDATE_FREQ 2
#define CRITIC_LAYER_SIZES {256, 256, 256}
#define ACTOR_LAYER_SIZES {256, 256, 256}
#define RENDER true


int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    bool render = RENDER;
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    std::shared_ptr<RobotBridgeTrain> robot_bridge = std::make_shared<RobotBridgeTrain>(SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, render);
    std::shared_ptr<TrainEnvironment> train_env = std::make_shared<TrainEnvironment>(robot_bridge, 600);
    train_env->max_goal_distance = 9.0f;

    torch::Device device(torch::kCPU);
    if (torch::cuda::is_available())
    {
        std::cout << "CUDA is available! Testing on GPU." << std::endl;
        device = torch::Device(torch::kCUDA);
    }
    else if (torch::mps::is_available())
    {
        std::cout << "MPS is available! Testing on Apple GPU." << std::endl;
        device = torch::Device(torch::kMPS);
    }

    std::vector<int> critic_layer_sizes = CRITIC_LAYER_SIZES;
    std::vector<int> actor_layer_sizes = ACTOR_LAYER_SIZES;

    TD3Agent agent(train_env, actor_layer_sizes, critic_layer_sizes, device, ACTOR_LR, CRITIC_LR, TAU, GAMMA, BATCH_SIZE, ACTOR_UPDATE_FREQ, 0);

    torch::load(agent.actor_local, "../models/best_actor.pt");
    torch::load(agent.critic_local_1, "../models/best_critic_1.pt");
    torch::load(agent.critic_local_2, "../models/best_critic_2.pt");
    agent.toDevice(device);

    torch::Tensor state = train_env->reset();
    train_env->updateGoalMarker();
    train_env->showObstacleMargins();
    int num_success = 0;
    int num_episodes = 0;
    while (true)
    {
        auto [action, _] = agent.getAction(state, true);
        auto [next_state, reward, done] = train_env->step(action);
        if (done.data_ptr<float>()[0] > 0.5)
        {
            num_episodes++;
            if (reward.data_ptr<float>()[0] > 40.0)
            {
                num_success++;
                std::cout << "SUCCESS | ";
            }
            else
            {
                std::cout << "FAIL    | ";
            }
            std::cout << "Success Rate: " << (float)num_success / num_episodes * 100.0f << "% (" << num_success << "/" << num_episodes << ")" << std::endl;
            state = train_env->reset();
            train_env->updateGoalMarker();
        }
        else
            state = next_state;
    }
}