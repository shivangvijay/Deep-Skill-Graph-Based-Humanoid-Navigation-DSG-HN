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

#define SCENE_FILE "../config/scene/test_scene.xml"
#define POLICY_DIR "config/policy/velocity"
#define CONFIG_PATH "config/config.yaml"

#define X_MIN -5.0f
#define X_MAX 5.0f
#define Y_MIN -5.0f
#define Y_MAX 5.0f

#define CRITIC_LR 3e-4
#define ACTOR_LR 1e-4
#define TAU 0.005
#define GAMMA 0.99
#define BATCH_SIZE 256
#define ACTOR_UPDATE_FREQ 4
#define CRITIC_LAYER_SIZES {128, 256, 128}
#define ACTOR_LAYER_SIZES {128, 256, 128}
#define RENDER false

/*
TODO: VERIFY THAT TD3 CAN STIL LEARN
*/

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    bool render = RENDER;
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    std::shared_ptr<RobotBridgeTrain> robot_bridge = std::make_shared<RobotBridgeTrain>(SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, render); //eng, std::move(env), render);
    std::shared_ptr<TrainEnvironment> train_env = std::make_shared<TrainEnvironment>(robot_bridge, 1000);

    torch::Device device(torch::kCPU);
    if (torch::cuda::is_available())
    {
        std::cout << "CUDA is available! Training on GPU." << std::endl;
        device = torch::Device(torch::kCUDA);
    }

    std::vector<int> critic_layer_sizes = CRITIC_LAYER_SIZES;
    std::vector<int> actor_layer_sizes = ACTOR_LAYER_SIZES;

    TD3Agent agent(train_env, actor_layer_sizes, critic_layer_sizes, device, ACTOR_LR, CRITIC_LR, TAU, GAMMA, BATCH_SIZE, ACTOR_UPDATE_FREQ);

    int num_frames = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    int num_steps = 20000;
    int num_epochs = 40;

    // for testing, gonna have a new random option that I am gonna add after 1000 steps
    PolicyOverOptionsAgent option_agent(train_env, critic_layer_sizes, device, CRITIC_LR, TAU, GAMMA, BATCH_SIZE);

    std::cout << "Starting training for " << num_epochs << " epochs, " << num_steps << " steps per epoch." << std::endl;
    auto state = train_env->reset();

    float best_reward = -std::numeric_limits<float>::infinity();

    for (int epoch = 0; epoch < num_epochs; epoch++)
    {
        float total_reward = 0.0f;

        torch::Tensor state = train_env->reset();
        std::cout << "Epoch " << epoch + 1 << "/" << num_epochs << " " << std::endl;
        int num_success = 0;
        int num_episodes = 0;

        for (int step = 0; step < num_steps; step++)
        {
            if (step == 10000){
                option_agent.addOption(-1.0);
            }
            std::cout << "\rStep: " << step + 1 << "/" << num_steps << std::flush;
            // auto option = option_agent.getOption(state);
            torch::Tensor action;
            torch::Tensor scaled_action;
            // if (option == 0)
            // {
            auto actions = agent.getAction(state);
            scaled_action = std::get<0>(actions);
            action = std::get<1>(actions);
            // }
            // else 
            // {
            //     action = torch::randn(3);
            //     scaled_action = action * torch::tensor(train_env->action_limits);
            // }

            // std::cout << " Option: " << option << std::endl;

            auto [next_state, reward, done] = train_env->step(scaled_action);
            total_reward += reward.item<float>();
            agent.addExperience(state, action, reward, next_state, done);
            agent.learn();
            // option_agent.addExperience(state, option, reward, next_state, done, 1);
            // option_agent.learn();
            if (done.data_ptr<float>()[0] > 0.5)
            {
                num_episodes++;
                if (reward.data_ptr<float>()[0] > 0.0)
                    num_success++;
                state = train_env->reset();
            }
            else
            {
                state = next_state;
            }

            // robot_bridge->printState(robot_bridge->getRobotState());
        }
        std::cout << std::endl;
        std::cout << "Average Reward: " << total_reward / (num_steps) << std::endl;
        std::cout << "Average Actor Loss: " << agent.total_actor_loss / (num_steps / ACTOR_UPDATE_FREQ) << std::endl;
        std::cout << "Average Critic Loss: " << agent.total_critic_loss / (num_steps) << std::endl;
        std::cout << "Success Rate: " << (float)num_success / num_episodes * 100.0f << "%" << std::endl;

        if (total_reward / (num_steps) > best_reward)
        {
            best_reward = total_reward / (num_steps);
            agent.hardCopy();
            std::cout << "New best reward! Saving model." << std::endl;
            torch::save(agent.actor_local, "best_actor.pt");
            torch::save(agent.critic_local_1, "best_critic_1.pt");
            torch::save(agent.critic_local_2, "best_critic_2.pt");
        }

        agent.total_actor_loss = 0.0;
        agent.total_critic_loss = 0.0;
    }
}