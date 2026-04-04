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
#include "skill.h"
#include <torch/torch.h>
#include <filesystem>

#define SCENE_FILE "../config/scene/umaze_scene_obs_free.xml"
#define POLICY_DIR "config/policy/velocity"
#define CONFIG_PATH "config/config.yaml"

#define X_MIN -7.0f
#define X_MAX 7.0f
#define Y_MIN -7.0f
#define Y_MAX 7.0f

#define CRITIC_LR 3e-4 // lowered from 3e-3
#define ACTOR_LR 1e-4  // lowered from 1e-4 for fine tuning
#define TAU 0.005
#define GAMMA 0.99
#define BATCH_SIZE 256
#define ACTOR_UPDATE_FREQ 2
#define CRITIC_LAYER_SIZES {256, 256, 256}
#define ACTOR_LAYER_SIZES {256, 256, 256}
#define RENDER false
#define MAX_OBSTACLES 8
#define PRETRAIN false
#define ACTOR_WARMUP_STEPS 0000

void train_her(const std::vector<Transition> &trajectory, std::shared_ptr<TrainEnvironment> env, TD3Agent &agent)
{
    if (trajectory.size() == 0 || trajectory.back().in_collision == true)
        return;

    AbstractedState augmented_goal;
    augmented_goal.position = trajectory.back().state.position;
    augmented_goal.orientation = trajectory.back().state.orientation;
    augmented_goal.velocity = trajectory.back().state.velocity;
    augmented_goal.angular_velocity = trajectory.back().state.angular_velocity;

    for (const auto &t : trajectory)
    {
        auto augmented_state = env->transformState(t.state, augmented_goal);
        auto augmented_next_state = env->transformState(t.next_state, augmented_goal);

        auto [augmented_reward, augmented_done] = env->computeReward(t.state, t.in_collision, augmented_goal);
        agent.addExperience(augmented_state, t.action, augmented_reward, augmented_next_state, augmented_done);
        agent.learn();
    }
}

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    bool render = RENDER;
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    std::shared_ptr<RobotBridgeTrain> robot_bridge = std::make_shared<RobotBridgeTrain>(SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, render); // eng, std::move(env), render);
    std::shared_ptr<TrainEnvironment> train_env = std::make_shared<TrainEnvironment>(robot_bridge, 1000);

    torch::Device device(torch::kCPU);
    if (torch::cuda::is_available())
    {
        std::cout << "CUDA is available! Training on GPU." << std::endl;
        device = torch::Device(torch::kCUDA);
    }
    else if (torch::mps::is_available())
    {
        std::cout << "MPS is available! Training on Apple GPU." << std::endl;
        device = torch::Device(torch::kMPS);
    }

    std::vector<int> critic_layer_sizes = CRITIC_LAYER_SIZES;
    std::vector<int> actor_layer_sizes = ACTOR_LAYER_SIZES;

    TD3Agent agent(train_env, actor_layer_sizes, critic_layer_sizes, device, ACTOR_LR, CRITIC_LR, TAU, GAMMA, BATCH_SIZE, ACTOR_UPDATE_FREQ, MAX_OBSTACLES, ACTOR_WARMUP_STEPS);

    if (PRETRAIN)
    {
        std::filesystem::path model_dir("../models");
        if (!std::filesystem::exists(model_dir))
        {
            std::cerr << "WARNING: PRETRAIN enabled but ../models does not exist. Skipping pretrained load.\n";
        }
        else
        {
            auto actor_path = model_dir / "best_actor.pt";
            auto critic1_path = model_dir / "best_critic_1.pt";
            auto critic2_path = model_dir / "best_critic_2.pt";
            if (std::filesystem::exists(actor_path) && std::filesystem::exists(critic1_path) && std::filesystem::exists(critic2_path))
            {
                torch::load(agent.actor_local, actor_path.string());
                torch::load(agent.critic_local_1, critic1_path.string());
                torch::load(agent.critic_local_2, critic2_path.string());
                agent.hardCopy();
            }
            else
            {
                std::cerr << "WARNING: Pretrained model files missing in ../models. Continuing without pretrained weights.\n";
            }
        }
    }

    int num_frames = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    int num_steps = 20000;
    int num_epochs = 60;

    std::cout << "Starting training for " << num_epochs << " epochs, " << num_steps << " steps per epoch." << std::endl;
    auto state = train_env->reset();
    auto underlying_state = train_env->getUnderlyingState().first;

    std::vector<Transition> her_transitions;
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
            std::cout << "\rStep: " << step + 1 << "/" << num_steps << std::flush;
            auto [scaled_action, action] = agent.getAction(state);

            auto [next_state, reward, done] = train_env->step(scaled_action);
            auto [next_underlying_state, collision] = train_env->getUnderlyingState();

            total_reward += reward.item<float>();
            agent.addExperience(state, action, reward, next_state, done);
            agent.learn();

            her_transitions.push_back({underlying_state, action, next_underlying_state, collision});

            if (done.item<float>() > 0.5)
            {
                num_episodes++;
                if (reward.data_ptr<float>()[0] > 0)
                    num_success++;
                train_her(her_transitions, train_env, agent);
                her_transitions.clear();
                state = train_env->reset();
                underlying_state = train_env->getUnderlyingState().first;
            }
            else
            {
                state = next_state;
                underlying_state = next_underlying_state;
            }

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
            std::filesystem::create_directories("../models");
            torch::save(agent.actor_local, "../models/best_actor.pt");
            torch::save(agent.critic_local_1, "../models/best_critic_1.pt");
            torch::save(agent.critic_local_2, "../models/best_critic_2.pt");
        }

        agent.total_actor_loss = 0.0;
        agent.total_critic_loss = 0.0;
    }
}
