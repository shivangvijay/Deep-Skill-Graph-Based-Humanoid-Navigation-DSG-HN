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
#include <random>
#include <fstream>

#define SCENE_FILE "../config/scene/umaze_scene.xml"
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
#define PRETRAIN false
#define ACTOR_WARMUP_STEPS 5000

static std::mt19937 her_rng(std::random_device{}());

int findSafeGoalIndex(const std::vector<Transition> &trajectory)
{
    constexpr int skip_last = 3;
    int end_idx = (int)trajectory.size() - 1;

    if (trajectory.back().in_collision)
    {
        for (int i = end_idx; i >= 0; i--)
        {
            if (!trajectory[i].in_collision)
            {
                end_idx = std::max(0, i - skip_last);
                break;
            }
        }
    }
    return end_idx;
}

void train_her(const std::vector<Transition> &trajectory, std::shared_ptr<TrainEnvironment> env, TD3Agent &agent)
{
    if (trajectory.size() < 2)
        return;

    int safe_end = findSafeGoalIndex(trajectory);
    if (safe_end < 1)
        return;

    // "Final" strategy: use the safe end state as the goal for all transitions
    AbstractedState final_goal;
    final_goal.position = trajectory[safe_end].next_state.position;
    final_goal.orientation = trajectory[safe_end].next_state.orientation;
    final_goal.velocity = {0, 0, 0};
    final_goal.angular_velocity = {0, 0, 0};

    for (int i = 0; i <= safe_end; i++)
    {
        const auto &t = trajectory[i];
        auto aug_state = env->transformState(t.state, final_goal);
        auto aug_next = env->transformState(t.next_state, final_goal);
        auto [aug_reward, aug_done] = env->computeReward(t.state, t.in_collision, final_goal);
        agent.addExperience(aug_state, t.action, aug_reward, aug_next, aug_done);
        agent.learn();
    }

    // "Future" strategy: for each transition, pick a random future state as goal
    for (int i = 0; i < safe_end; i++)
    {
        std::uniform_int_distribution<int> dist(i + 1, safe_end);
        int future_idx = dist(her_rng);

        AbstractedState future_goal;
        future_goal.position = trajectory[future_idx].next_state.position;
        future_goal.orientation = trajectory[future_idx].next_state.orientation;
        future_goal.velocity = {0, 0, 0};
        future_goal.angular_velocity = {0, 0, 0};

        const auto &t = trajectory[i];
        auto aug_state = env->transformState(t.state, future_goal);
        auto aug_next = env->transformState(t.next_state, future_goal);
        auto [aug_reward, aug_done] = env->computeReward(t.state, t.in_collision, future_goal);
        agent.addExperience(aug_state, t.action, aug_reward, aug_next, aug_done);
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
    std::shared_ptr<TrainEnvironment> train_env = std::make_shared<TrainEnvironment>(robot_bridge, 400);

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

    TD3Agent agent(train_env, actor_layer_sizes, critic_layer_sizes, device, ACTOR_LR, CRITIC_LR, TAU, GAMMA, BATCH_SIZE, ACTOR_UPDATE_FREQ, ACTOR_WARMUP_STEPS);

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
                agent.toDevice(device);
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
    if (render) train_env->updateGoalMarker();
    auto underlying_state = train_env->getUnderlyingState().first;

    std::vector<Transition> her_transitions;
    float best_reward = -std::numeric_limits<float>::infinity();

    std::filesystem::create_directories("../models/logs");
    std::ofstream log_file("../models/logs/training_log.csv");
    log_file << "epoch,avg_reward,actor_loss,critic_loss,success_rate,max_goal_dist,noise" << std::endl;

    for (int epoch = 0; epoch < num_epochs; epoch++)
    {
        float total_reward = 0.0f;

        torch::Tensor state = train_env->reset();
        if (render) train_env->updateGoalMarker();
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
                if (render) train_env->updateGoalMarker();
                underlying_state = train_env->getUnderlyingState().first;
            }
            else
            {
                state = next_state;
                underlying_state = next_underlying_state;
            }

        }
        std::cout << std::endl;
        float success_rate = (num_episodes > 0) ? (float)num_success / num_episodes * 100.0f : 0.0f;
        std::cout << "Average Reward: " << total_reward / (num_steps) << std::endl;
        std::cout << "Average Actor Loss: " << agent.total_actor_loss / (num_steps / ACTOR_UPDATE_FREQ) << std::endl;
        std::cout << "Average Critic Loss: " << agent.total_critic_loss / (num_steps) << std::endl;
        std::cout << "Success Rate: " << success_rate << "%" << std::endl;
        std::cout << "Max Goal Distance: " << train_env->getMaxGoalDistance() << "m" << std::endl;

        if (success_rate > 65.0f)
            train_env->increaseGoalDistance(2.0f);
        else if (success_rate < 10.0f)
            train_env->decreaseGoalDistance(1.0f);

        // Decay exploration noise: 0.3 → 0.05 linearly over all epochs
        float noise = 0.3f - (0.3f - 0.05f) * ((float)epoch / (float)(num_epochs - 1));
        agent.setExplorationNoise(noise);
        std::cout << "Exploration Noise: " << noise << std::endl;

        float avg_reward = total_reward / (num_steps);
        float avg_actor_loss = agent.total_actor_loss / std::max(1, num_steps / ACTOR_UPDATE_FREQ);
        float avg_critic_loss = agent.total_critic_loss / std::max(1, num_steps);

        // Log to CSV
        log_file << epoch + 1 << "," << avg_reward << "," << avg_actor_loss << ","
                 << avg_critic_loss << "," << success_rate << ","
                 << train_env->getMaxGoalDistance() << "," << noise << std::endl;

        // Save checkpoint every epoch
        std::string epoch_dir = "../models/epoch" + std::to_string(epoch + 1);
        std::filesystem::create_directories(epoch_dir);
        torch::save(agent.actor_local, epoch_dir + "/actor.pt");
        torch::save(agent.critic_local_1, epoch_dir + "/critic_1.pt");
        torch::save(agent.critic_local_2, epoch_dir + "/critic_2.pt");

        // Also save as best if it beats previous best
        if (avg_reward > best_reward)
        {
            best_reward = avg_reward;
            agent.hardCopy();
            std::cout << "New best reward! Saving best model." << std::endl;
            torch::save(agent.actor_local, "../models/best_actor.pt");
            torch::save(agent.critic_local_1, "../models/best_critic_1.pt");
            torch::save(agent.critic_local_2, "../models/best_critic_2.pt");
        }
        torch::save(agent.actor_local, "../models/last_actor.pt");
        torch::save(agent.critic_local_1, "../models/last_critic_1.pt");
        torch::save(agent.critic_local_2, "../models/last_critic_2.pt");

        agent.total_actor_loss = 0.0;
        agent.total_critic_loss = 0.0;
    }
}
