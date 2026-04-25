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

// Scene file setup
#define SCENE_FILE "../config/scene/test_scene.xml"
#define POLICY_DIR "config/policy/velocity"
#define CONFIG_PATH "config/config.yaml"

#define X_MIN -7.0f
#define X_MAX 7.0f
#define Y_MIN -7.0f
#define Y_MAX 7.0f

// Hyperparameters
#define CRITIC_LR 3e-4
#define ACTOR_LR 1e-4
#define TAU 0.005
#define GAMMA 0.99
#define BATCH_SIZE 256
#define UPDATES_PER_STEP 2
#define ACTOR_UPDATE_FREQ 2
#define CRITIC_LAYER_SIZES {256, 256, 256}
#define ACTOR_LAYER_SIZES {256, 256, 256}
#define RENDER false
#define PRETRAIN true
#define PRETRAIN_ACTOR_PATH "../models/pretrain_actor_test_scene.pt"
#define PRETRAIN_CRITIC_1_PATH "../models/pretrain_critic_1_test_scene.pt"
#define PRETRAIN_CRITIC_2_PATH "../models/pretrain_critic_2_test_scene.pt"
#define USE_HUMAN_COLLECTED_BUFFER false
#define HUMAN_COLLECTED_BUFFER_PATH "../../sandbox/transitions_narrow_point_to_point.csv"
#define ACTOR_WARMUP_STEPS 5000
#define USE_HER false
#define USE_CURRICULUM true
#define START_TRAIN_DISTANCE 9.0
#define START_NOISE 0.3
#define MAX_ENV_STEPS 600
#define NARROW_MAP false // use this if you want to use the reward and state tuned for the narrow shimmy map type

#define TEST false

// TODO next: retrain model for UMaze, and then run DSC on both
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

    // 1. Identify a safe goal index (avoids relabeling collisions as successes)
    int safe_end = findSafeGoalIndex(trajectory);
    if (safe_end < 1)
        return;

    // Lambda for relabeling a SINGLE transition to keep the logic clean
    auto relabel_and_add = [&](int idx, const AbstractedState &hindsight_goal)
    {
        const auto &t = trajectory[idx];

        // Relabel observations relative to the NEW goal
        auto aug_state = env->transformState(t.state, hindsight_goal);
        auto aug_next = env->transformState(t.next_state, hindsight_goal);

        // Manual Sparse Reward Calculation
        float dx = t.next_state.position[0] - hindsight_goal.position[0];
        float dy = t.next_state.position[1] - hindsight_goal.position[1];
        float dist = std::sqrt(dx * dx + dy * dy);

        float aug_reward;
        bool aug_done = false;

        // Reward Scaling: Reduced by 10x to ground the Critic and prevent divergence
        if (t.in_collision)
        {
            aug_reward = -50.0f;
            aug_done = true;
        }
        else if (dist < env->getSuccessRadius())
        {
            aug_reward = 50.0f;
            aug_done = true;
        }
        else
        {
            // Constant small penalty to encourage efficiency without distance noise
            aug_reward = -0.1f;
        }

        agent.addExperience(aug_state, t.action,
                            torch::tensor({aug_reward}),
                            aug_next,
                            torch::tensor({(float)aug_done}));
    };

    // --- STRATEGY: FINAL ---
    // Relabel the entire path as if the robot's final safe position was the goal.
    AbstractedState final_goal;
    final_goal.position = trajectory[safe_end].next_state.position;
    final_goal.orientation = trajectory[safe_end].next_state.orientation;
    final_goal.velocity = {0, 0, 0};
    final_goal.angular_velocity = {0, 0, 0};
    for (int i = 0; i <= safe_end; i++)
    {
        relabel_and_add(i, final_goal);
    }

    // --- STRATEGY: FUTURE ---
    // For each real step, sample k random future states as goals.
    // Standard k-ratio is typically 4 or 8.
    const int k_ratio = 4;
    for (int i = 0; i < safe_end; i++)
    {
        for (int k = 0; k < k_ratio; k++)
        {
            std::uniform_int_distribution<int> dist(i + 1, safe_end);
            int future_idx = dist(her_rng);

            AbstractedState future_goal;
            future_goal.position = trajectory[future_idx].next_state.position;
            future_goal.orientation = trajectory[future_idx].next_state.orientation;
            future_goal.velocity = {0, 0, 0};
            future_goal.angular_velocity = {0, 0, 0};

            relabel_and_add(i, future_goal);
        }
    }
}

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    bool render = RENDER;
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    std::shared_ptr<RobotBridgeTrain> robot_bridge = std::make_shared<RobotBridgeTrain>(SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, render); // eng, std::move(env), render);
    std::shared_ptr<TrainEnvironment> train_env = std::make_shared<TrainEnvironment>(robot_bridge, MAX_ENV_STEPS, NARROW_MAP);

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

    TD3Agent agent(train_env, actor_layer_sizes, critic_layer_sizes, device, ACTOR_LR, CRITIC_LR, TAU, GAMMA, BATCH_SIZE, ACTOR_UPDATE_FREQ, ACTOR_WARMUP_STEPS, USE_HUMAN_COLLECTED_BUFFER);

    if (PRETRAIN || TEST)
    {
        auto actor_path = PRETRAIN_ACTOR_PATH;
        auto critic1_path = PRETRAIN_CRITIC_1_PATH;
        auto critic2_path = PRETRAIN_CRITIC_2_PATH;
        if (std::filesystem::exists(actor_path) && std::filesystem::exists(critic1_path) && std::filesystem::exists(critic2_path))
        {
            torch::load(agent.actor_local, actor_path);
            torch::load(agent.critic_local_1, critic1_path);
            torch::load(agent.critic_local_2, critic2_path);
            agent.toDevice(device);
            agent.hardCopy();
        }
        else
        {
            std::cerr << "WARNING: Pretrained model files missing in ../models. Continuing without pretrained weights.\n";
        }
    }

    if (USE_HUMAN_COLLECTED_BUFFER && !TEST)
    {
        std::string human_data_filepath = HUMAN_COLLECTED_BUFFER_PATH;
        agent.loadHumanData(train_env, human_data_filepath);
        agent.pretrainFromHumanData(7500);
    }

    int num_frames = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    int num_steps = 10000;
    int num_epochs = 100;

    std::cout << "Starting training for " << num_epochs << " epochs, " << num_steps << " steps per epoch." << std::endl;

    std::vector<Transition> her_transitions;
    float best_reward = -std::numeric_limits<float>::infinity();

    std::filesystem::create_directories("../models/logs");
    std::ofstream log_file("../models/logs/training_log.csv");
    log_file << "epoch,avg_reward,actor_loss,critic_loss,success_rate,max_goal_dist,noise" << std::endl;

    AbstractedState global_goal = {{2.0, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    AbstractedState global_start = {{-2.0, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    // train_env->setGoal(global_goal);
    train_env->setGoalDistance(START_TRAIN_DISTANCE);
    agent.setExplorationNoise(START_NOISE);
    agent.resetNoise();

    for (int epoch = 0; epoch < num_epochs; epoch++)
    {
        float total_reward = 0.0f;

        train_env->clearGoal();
        torch::Tensor state = train_env->reset(); // To(global_start);
        agent.resetNoise();
        if (render)
            train_env->updateGoalMarker();
        auto underlying_state = train_env->getUnderlyingState().first;

        std::cout << "Epoch " << epoch + 1 << "/" << num_epochs << " " << std::endl;
        int num_success = 0;
        int num_episodes = 0;

        for (int step = 0; step < num_steps; step++)
        {
            std::cout << "\rStep: " << step + 1 << "/" << num_steps << std::flush;
            auto [scaled_action, action] = agent.getAction(state, TEST);

            auto [next_state, reward, done] = train_env->step(scaled_action);
            auto [next_underlying_state, collision] = train_env->getUnderlyingState();

            total_reward += reward.item<float>();
            if (!TEST)
            {
                agent.addExperience(state, action, reward, next_state, done);
                for (int i = 0; i < UPDATES_PER_STEP; i++)
                    agent.learn();
                if (USE_HER)
                    her_transitions.push_back({underlying_state, action, next_underlying_state, collision});
            }

            if (done.item<float>() > 0.5)
            {
                num_episodes++;
                if (reward.data_ptr<float>()[0] > train_env->success_val) // reduced from 45, since new max reward is 250 / 10
                    num_success++;
                if (!TEST && USE_HER)
                {
                    train_her(her_transitions, train_env, agent);
                    her_transitions.clear();
                }
                train_env->clearGoal();
                state = train_env->reset(); // To(global_start);
                if (render)
                    train_env->updateGoalMarker();
                underlying_state = train_env->getUnderlyingState().first;
                agent.resetNoise();
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

        float success_needed = 65.0f;
        if (train_env->getMaxGoalDistance() <= 5.5)
        {
            success_needed = 80.0f;
        }
        if (success_rate > success_needed && USE_CURRICULUM)
            train_env->increaseGoalDistance(2.0f);
        else if (success_rate < 10.0f && USE_CURRICULUM)
            train_env->decreaseGoalDistance(1.0f);

        // Decay exploration noise: 0.3 → 0.05 linearly over all epochs
        float noise = START_NOISE - (START_NOISE - 0.05f) * ((float)epoch / (float)(num_epochs - 1));
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
        agent.total_actor_loss = 0.0;
        agent.total_critic_loss = 0.0;
    }
}
