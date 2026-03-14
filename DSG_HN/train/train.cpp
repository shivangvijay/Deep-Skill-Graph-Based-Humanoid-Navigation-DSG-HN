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

#define SCENE_FILE "ai_maker_space_scene.xml"
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
#define BATCH_SIZE 16
#define ACTOR_UPDATE_FREQ 4
#define CRITIC_LAYER_SIZES {64, 128, 64}
#define ACTOR_LAYER_SIZES {64, 128, 64}

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    bool render = false;
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);
    auto eng = std::make_shared<MuJoCoEngine>(render);
    eng->initialize(SCENE_FILE);
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::MuJoCoArticulation>(eng));
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    std::shared_ptr<RobotBridgeTrain> robot_bridge = std::make_shared<RobotBridgeTrain>(SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, eng, std::move(env), render);
    std::shared_ptr<TrainEnvironment> train_env = std::make_shared<TrainEnvironment>(robot_bridge, 2000);

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
    int num_epochs = 20;

    std::cout << "Starting training for " << num_epochs << " epochs, " << num_steps << " steps per epoch." << std::endl;
    auto state = train_env->reset();

    // while (true)
    // {
    //     auto [state, reward, done] = train_env->step(torch::zeros({3}));
    // }

    // for (int i = 0; i < 100; i++)
    // {
    //     std::cout << "\rStep: " << i + 1 << "/100" << std::flush;
    //     auto [next_state, reward, done] = agent.act(train_env, state);
    //     if (done.data_ptr<float>()[0] > 0.5)
    //         state = train_env->reset(); // Just testing the environment step function with zero actions for now
    //     else
    //         state = next_state;
    // }

    float best_reward = -std::numeric_limits<float>::infinity();

    for (int epoch = 0; epoch < num_epochs; epoch++)
    {
        torch::Tensor state = train_env->reset();
        std::cout << "Epoch " << epoch + 1 << "/" << num_epochs << " " << std::endl;

        for (int step = 0; step < num_steps; step++)
        {
            std::cout << "\rStep: " << step + 1 << "/" << num_steps << std::flush;
            auto [next_state, reward, done] = agent.act(train_env, state);
            agent.learn(train_env);
            if (done.data_ptr<float>()[0] > 0.5)
            {
                state = train_env->reset();
            }
            else
            {
                state = next_state;
            }

            // robot_bridge->printState(robot_bridge->getRobotState());
        }
        std::cout << std::endl;
        std::cout << "Average Reward: " << agent.total_reward / (num_steps) << std::endl;
        std::cout << "Average Actor Loss: " << agent.total_actor_loss / (num_steps / ACTOR_UPDATE_FREQ) << std::endl;
        std::cout << "Average Critic Loss: " << agent.total_critic_loss / (num_steps) << std::endl;

        if (agent.total_reward > best_reward)
        {
            best_reward = agent.total_reward;
            agent.hard_copy();
            std::cout << "New best reward! Saving model." << std::endl;
            torch::save(agent.actor_local, "best_actor.pt");
            torch::save(agent.critic_local_1, "best_critic_1.pt");
            torch::save(agent.critic_local_2, "best_critic_2.pt");
        }

        agent.total_reward = 0.0;
        agent.total_actor_loss = 0.0;
        agent.total_critic_loss = 0.0;
    }
}