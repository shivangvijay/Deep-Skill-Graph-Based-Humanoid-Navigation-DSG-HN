#pragma once
#include "model.hpp"
#include "environment.h"
#include <torch/torch.h>

using namespace torch;
using Experience = std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor>;

class TD3Agent
{
public:
    TD3Agent(
        std::shared_ptr<TrainEnvironment> env,
        const std::vector<int> &actor_layer_sizes,
        std::vector<int> &critic_layer_sizes,
        torch::Device device, float lr_actor,
        float lr_critic,
        float tau,
        float gamma,
        int batch_size,
        int actor_update_freq);

    void hard_copy();
    void soft_update();
    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> act(std::shared_ptr<TrainEnvironment> env, torch::Tensor state, bool eval = false);
    void learn(std::shared_ptr<TrainEnvironment> env);
    double total_actor_loss = 0.0;
    double total_critic_loss = 0.0;
    double total_reward = 0.0;
    int learn_step = 0;

    Actor actor_local;
    Critic critic_local_1;
    Critic critic_local_2;

private:
    Actor actor_target;
    Critic critic_target_1;
    Critic critic_target_2;

    torch::optim::Adam actor_optimizer;
    torch::optim::Adam critic_optimizer_1;
    torch::optim::Adam critic_optimizer_2;

    ReplayBuffer replay_buffer;
    torch::Device device;
    torch::Tensor action_limits;
    torch::Tensor last_action;
    bool is_first_step = true;
    float tau;
    float gamma;
    int batch_size;
    int actor_update_freq;
    float lr_critic;
    float lr_actor;
};