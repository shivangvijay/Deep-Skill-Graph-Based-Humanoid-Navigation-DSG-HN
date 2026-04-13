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
        const std::vector<int> &critic_layer_sizes,
        torch::Device device, float lr_actor,
        float lr_critic,
        float tau,
        float gamma,
        int batch_size,
        int actor_update_freq,
        int actor_warmup_steps);

    std::pair<torch::Tensor, torch::Tensor> getAction(torch::Tensor state, bool eval = false);
    void addExperience(torch::Tensor state, torch::Tensor action, torch::Tensor reward, torch::Tensor next_state, torch::Tensor done);

    void learn();
    void hardCopy();
    void toDevice(torch::Device d);

    double total_actor_loss = 0.0;
    double total_critic_loss = 0.0;
    int learn_step = 0;
    float exploration_noise = 0.3f;

    void setExplorationNoise(float noise) { exploration_noise = noise; }
    void setLearningRates(float lr_actor, float lr_critic) {
        for (auto& param_group : actor_optimizer.param_groups()) {
            static_cast<torch::optim::AdamOptions&>(param_group.options()).lr(lr_actor);
        }
        for (auto& param_group : critic_optimizer_1.param_groups()) {
            static_cast<torch::optim::AdamOptions&>(param_group.options()).lr(lr_critic);
        }
        for (auto& param_group : critic_optimizer_2.param_groups()) {
            static_cast<torch::optim::AdamOptions&>(param_group.options()).lr(lr_critic);
        }
    }

    Actor actor_local;
    Critic critic_local_1;
    Critic critic_local_2;
    ReplayBuffer replay_buffer;
    int actor_update_freq;

private:
    Actor actor_target;
    Critic critic_target_1;
    Critic critic_target_2;

    torch::optim::Adam actor_optimizer;
    torch::optim::Adam critic_optimizer_1;
    torch::optim::Adam critic_optimizer_2;

    torch::Device device;
    torch::Tensor action_scaling_factors;
    torch::Tensor action_shift_factors;

    float tau;
    float gamma;
    int batch_size;
    float lr_critic;
    float lr_actor;
    int actor_warmup_steps;

    void softUpdate();
    torch::Tensor prepareLocalState(torch::Tensor state);
};

class PolicyOverOptionsAgent
{
public:
    PolicyOverOptionsAgent(
        std::shared_ptr<TrainEnvironment> env,
        const std::vector<int> &layer_sizes,
        torch::Device device,
        float lr,
        float tau,
        float gamma,
        int batch_size);

    void addOption(float initial_bias);
    void learn();
    void hardCopy();

    torch::Tensor getOptions(torch::Tensor state);
    void addExperience(torch::Tensor state, torch::Tensor option, torch::Tensor cumulative_reward, torch::Tensor next_state, torch::Tensor done, torch::Tensor num_steps);
    void addExperience(torch::Tensor state, int option, float cumulative_reward, torch::Tensor next_state, bool done, int num_steps);

    PolicyOverOptions q;
    PolicyOverOptions target_q;

private:
    void softUpdate();

    std::unique_ptr<torch::optim::Adam> optimizer;
    ReplayBuffer replay_buffer;
    torch::Device device;
    float lr;
    float tau;
    int batch_size;
    float gamma;
};