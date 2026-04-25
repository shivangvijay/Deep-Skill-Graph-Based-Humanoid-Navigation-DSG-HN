#pragma once
#include "model.hpp"
#include "environment.h"
#include <torch/torch.h>
#include <unordered_map>

using namespace torch;
using Experience = std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor>;

class OUNoise
{
public:
    OUNoise(int size, float mu = 0.0f, float theta = 0.15f, float sigma = 0.3f);
    void reset();
    void setSigma(float s);

    torch::Tensor sample();

private:
    int size;
    float mu, theta, sigma;
    torch::Tensor state;
};

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
        int actor_warmup_steps,
        bool use_human_buffer = false,
        float human_data_percentage=0.25);

    std::pair<torch::Tensor, torch::Tensor> getAction(torch::Tensor state, bool eval = false);
    void addExperience(torch::Tensor state, torch::Tensor action, torch::Tensor reward, torch::Tensor next_state, torch::Tensor done);

    void loadHumanData(std::shared_ptr<TrainEnvironment> env, const std::string &filepath);
    void pretrainFromHumanData(int iterations);
    void learn();
    void hardCopy();
    void toDevice(torch::Device d);
    void resetNoise();

    double total_actor_loss = 0.0;
    double total_critic_loss = 0.0;
    int learn_step = 0;

    void setExplorationNoise(float noise);
    void setLearningRates(float lr_actor, float lr_critic);

    Actor actor_local;
    Critic critic_local_1;
    Critic critic_local_2;
    int actor_update_freq;

private:

    std::unique_ptr<OUNoise> ou_noise;
    float exploration_noise = 0.3f;
    float human_data_percentage;

    Actor actor_target;
    Critic critic_target_1;
    Critic critic_target_2;
    bool use_human_buffer;

    ReplayBuffer replay_buffer;
    ReplayBuffer human_replay_buffer; // replay buffer for human collected data

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
    std::unordered_map<int, ReplayBuffer> replay_buffers;
    int option_count = 0;
    torch::Device device;
    float lr;
    float tau;
    int batch_size;
    float gamma;
};
