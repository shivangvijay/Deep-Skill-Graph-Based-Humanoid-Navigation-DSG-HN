// Note: fils is a modified version as that found in: https://github.com/hrshl212/TD3-libtorch/blob/main/model.hpp
#pragma once
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <iostream>
#include <torch/torch.h>
#include <boost/circular_buffer.hpp>
#include <vector>
#include <random>

using namespace torch;
using Experience = std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor>; // this is how we are going to save expirience in buffer

/*learning hyper-params*/
const int64_t actor_state = 4; // input size of actor
const int64_t actor_layer_1 = 16;
const int64_t actor_layer_2 = 32;
// const int64_t actor_layer_3 = 64;
// const int64_t actor_layer_4 = 32;
const int64_t actor_out = 2; // output size of actor

// input to critic = actor_state , actor_out as critic is for action-value
const int64_t critic_layer_1 = 16;
const int64_t critic_layer_2 = 32;
const int64_t critic_layer_3 = actor_out;
const int64_t critic_out = 1; // output_size of critic
// note: architecture and activation f's of networks can be changed in "model.hpp"

const float actor_lr = 1e-3;  // learning rate for actor = 0.0001
const float critic_lr = 1e-3; // learning rate of critic= 0.001
const float tau = 0.005;      // rate at which target nets are updated from local nets
const int batch_size = 16;
const double action_scale = 1.0; // we are going to scale the output of action by this quantity
const float gamma2 = 0.99;       // forget/discout rate

inline void xavier_init_weights(torch::nn::Module &module)
{
    torch::NoGradGuard noGrad;

    if (auto *linear = module.as<torch::nn::Linear>())
    {
        torch::nn::init::xavier_normal_(linear->weight);
        torch::nn::init::constant_(linear->bias, 0.01);
    }
}

struct CriticImpl : nn::Module
{
    CriticImpl(int state_size, int action_size, const std::vector<int> &layer_sizes, torch::Device device_) : device(device_)
    {
        hidden_layers = register_module("layers", nn::Sequential());
        for (int i = 0; i < layer_sizes.size(); i++)
        {
            int input_size = (i == 0) ? (state_size + 2 * action_size) : layer_sizes[i - 1]; // 2 * action size since state includes last action
            int output_size = layer_sizes[i];
            hidden_layers->push_back(nn::Linear(input_size, output_size));
            hidden_layers->push_back(nn::ReLU());
        }
        output_layer = register_module("output_layer", nn::Linear(layer_sizes.back(), 1));

        this->to(device);
        xavier_init_weights(*this);
    }

    torch::Tensor forward(torch::Tensor state, torch::Tensor action)
    {
        state = state.to(device);
        action = action.to(device);
        if (state.dim() == 1)
            state = torch::unsqueeze(state, 0);
        if (action.dim() == 1)
            action = torch::unsqueeze(action, 0);

        auto x = torch::cat({state, action}, -1);
        x = hidden_layers->forward(x);
        return output_layer->forward(x);
    }

    nn::Sequential hidden_layers{nullptr};
    nn::Linear output_layer{nullptr};
    torch::Device device = torch::kCPU;
};

struct ActorImpl : nn::Module
{
    ActorImpl(int state_size, int action_size, const std::vector<int> &layer_sizes, torch::Device device_) : device(device_)
    {
        hidden_layers = register_module("layers", nn::Sequential());
        for (int i = 0; i < layer_sizes.size(); i++)
        {
            int input_size = (i == 0) ? state_size + action_size : layer_sizes[i - 1]; // include last action
            int output_size = layer_sizes[i];
            hidden_layers->push_back(nn::Linear(input_size, output_size));
            hidden_layers->push_back(nn::ReLU());
        }
        output_layer = register_module("output_layer", nn::Linear(layer_sizes.back(), action_size));

        this->to(device);

        xavier_init_weights(*this);
    }

    torch::Tensor forward(torch::Tensor state)
    {
        state = state.to(device);
        if (state.dim() == 1)
            state = torch::unsqueeze(state, 0);

        auto x = hidden_layers->forward(state);
        return torch::tanh(output_layer->forward(x));
        ;
    }

    nn::Sequential hidden_layers{nullptr};
    nn::Linear output_layer{nullptr};
    torch::Device device = torch::kCPU;
    torch::Tensor action_scales;
};
TORCH_MODULE(Critic);
TORCH_MODULE(Actor);

class ReplayBuffer
{
public:
    ReplayBuffer() {}

    void addExperienceState(torch::Tensor state, torch::Tensor action, torch::Tensor reward, torch::Tensor next_state, torch::Tensor done) // add the 5 info to buffer
    {
        auto s = state.detach().reshape({1, -1});
        auto a = action.detach().reshape({1, -1});
        auto r = reward.detach().reshape({1, 1});
        auto ns = next_state.detach().reshape({1, -1});
        auto d = done.detach().reshape({1, 1});
        addExperienceState(std::make_tuple(s, a, r, ns, d)); // but first convert them to a tuple
    }

    void addExperienceState(Experience experience)
    {
        circular_buffer.push_back(experience); // finally add them
    }

    Experience sample(int batch_size)
    {
        std::random_device rd;
        std::mt19937 re(rd());
        // Sample indices from 0 to current size
        std::uniform_int_distribution<size_t> dist(0, circular_buffer.size() - 1);

        std::vector<torch::Tensor> states, actions, rewards, next_states, dones;

        for (int i = 0; i < batch_size; i++)
        {
            Experience exp = circular_buffer.at(dist(re));
            states.push_back(std::get<0>(exp));
            actions.push_back(std::get<1>(exp));
            rewards.push_back(std::get<2>(exp));
            next_states.push_back(std::get<3>(exp));
            dones.push_back(std::get<4>(exp));
        }

        // Concatenate all tensors into batches
        return std::make_tuple(
            torch::cat(states, 0),
            torch::cat(actions, 0),
            torch::cat(rewards, 0),
            torch::cat(next_states, 0),
            torch::cat(dones, 0));
    }

    size_t getLength()
    {
        return circular_buffer.size(); // size of the buffer
    }

    boost::circular_buffer<Experience> circular_buffer{10000}; // max size of buffer = 10000
}; // refer to https://github.com/EmmiOcean/DDPG_LibTorch/blob/master/replayBuffer.h

class OUNoise
{
    //"""Ornstein-Uhlenbeck process."""
private:
    size_t size;
    std::vector<double> mu;
    std::vector<double> state;
    double theta = 0.15;
    double sigma = 0.1;

public:
    OUNoise(size_t size_in)
    {
        size = size_in;
        mu = std::vector<double>(size, 0);
        reset();
    }

    void reset()
    {
        state = mu;
    }

    std::vector<double> sample(std::vector<double> action)
    {
        //"""Update internal state and return it as a noise sample."""
        for (size_t i = 0; i < state.size(); i++)
        {
            auto random = ((double)rand() / (RAND_MAX));
            double dx = theta * (mu[i] - state[i]) + sigma * random;
            state[i] = state[i] + dx;
            action[i] += state[i];
        }
        return action;
    }
};