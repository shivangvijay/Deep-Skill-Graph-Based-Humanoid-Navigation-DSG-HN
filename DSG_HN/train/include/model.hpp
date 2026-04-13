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
#include <cmath>

using namespace torch;
using Experience = std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor>; // this is how we are going to save expirience in buffer

inline void xavier_init_weights(torch::nn::Module &module);

struct ObstacleAttentionEncoderImpl : nn::Module
{
    ObstacleAttentionEncoderImpl(int base_state_size_, int obstacle_feature_size_, int attention_dim_, torch::Device device_)
        : base_state_size(base_state_size_), obstacle_feature_size(obstacle_feature_size_), attention_dim(attention_dim_), device(device_)
    {
        query_proj = register_module("query_proj", nn::Linear(base_state_size, attention_dim));
        obs_embed = register_module("obs_embed", nn::Linear(obstacle_feature_size, attention_dim));
        key_proj = register_module("key_proj", nn::Linear(attention_dim, attention_dim));
        value_proj = register_module("value_proj", nn::Linear(attention_dim, attention_dim));
        this->to(device);
        xavier_init_weights(*this);
    }

    torch::Tensor forward(torch::Tensor state)
    {
        state = state.to(device);
        if (state.dim() == 1)
            state = torch::unsqueeze(state, 0);

        auto base = state.narrow(-1, 0, base_state_size);
        int64_t obs_flat_dim = state.size(-1) - base_state_size;
        if (obs_flat_dim < obstacle_feature_size)
        {
            auto zero_ctx = torch::zeros({state.size(0), 2 * attention_dim}, state.options());
            return torch::cat({base, zero_ctx}, -1);
        }

        int64_t obs_count = obs_flat_dim / obstacle_feature_size;
        int64_t used_obs_dim = obs_count * obstacle_feature_size;
        auto obstacle_flat = state.narrow(-1, base_state_size, used_obs_dim);
        auto obstacle_tokens = obstacle_flat.view({state.size(0), obs_count, obstacle_feature_size});

        auto obs_latent = torch::relu(obs_embed->forward(obstacle_tokens));
        auto query = query_proj->forward(base).unsqueeze(1);
        auto keys = key_proj->forward(obs_latent);
        auto values = value_proj->forward(obs_latent);

        auto scores = torch::bmm(query, keys.transpose(1, 2)) / std::sqrt((double)attention_dim);
        auto weights = torch::softmax(scores, -1);
        auto attended = torch::bmm(weights, values).squeeze(1);

        auto max_pooled = std::get<0>(obs_latent.max(1));
        auto ctx = torch::cat({attended, max_pooled}, -1);
        return torch::cat({base, ctx}, -1);
    }

    int outputDim() const
    {
        return base_state_size + 2 * attention_dim;
    }

    nn::Linear query_proj{nullptr};
    nn::Linear obs_embed{nullptr};
    nn::Linear key_proj{nullptr};
    nn::Linear value_proj{nullptr};
    int base_state_size = 0;
    int obstacle_feature_size = 4;
    int attention_dim = 32;
    torch::Device device = torch::kCPU;
};

TORCH_MODULE(ObstacleAttentionEncoder);

inline void xavier_init_weights(torch::nn::Module &module)
{
    torch::NoGradGuard noGrad;

    if (auto *linear = module.as<torch::nn::LinearImpl>())
    {
        torch::nn::init::xavier_normal_(linear->weight);
        if (linear->bias.defined())
            torch::nn::init::constant_(linear->bias, 0.01);
    }

    for (const auto &child : module.children())
    {
        xavier_init_weights(*child);
    }
}

struct CriticImpl : nn::Module
{
    CriticImpl(int base_state_size, int obstacle_feature_size, int action_size, const std::vector<int> &layer_sizes, torch::Device device_, int attention_dim = 32)
        : device(device_)
    {
        encoder = register_module("encoder", ObstacleAttentionEncoder(base_state_size, obstacle_feature_size, attention_dim, device_));
        hidden_layers = register_module("layers", nn::Sequential());
        for (int i = 0; i < layer_sizes.size(); i++)
        {
            int input_size = (i == 0) ? (encoder->outputDim() + action_size) : layer_sizes[i - 1];
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
        state = encoder->forward(state);
        action = action.to(device);
        if (action.dim() == 1)
            action = torch::unsqueeze(action, 0);

        auto x = torch::cat({state, action}, -1);
        x = hidden_layers->forward(x);
        return output_layer->forward(x);
    }

    ObstacleAttentionEncoder encoder{nullptr};
    nn::Sequential hidden_layers{nullptr};
    nn::Linear output_layer{nullptr};
    torch::Device device = torch::kCPU;
};

struct ActorImpl : nn::Module
{
    ActorImpl(int base_state_size, int obstacle_feature_size, int action_size, const std::vector<int> &layer_sizes, torch::Device device_, int attention_dim = 32)
        : device(device_)
    {
        encoder = register_module("encoder", ObstacleAttentionEncoder(base_state_size, obstacle_feature_size, attention_dim, device_));
        hidden_layers = register_module("layers", nn::Sequential());
        for (int i = 0; i < layer_sizes.size(); i++)
        {
            int input_size = (i == 0) ? encoder->outputDim() : layer_sizes[i - 1];
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
        state = encoder->forward(state);

        auto x = hidden_layers->forward(state);
        return torch::tanh(output_layer->forward(x));
    }

    ObstacleAttentionEncoder encoder{nullptr};
    nn::Sequential hidden_layers{nullptr};
    nn::Linear output_layer{nullptr};
    torch::Device device = torch::kCPU;
    torch::Tensor action_scales;
};

struct PolicyOverOptionsImpl : nn::Module
{
    PolicyOverOptionsImpl(int base_state_size, int obstacle_feature_size, const std::vector<int> &layer_sizes, torch::Device device_, int attention_dim = 32)
        : device(device_)
    {
        encoder = register_module("encoder", ObstacleAttentionEncoder(base_state_size, obstacle_feature_size, attention_dim, device_));
        hidden_layers = register_module("layers", nn::Sequential());
        for (int i = 0; i < layer_sizes.size(); i++)
        {
            int input_size = (i == 0) ? encoder->outputDim() : layer_sizes[i - 1];
            int output_size = layer_sizes[i];
            hidden_layers->push_back(nn::Linear(input_size, output_size));
            hidden_layers->push_back(nn::ReLU());
        }
        output_layer = register_module("output_layer", nn::Linear(layer_sizes.back(), 1)); // start with just one option
        this->to(device);
        xavier_init_weights(*this);
    }

    void addOption(float initial_bias) // note: need to reset optimizer when calling this function
    {
        torch::NoGradGuard no_grad;

        int64_t input_size = output_layer->weight.size(1);
        int64_t old_output_size = output_layer->weight.size(0);
        int64_t new_output_size = old_output_size + 1;

        auto new_layer = nn::Linear(input_size, new_output_size);
        new_layer->to(device);

        // narrow (dim, start, length) is used to select a sub-tensor, and then we copy the old weights into that sub-tensor
        new_layer->weight.narrow(0, 0, old_output_size).copy_(output_layer->weight);
        new_layer->bias.narrow(0, 0, old_output_size).copy_(output_layer->bias);

        new_layer->weight.narrow(0, old_output_size, 1).fill_(0);
        new_layer->bias[old_output_size].fill_(initial_bias);
        this->unregister_module("output_layer");
        output_layer = register_module("output_layer", new_layer);
    }

    torch::Tensor forward(torch::Tensor state)
    {
        state = encoder->forward(state);

        auto x = hidden_layers->forward(state);
        return output_layer->forward(x);
    }

    ObstacleAttentionEncoder encoder{nullptr};
    nn::Sequential hidden_layers{nullptr};
    nn::Linear output_layer{nullptr};
    torch::Device device = torch::kCPU;
};

TORCH_MODULE(Critic);
TORCH_MODULE(Actor);
TORCH_MODULE(PolicyOverOptions);

class ReplayBuffer
{
public:
    ReplayBuffer() {}

    void addExperienceState(torch::Tensor state, torch::Tensor action, torch::Tensor reward, torch::Tensor next_state, torch::Tensor done) // add the 5 info to buffer
    {
        if (state.dim() == 1) state = torch::unsqueeze(state, 0);
        if (action.dim() == 1) action = torch::unsqueeze(action, 0);
        if (reward.dim() == 1) reward = torch::unsqueeze(reward, 0);
        if (next_state.dim() == 1) next_state = torch::unsqueeze(next_state, 0);
        if (done.dim() == 1) done = torch::unsqueeze(done, 0);
        addExperienceState(std::make_tuple(state, action, reward, next_state, done)); // but first convert them to a tuple
    }

    void addExperienceState(torch::Tensor state, torch::Tensor action, torch::Tensor reward, torch::Tensor next_state, torch::Tensor done, torch::Tensor num_steps)
    {
        done = torch::cat({done, num_steps.to(state.device())}, -1); // concatenate num_steps to state
        addExperienceState(state, action, reward, next_state, done); // then add to
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

    boost::circular_buffer<Experience> circular_buffer{100000}; // max size of buffer = 10000
}; // refer to https://github.com/EmmiOcean/DDPG_LibTorch/blob/master/replayBuffer.h