#pragma once

// transition_model.hpp
//
// Self-contained header for the Gaussian delta transition model and random-shooting MPC.
// Extracted from transition_mpc_eval.cpp / transition_train_gaussian_delta.cpp so that
// DeepSkillGraph can use the model inline without linking against a separate binary.
//
// Architecture: GaussianMLP  (kTM_InputDim → μ/logσ² of kTM_OutputDim-D delta)
//   input  = [qw, 0, 0, qz, vx, vy, oz, joint_pos×35, joint_vel×35]  (77-D)
//            concatenated with normalised action (3-D)
//   output = delta of [x, y, yaw, vx, vy, oz, joint_pos×35, joint_vel×35] (76-D)

#include <torch/torch.h>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot_bridge.h"  // Obstacle struct

// ─── Dimension constants ──────────────────────────────────────────────────────
static constexpr int kTM_JointDim      = 35;
static constexpr int kTM_InputStateDim = 7 + (2 * kTM_JointDim);         // 77
static constexpr int kTM_ActionDim     = 3;
static constexpr int kTM_InputDim      = kTM_InputStateDim + kTM_ActionDim; // 80
static constexpr int kTM_OutputDim     = 6 + (2 * kTM_JointDim);          // 76

// ─── State / action / plan types ─────────────────────────────────────────────

struct RolloutState
{
    double x = 0.0, y = 0.0, z = 0.0;
    double yaw = 0.0;
    double vx = 0.0, vy = 0.0, oz = 0.0;
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
};

struct ActionSample
{
    double vx = 0.0, vy = 0.0, yaw = 0.0;
};

struct PlanResult
{
    double                    cost      = std::numeric_limits<double>::infinity();
    bool                      collision = false;
    std::vector<ActionSample> actions;
    std::vector<RolloutState> states;
};

// ─── GaussianMLP ─────────────────────────────────────────────────────────────
// Architecture matches transition_train_gaussian_delta.cpp / transition_mpc_eval.cpp.
// Saved checkpoints must be produced by those training scripts.

struct GaussianMLPImpl : torch::nn::Module
{
    GaussianMLPImpl(int64_t input_dim, int64_t output_dim, double dropout = 0.2)
    {
        trunk = register_module("trunk", torch::nn::Sequential());
        trunk->push_back(torch::nn::Linear(input_dim, 256));
        trunk->push_back(torch::nn::LayerNorm(torch::nn::LayerNormOptions({256})));
        trunk->push_back(torch::nn::SiLU());
        trunk->push_back(torch::nn::Dropout(torch::nn::DropoutOptions(dropout)));
        trunk->push_back(torch::nn::Linear(256, 256));
        trunk->push_back(torch::nn::LayerNorm(torch::nn::LayerNormOptions({256})));
        trunk->push_back(torch::nn::SiLU());
        trunk->push_back(torch::nn::Dropout(torch::nn::DropoutOptions(dropout)));
        trunk->push_back(torch::nn::Linear(256, 128));
        trunk->push_back(torch::nn::LayerNorm(torch::nn::LayerNormOptions({128})));
        trunk->push_back(torch::nn::SiLU());
        trunk->push_back(torch::nn::Dropout(torch::nn::DropoutOptions(dropout)));
        mu_head      = register_module("mu_head",      torch::nn::Linear(128, output_dim));
        log_var_head = register_module("log_var_head", torch::nn::Linear(128, output_dim));

        apply([](torch::nn::Module &m) {
            if (auto *linear = m.as<torch::nn::Linear>())
            {
                torch::NoGradGuard ng;
                torch::nn::init::xavier_uniform_(linear->weight);
                torch::nn::init::constant_(linear->bias, 0.0);
            }
        });
    }

    std::pair<torch::Tensor, torch::Tensor> forward(torch::Tensor x)
    {
        auto h = trunk->forward(x);
        return {mu_head->forward(h), log_var_head->forward(h)};
    }

    torch::nn::Sequential trunk{nullptr};
    torch::nn::Linear     mu_head{nullptr}, log_var_head{nullptr};
};
TORCH_MODULE(GaussianMLP);

// ─── Normaliser ───────────────────────────────────────────────────────────────
// Normaliser file format: each line is "<key> <float> <float> ...".
// Expected keys: input_mean, input_std, action_mean, action_std, target_mean, target_std.
// These match the output of transition_train_gaussian_delta.cpp.

struct TransitionNormaliser
{
    torch::Tensor input_mean, input_std;
    torch::Tensor action_mean, action_std;
    torch::Tensor target_mean, target_std;

    torch::Tensor norm_input (const torch::Tensor &x) const { return (x - input_mean)  / input_std;  }
    torch::Tensor norm_action(const torch::Tensor &x) const { return (x - action_mean) / action_std; }
    torch::Tensor unnorm_target(const torch::Tensor &x) const { return x * target_std + target_mean; }

    static TransitionNormaliser load(const std::string &path)
    {
        std::ifstream in(path);
        if (!in.is_open())
            throw std::runtime_error("TransitionNormaliser: cannot open " + path);

        std::unordered_map<std::string, torch::Tensor> tensors;
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty()) continue;
            std::istringstream ss(line);
            std::string name; ss >> name;
            std::vector<float> vals;
            float v;
            while (ss >> v) vals.push_back(v);
            auto t = torch::empty({static_cast<int64_t>(vals.size())}, torch::kFloat32);
            std::memcpy(t.data_ptr<float>(), vals.data(), vals.size() * sizeof(float));
            tensors.emplace(name, std::move(t));
        }

        TransitionNormaliser n;
        n.input_mean  = tensors.at("input_mean");
        n.input_std   = tensors.at("input_std");
        n.action_mean = tensors.at("action_mean");
        n.action_std  = tensors.at("action_std");
        n.target_mean = tensors.at("target_mean");
        n.target_std  = tensors.at("target_std");
        return n;
    }
};

// ─── Pure helpers (inline to avoid ODR issues across TUs) ────────────────────

inline double tmWrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

inline torch::Tensor rolloutStateToFeatures(const RolloutState &s)
{
    const float qw = static_cast<float>(std::cos(s.yaw * 0.5));
    const float qz = static_cast<float>(std::sin(s.yaw * 0.5));
    std::vector<float> row = { qw, 0.0f, 0.0f, qz,
                               static_cast<float>(s.vx),
                               static_cast<float>(s.vy),
                               static_cast<float>(s.oz) };
    for (double v : s.joint_pos) row.push_back(static_cast<float>(v));
    for (double v : s.joint_vel) row.push_back(static_cast<float>(v));
    return torch::tensor(row, torch::kFloat32);
}

inline torch::Tensor actionToTensor(const ActionSample &a)
{
    return torch::tensor({ static_cast<float>(a.vx),
                           static_cast<float>(a.vy),
                           static_cast<float>(a.yaw) }, torch::kFloat32);
}

// Predict one-step state delta using the trained Gaussian MLP.
inline torch::Tensor predictDelta(GaussianMLP &model,
                                   const TransitionNormaliser &norm,
                                   const RolloutState &state,
                                   const torch::Tensor &action_row,
                                   torch::Device device)
{
    auto feats = rolloutStateToFeatures(state).unsqueeze(0);
    auto input = torch::cat({norm.norm_input(feats),
                             norm.norm_action(action_row.unsqueeze(0))}, 1).to(device);
    torch::NoGradGuard ng;
    auto [mu, log_var] = model->forward(input);
    (void)log_var;
    return norm.unnorm_target(mu.to(torch::kCPU)).squeeze(0);
}

// Apply the predicted delta to a RolloutState in-place.
inline void applyDelta(RolloutState &s, const torch::Tensor &delta)
{
    s.x   += delta[0].item<double>();
    s.y   += delta[1].item<double>();
    s.yaw  = tmWrapAngle(s.yaw + delta[2].item<double>());
    s.vx  += delta[3].item<double>();
    s.vy  += delta[4].item<double>();
    s.oz  += delta[5].item<double>();
    for (int i = 0; i < kTM_JointDim; ++i)
        s.joint_pos[i] += delta[6 + i].item<double>();
    for (int i = 0; i < kTM_JointDim; ++i)
        s.joint_vel[i] += delta[6 + kTM_JointDim + i].item<double>();
}

// ─── Obstacle collision helpers ───────────────────────────────────────────────

inline double tmObstacleRadius(const Obstacle &obs)
{
    if (obs.type.find("box") != std::string::npos)
        return std::sqrt(static_cast<double>(obs.size[0]) * obs.size[0] +
                         static_cast<double>(obs.size[1]) * obs.size[1]);
    return static_cast<double>(obs.size[0]);
}

inline bool tmStateInCollision(const RolloutState &s,
                                const std::vector<Obstacle> &obstacles,
                                double base_radius,
                                double clearance)
{
    for (const auto &obs : obstacles)
    {
        const double dx = s.x - obs.position[0];
        const double dy = s.y - obs.position[1];
        if (std::sqrt(dx*dx + dy*dy) <= base_radius + tmObstacleRadius(obs) + clearance)
            return true;
    }
    return false;
}

// ─── Evaluate a single candidate action sequence (imagined rollout) ───────────

inline PlanResult evaluateSequence(GaussianMLP &model,
                                    const TransitionNormaliser &norm,
                                    const RolloutState &initial,
                                    const std::vector<ActionSample> &actions,
                                    const std::vector<Obstacle> &obstacles,
                                    torch::Device device,
                                    double goal_x, double goal_y,
                                    double base_radius, double clearance,
                                    double collision_penalty,
                                    double action_penalty,
                                    double smoothness_penalty,
                                    double goal_weight)
{
    PlanResult result;
    result.cost = 0.0;
    result.actions = actions;
    result.states.reserve(actions.size() + 1);
    result.states.push_back(initial);

    RolloutState cur = initial;
    ActionSample prev{};

    for (std::size_t i = 0; i < actions.size(); ++i)
    {
        const auto &a = actions[i];
        auto delta = predictDelta(model, norm, cur, actionToTensor(a), device);
        applyDelta(cur, delta);
        result.states.push_back(cur);

        const double dx = cur.x - goal_x, dy = cur.y - goal_y;
        result.cost += goal_weight * (dx*dx + dy*dy);
        result.cost += action_penalty * (a.vx*a.vx + a.vy*a.vy + a.yaw*a.yaw);
        if (i > 0)
        {
            const double dvx = a.vx - prev.vx, dvy = a.vy - prev.vy, dyaw = a.yaw - prev.yaw;
            result.cost += smoothness_penalty * (dvx*dvx + dvy*dvy + dyaw*dyaw);
        }
        if (tmStateInCollision(cur, obstacles, base_radius, clearance))
        {
            result.cost += collision_penalty;
            result.collision = true;
            break;
        }
        prev = a;
    }
    return result;
}

// ─── Random-shooting MPC ─────────────────────────────────────────────────────
// Samples `num_candidates` random action sequences of length `horizon` and
// returns the one with the lowest imagined cost (distance to goal + penalties).

inline PlanResult randomShootMPC(GaussianMLP &model,
                                  const TransitionNormaliser &norm,
                                  const RolloutState &initial,
                                  const std::vector<Obstacle> &obstacles,
                                  torch::Device device,
                                  double goal_x, double goal_y,
                                  int horizon, int num_candidates,
                                  const std::array<double, 3> &action_limits,
                                  double base_radius, double clearance,
                                  double collision_penalty,
                                  double action_penalty,
                                  double smoothness_penalty,
                                  double goal_weight,
                                  unsigned int seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    PlanResult best;

    for (int c = 0; c < num_candidates; ++c)
    {
        std::vector<ActionSample> actions;
        actions.reserve(horizon);
        for (int h = 0; h < horizon; ++h)
            actions.push_back({ unit(rng) * action_limits[0],
                                unit(rng) * action_limits[1],
                                unit(rng) * action_limits[2] });

        auto result = evaluateSequence(model, norm, initial, actions, obstacles, device,
                                        goal_x, goal_y, base_radius, clearance,
                                        collision_penalty, action_penalty,
                                        smoothness_penalty, goal_weight);
        if (result.cost < best.cost)
            best = std::move(result);
    }
    return best;
}
