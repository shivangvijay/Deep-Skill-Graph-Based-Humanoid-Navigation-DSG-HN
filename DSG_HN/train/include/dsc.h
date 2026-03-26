#pragma once

#include "skill.h"
#include "agent.h"
#include "environment.h"
#include <torch/torch.h>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <deque>
#include "param.h"
#include "robot_bridge_train.h"
#include "environment.h"
#include <torch/torch.h>
#include <filesystem>
#include <iostream>

// DeepSkillChaining — backward chains skills from the global goal toward the start.
//
// Skill indices:
//   _skills[0] = oG  (pre-trained global option, always available, never retrained)
//   _skills[1] = og1 (terminal chain skill, β = reach global goal)
//   _skills[i] = ogi (β = enter _skills[i-1]'s initiation set)
//
// A std::deque is used so that &_skills[i] pointers remain stable after emplace_back.

class DeepSkillChaining
{
public:
    struct Config
    {
        int steps_per_episode    = 1000;
        int gestation_n          = 5;
        int last_k               = 10;  // states collected per successful gestation episode
        int refinement_eps       = 20;  // eval rollouts from SVM boundary for Phase 3
        double nu                = 0.1; // one-class SVM outlier fraction
        int max_skills           = 6;
        float start_noise_radius = 2.0f;

        std::vector<int> actor_layers  = {128, 256, 128};
        std::vector<int> critic_layers = {128, 256, 128};
        std::vector<int> poo_layers    = {128, 256, 128};

        float lr_actor          = 1e-4f;
        float lr_critic         = 3e-4f;
        float lr_poo            = 1e-4f;
        float tau               = 0.005f;
        float gamma             = 0.99f;
        int   batch_size        = 256;
        int   actor_update_freq = 4;
    };

    DeepSkillChaining(std::shared_ptr<TrainEnvironment> env,
                      torch::Device device,
                      std::array<float, 3> global_goal);

    DeepSkillChaining(std::shared_ptr<TrainEnvironment> env,
                      torch::Device device,
                      std::array<float, 3> global_goal,
                      Config cfg);

    // Load a pre-trained TD3 policy as oG. Must be called before train().
    void loadGlobalOption(const std::string &actor_path,
                          const std::string &critic1_path,
                          const std::string &critic2_path);

    // Train the chain backward from the global goal. Returns total skill count including oG.
    int train();

    // Execute one episode using πO over the trained chain. Returns cumulative reward.
    float execute();

    void save(const std::string &dir) const;
    void load(const std::string &dir, int num_skills);

    int numSkills() const;

private:
    std::shared_ptr<TrainEnvironment> _env;
    torch::Device                     _device;
    std::array<float, 3>              _global_goal;
    Config                            _cfg;
    std::deque<Skill>                 _skills;
    PolicyOverOptionsAgent            _poo;

    Skill _makeSkill();
    bool  _startCovered();
};
