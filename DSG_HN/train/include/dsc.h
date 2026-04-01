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
#include <random>

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
        int warmup_episodes = 5; // episodes where you just train the global option. The better the pretrained option is, the lower this can be
        int steps_per_episode = 1000;
        int gestation_train_steps = 5000; // TD3 training steps per epoch before validation
        int gestation_n = 10;             // successes required out of 2N validation trials
        int last_k = 10;                  // states collected per successful validation episode
        int refinement_eps = 20;          // eval rollouts from SVM boundary for Phase 3
        double nu = 0.1; // One-class SVM outlier fraction. If classifier is too tight after adding velocity, try 0.15
        int max_option_steps = 20;        // how long an option can execute for
        int max_skills = 6;
        float start_noise_radius = 2.0f;

        std::vector<int> actor_layers = {256, 256, 256};
        std::vector<int> critic_layers = {256, 256, 256};
        std::vector<int> poo_layers = {256, 256, 256};

        float lr_actor = 1e-4f;
        float lr_critic = 3e-4f;
        int max_obstacles = 8;
        int actor_warmup_steps = 0;
        float lr_poo = 1e-4f;
        float tau = 0.005f;
        float gamma = 0.99f;
        int batch_size = 256;
        int actor_update_freq = 2;

        bool render_training = false; // if true, startRender() is called before the training loop (slows training)
        bool verbose = false;         // per-rollout logging inside each episode
        int log_interval = 50;        // print episode summary + skill status table every N episodes

        // Velocity curriculum — velocity_weight is ramped automatically each episode
        float velocity_weight            = 0.0f;  // managed automatically by curriculum; do not set manually
        int   vel_curriculum_start_ep    = 0;     // episode at which velocity_weight starts ramping from 0
        int   vel_curriculum_end_ep      = -1;    // episode at which weight reaches 1.0 (-1 = max_episodes)

        // Reward thresholds — tune from here
        float vel_success_threshold      = 0.4f;  // ~half max vx (1.0 m/s); robot must roughly match approach speed
        float ang_vel_success_threshold  = 0.4f;  // ~40% of max yaw (1.0 rad/s); loose enough to not over-constrain
        float vel_shaping_radius         = 1.0f;  // 4x success radius (0.25m); velocity penalty only in final approach
        float vel_penalty_scale          = 20.0f; // keeps vel penalty smaller than position penalty (/50.0) at equal errors
    };

    DeepSkillChaining(std::shared_ptr<RobotBridgeTrain> robot_bridge,
                      torch::Device device,
                      AbstractedState global_goal,
                      AbstractedState global_start,
                      const std::string &pretrain_actor_path,
                      const std::string &pretrain_critic1_path,
                      const std::string &pretrain_critic2_path,
                      Config cfg);

    // Train the chain backward from the global goal. Returns total skill count excluding the global option
    int train(int max_episodes);

    // Execute one episode using πO over the trained chain. Returns cumulative reward.
    float execute();

    // void save(const std::string &dir) const;
    // void load(const std::string &dir, int num_skills);

    int numSkills() const;

private:
    std::shared_ptr<RobotBridgeTrain> _robot_bridge;
    std::shared_ptr<TrainEnvironment> _env;
    torch::Device _device;
    AbstractedState _global_goal;
    AbstractedState _global_start;

    Config _cfg;
    std::vector<std::shared_ptr<Skill>> _skills;
    PolicyOverOptionsAgent _poo;

    mutable std::mt19937 _rng;

    int _global_option_idx = 0;
    int _unfinished_option_idx = 0;

    void _warmupRollout();
    float _dscRollout();
    std::pair<int, AbstractedState> _pickOption();
    bool _containsGlobalStartState();
    void _makeSkill(bool is_global, std::shared_ptr<Skill> parent);
    void _loadGlobalOption(const std::string &actor_path, const std::string &critic1_path, const std::string &critic2_path);

    float _sampleGaussianDist(float mu, float std); // utility function to sample a gaussian
    std::array<float, 4> _getGaussianQuaternionPerturbation(const std::array<float, 4>& q_orig, float sigma_rad); // utility function to perturb quaternion by random noise

    AbstractedState _sampleStartNearObstacle(); // gaussian sampling (see robot autonomy slides)
    AbstractedState _sampleStartInterpolated(); // randomly sample position linearly interpolated between start and goal (with noise)
    void _validateOption();
    bool _shouldCreateNewOption();

};
