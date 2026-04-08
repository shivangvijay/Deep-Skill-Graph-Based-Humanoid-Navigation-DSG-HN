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
#include <limits>
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
        double nu = 0.1;                  // one-class SVM outlier fraction
        int max_option_steps = 20;        // how long an option can execute for
        int max_skills = 6;
        float start_noise_radius = 2.0f;

        std::vector<int> actor_layers = {256, 256, 256};
        std::vector<int> critic_layers = {256, 256, 256};
        std::vector<int> poo_layers = {256, 256, 256};

        float lr_actor = 1e-4f;
        float lr_critic = 3e-4f;
        float lr_actor_global = 1e-5;
        float lr_critic_global = 3e-5;
        int max_obstacles = 8;
        int actor_warmup_steps = 0;
        float lr_poo = 1e-4f;
        float tau = 0.005f;
        float gamma = 0.99f;
        int batch_size = 256;
        int actor_update_freq = 2;

        bool render_training = false;           // if true, startRender() is called before the training loop (slows training)
        bool verbose = false;                   // per-rollout logging inside each episode
        bool visualize_initiation_sets = false; // if true, periodically render initiation set visualization
        int log_interval = 50;                  // print episode summary + skill status table every N episodes

        // DSG only: maximum sibling skills allowed under one parent node (ignored by DSC)
        int max_children_per_node = 3;

        // DSG only: run expansion phase every N episodes; consolidation otherwise
        int expansion_freq = 5;

        // DSG only: steps the global option runs as an MPC proxy toward s_rand
        int mpc_steps = 20;

        // DSG only: epsilon-ball radius (metres) defining a goal region node
        float goal_region_epsilon = 1.0f;

        // DSG only: max expansion attempts before falling back to consolidation (paper: 10)
        int max_expansion_tries = 10;
    };

    // TODO: depending on DSG setup may need other constructors that allow you to pass in the global agent
    // without having to save it to a file
    DeepSkillChaining(std::shared_ptr<RobotBridgeTrain> robot_bridge,
                      torch::Device device,
                      AbstractedState global_goal,
                      AbstractedState global_start,
                      const std::string &pretrain_actor_path,
                      const std::string &pretrain_critic1_path,
                      const std::string &pretrain_critic2_path,
                      const std::string &scene_file,
                      Config cfg);

    DeepSkillChaining(std::shared_ptr<RobotBridgeTrain> robot_bridge,
                      torch::Device device,
                      AbstractedState global_goal,
                      AbstractedState global_start,
                      const std::string &scene_file,
                      Config cfg);

    // Train the chain backward from the global goal. Returns total skill count excluding the global option
    virtual int train(int max_episodes);

    // Execute one episode using πO over the trained chain. Returns cumulative reward.
    float execute();

    void visualizeInitiationSets();

    virtual void save(const std::string &dir) const;
    virtual void load(const std::string &dir, const std::string &scene_file);

    int numSkills() const;

protected:
    std::shared_ptr<RobotBridgeTrain> _robot_bridge;
    std::shared_ptr<TrainEnvironment> _env;
    torch::Device _device;
    AbstractedState _global_goal;
    AbstractedState _global_start;
    std::string _scene_file_path;

    Config _cfg;
    std::vector<std::shared_ptr<Skill>> _skills;
    PolicyOverOptionsAgent _poo;

    mutable std::mt19937 _rng;

    int _global_option_idx = 0;
    int _unfinished_option_idx = 0;

    void _warmupRollout();
    float _dscRollout(bool eval);
    virtual std::pair<int, AbstractedState> _pickOption(bool eval);
    bool _containsGlobalStartState();
    virtual void _makeSkill(bool is_global, std::shared_ptr<Skill> parent);
    void _loadGlobalOption(const std::string &actor_path, const std::string &critic1_path, const std::string &critic2_path);

    float _sampleGaussianDist(float mu, float std);
    std::array<float, 4> _getGaussianQuaternionPerturbation(const std::array<float, 4> &q_orig, float sigma_rad);

    AbstractedState _sampleStartNearObstacle();
    AbstractedState _sampleStartInterpolated();
    AbstractedState _sampleStartNearBoundary();
    virtual void _validateOption();
    virtual bool _shouldCreateNewOption();
};
