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
        double optimistic_svc_c = 1.0;    // phase-2 optimistic SVC C
        double optimistic_svc_gamma = 0.5; // phase-2 optimistic SVC gamma
        bool optimistic_svc_balance_classes = true; // phase-2 optimistic SVC class balancing
        double optimistic_svc_positive_margin_tolerance = 0.0; // include samples with decisionValue >= -tol as optimistic positives
        double pessimistic_ocsvm_gamma = 0.5;      // one-class gamma for pessimistic classifier
        double optimistic_ocsvm_gamma = 0.5;       // one-class gamma for optimistic classifier in phase-1
        double optimistic_ocsvm_nu_divisor = 10.0; // optimistic phase-1 nu = nu / divisor
        float subgoal_robustness_tolerance = 0.25f; // neighborhood tolerance in sampleSubgoalState
        int negative_samples_per_failure = 1; // number of failure states added as negatives per failed rollout
        int max_option_steps = 20;        // how long an option can execute for
        int max_skills = 6;
        float start_noise_radius = 2.0f;
        float val_accuracy_threshold = 0.8f;
        float success_radius = 0.5f;      // environment goal termination radius (meters)

        std::vector<int> actor_layers = {256, 256, 256};
        std::vector<int> critic_layers = {256, 256, 256};
        std::vector<int> poo_layers = {256, 256, 256};

        float lr_actor = 1e-4f;
        float lr_critic = 3e-4f;
        float lr_actor_global = 1e-5;
        float lr_critic_global = 3e-5;
        int actor_warmup_steps = 0;
        float lr_poo = 1e-4f;
        float tau = 0.005f;
        float gamma = 0.99f;
        int batch_size = 256;
        int poo_batch_size = 16;
        int actor_update_freq = 2;

        bool render_training = false;           // if true, startRender() is called before the training loop (slows training)
        bool verbose = false;                   // per-rollout logging inside each episode
        bool visualize_initiation_sets = false; // if true, periodically render initiation set visualization
        int log_interval = 50;                  // print episode summary + skill status table every N episodes
    };

    struct DSCProblem
    {
        int v_d;
        int v_a;
        int v_g;
        std::vector<int> dsc_chain; // skill chain from v_d to v_a, excluding v_d and v_a themselves
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
                      Config cfg, bool eval=false, bool make_goal_option=true);

    DeepSkillChaining(std::shared_ptr<RobotBridgeTrain> robot_bridge,
                      torch::Device device,
                      AbstractedState global_goal,
                      AbstractedState global_start,
                      const std::string &scene_file,
                      Config cfg, bool eval=false, bool make_goal_option=true);

    // Train the chain backward from the global goal. Returns total skill count excluding the global option
    virtual int train(int max_episodes);

    // Execute one episode using πO over the trained chain. Returns cumulative reward.
    virtual float execute();

    void visualizeInitiationSets();

    virtual void save(const std::string &dir) const;
    virtual void load(const std::string &dir, const std::string &scene_file);

    int numSkills() const;
    void setEvalMode(bool eval);

protected:
    std::shared_ptr<RobotBridgeTrain> _robot_bridge;
    std::shared_ptr<TrainEnvironment> _env;
    torch::Device _device;
    AbstractedState _global_goal;
    AbstractedState _global_start;
    std::string _scene_file_path;
    bool _eval;

    Config _cfg;
    std::vector<std::shared_ptr<Skill>> _skills;
    PolicyOverOptionsAgent _poo;

    mutable std::mt19937 _rng;

    int _global_option_idx = 0;
    int _unfinished_option_idx = 0;

    void _warmupRollout();
    virtual float _dscRollout();
    virtual std::pair<int, AbstractedState> _pickOption();
    bool _containsGlobalStartState();
    virtual void _makeSkill(bool is_global, std::shared_ptr<Skill> parent);
    virtual void _makeSkill(bool is_global, std::shared_ptr<Skill> parent, const AbstractedState& local_goal);
    void _loadGlobalOption(const std::string &actor_path, const std::string &critic1_path, const std::string &critic2_path);

    float _sampleGaussianDist(float mu, float std);
    std::array<float, 4> _getGaussianQuaternionPerturbation(const std::array<float, 4> &q_orig, float sigma_rad);

    AbstractedState _sampleStartNearObstacle();
    AbstractedState _sampleStartInterpolated();
    AbstractedState _sampleStartNearBoundary();
    virtual AbstractedState _sampleSpawnState(); // default returns _global_start
    virtual void _validateOption();
    virtual bool _shouldCreateNewOption();
};
