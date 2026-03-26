#pragma once

#include "agent.h"
#include "classifier.h"
#include "environment.h"
#include <torch/torch.h>
#include <string>
#include <vector>
#include <array>
#include <deque>
#include <random>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>

// State tensor layout (see environment.h / robotStateToTensor):
//   [0:35]   q              — joint positions
//   [35:70]  dq             — joint velocities
//   [70:73]  relative_goal  — goal - robot position  (NOT used by classifier)
//   [73:76]  velocity       — linear velocity
//   [76:79]  accel
//   [79:83]  orientation    — quaternion
//   [83:86]  angular_velocity
//   [86+]    obstacle data

struct GestationRecord
{
    std::vector<float> classifier_vec; // 13-dim: [pos(3), vel(3), quat(4), ang_vel(3)]
    AbstractedState    state;          // full navigation state for env interactions
};

class Skill
{
public:
    Skill(std::shared_ptr<TrainEnvironment> env,
          const std::vector<int> &actor_layer_sizes,
          std::vector<int> &critic_layer_sizes,
          torch::Device device,
          float lr_actor, float lr_critic,
          float tau, float gamma,
          int batch_size, int actor_update_freq);

    // DSC/DSG 3-phase initiation set learning.
    // next_skill=nullptr → terminal skill (success = reach global goal).
    // next_skill!=nullptr → non-terminal (success = next_skill->canStart()).
    void learn(int steps_per_episode,
               int gestation_n          = 5,
               int last_k               = 10,
               int refinement_eps       = 20,
               double nu                = 0.1,
               const Skill *next_skill  = nullptr,
               float start_noise_radius = 2.0f);

    bool canStart(const torch::Tensor &state) const;
    void setAlwaysAvailable();

    // Copy TD3 weights from another skill as a warm start before learn().
    void initFromSkill(const Skill &other);

    // Sample a random gestation record as a subgoal for the preceding skill.
    AbstractedState      sampleSubgoalState() const;

    // Sample a start position near this skill's initiation set (with Gaussian noise).
    std::array<float, 3> sampleStartPosition(float noise_radius = 2.0f) const;

    void save(const std::string &actor_path,
              const std::string &critic1_path,
              const std::string &critic2_path,
              const std::string &classifier_path) const;

    void load(const std::string &actor_path,
              const std::string &critic1_path,
              const std::string &critic2_path,
              const std::string &classifier_path);

    TD3Agent &agent();

private:
    std::shared_ptr<TrainEnvironment> _env;
    TD3Agent                          _agent;
    InitiationSetClassifier           _classifier;
    std::vector<GestationRecord>      _gestation_records;
    mutable std::mt19937              _rng;
    bool                              _always_available = false;

    // Build 13-dim classifier feature: [abs_pos(3), vel(3), quat(4), ang_vel(3)]
    std::vector<float> _classifierVec(const torch::Tensor &full_state,
                                       const std::array<float, 3> &abs_pos) const;

    // Unified termination check for gestation and refinement rollouts.
    std::pair<bool, bool> _checkTermination(const torch::Tensor &next_state,
                                             const torch::Tensor &reward,
                                             const torch::Tensor &done,
                                             const Skill *next_skill) const;

    // Phase 1: train policy until gestation_n successes; collect last_k states per success.
    std::vector<GestationRecord> _gestation(int steps_per_episode, int gestation_n, int last_k,
                                             const Skill *next_skill, float start_noise_radius);

    // Phase 2: one-class SVM on collected gestation states.
    void _learnInitialClassifier(const std::vector<GestationRecord> &records, double nu);

    // Phase 3: rollout from SVM boundary states, label, train binary SVM.
    void _refineClassifier(const std::vector<GestationRecord> &records,
                            int refinement_eps, int steps_per_episode,
                            const Skill *next_skill);

    // Pick a Phase-2 SVM support vector as an AbstractedState for Phase-3 spawning.
    AbstractedState _sampleSupportVector(const std::vector<GestationRecord> &records) const;
};
