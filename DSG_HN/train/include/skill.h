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
#include <string>
#include <functional>
#include <math.h>
#include <torch/torch.h>

// When false, linear and angular velocity dimensions are zeroed in the classifier feature vector
// and stripped from sampled subgoal states. Set to true to re-enable full 13D representation.
// Safe to flip: only affects _classifierVec() and sampleSubgoalState(); no structural changes needed.
static constexpr bool kUseVelocityInClassifier = false;

struct Transition
{
    RobotState state;
    torch::Tensor action;
    RobotState next_state;
    bool in_collision;
};

struct GestationRecord
{
    std::vector<float> classifier_vec; // 13-dim: [pos(3), vel(3), quat(4), ang_vel(3)]
    AbstractedState state;             // full navigation state for env interactions
};

class Skill
{
public:
    Skill(int id, std::shared_ptr<TrainEnvironment> env,
          const std::vector<int> &actor_layer_sizes,
          const std::vector<int> &critic_layer_sizes,
          torch::Device device,
          float lr_actor, float lr_critic,
          float tau, float gamma, int max_obstacles, int actor_warmup_steps,
          int batch_size, int actor_update_freq, int k, int max_steps, double nu,
          std::shared_ptr<Skill> parent, int gestation_period, bool is_global, AbstractedState global_goal,
          std::shared_ptr<Skill> global_option);

    AbstractedState getLocalGoal();

    std::tuple<int, float, bool, torch::Tensor, torch::Tensor> rollout(const AbstractedState &goal);
    void validateSkill(bool success); // mark if skill is validated

    bool canStart(const RobotState &state) const;
    bool canStart(const AbstractedState &state) const;

    // Pessimistic initiation check — tighter boundary used for termination and subgoal sampling.
    // Returns true during gestation (mirrors Python's pessimistic_is_init_true).
    bool canStartPessimistic(const RobotState &state) const;
    bool canStartPessimistic(const AbstractedState &state) const;

    float distanceToState(const AbstractedState &state) const;

    // Copy TD3 weights from another skill as a warm start before learn().
    void initFromSkill(std::shared_ptr<Skill> other);

    // Sample a random gestation record as a subgoal for the preceding skill.
    AbstractedState sampleSubgoalState(bool uniform) const;
    std::string getTrainingPhase() const;
    bool atTermination(const AbstractedState &goal) const;

    void save(const std::string &actor_path,
              const std::string &critic1_path,
              const std::string &critic2_path,
              const std::string &classifier_path) const;

    void load(const std::string &actor_path,
              const std::string &critic1_path,
              const std::string &critic2_path,
              const std::string &classifier_path);

    TD3Agent &agent();

    int goalHits() const { return _goal_hits; }
    int gestationPeriod() const { return _gestation_period; }
    const std::vector<GestationRecord>& getPositiveGestationRecords() const { return _positive_gestation_records; }

private:
    std::shared_ptr<TrainEnvironment> _env;
    std::shared_ptr<Skill> _parent;
    std::shared_ptr<Skill> _global_option;

    bool _is_global;
    TD3Agent _agent;
    InitiationSetClassifier _classifier;             // optimistic boundary: loose OneClassSVM or balanced SVC
    InitiationSetClassifier _pessimistic_classifier; // tight boundary: OneClassSVM(nu) or OneClassSVM re-fit on SVC-positives
    std::vector<GestationRecord> _positive_gestation_records;
    std::vector<std::vector<float>> _gestation_vecs;
    std::vector<int> _gestation_labels;
    bool _has_negative_gestation = false;
    bool _validated = false;

    AbstractedState _global_goal;
    mutable std::mt19937 _rng;
    bool _always_available = false;
    int _goal_hits = 0;
    int _gestation_period;
    int _max_steps = 0;
    int _k = 0;
    double _nu;
    float _gamma;

    int _id;

    // Build 13-dim classifier feature: [abs_pos(3), vel(3), quat(4), ang_vel(3)]
    std::vector<float> _classifierVec(const RobotState &state) const;
    std::vector<float> _classifierVec(const AbstractedState &state) const;

    // Unified termination check for gestation and refinement rollouts.
    bool _atLocalGoal(const AbstractedState &goal) const;

    void _fitClassifier(const std::vector<GestationRecord> &states, bool term_success);

    float _euclideanDistance(const std::array<float, 3> &a, const std::array<float, 3> &b, bool sqrt) const;
    float _euclideanDistance(const std::array<float, 4> &a, const std::array<float, 4> &b, bool sqrt) const;
    void _herUpdate(const std::vector<Transition>& trajectory);
};
