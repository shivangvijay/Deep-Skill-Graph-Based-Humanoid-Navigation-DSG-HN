#pragma once

#include "agent.h"
#include "classifier.h"
#include "environment.h"
#include <torch/torch.h>
#include <string>
#include <vector>
#include <array>
#include <deque>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>

// State tensor layout (see environment.h / robotStateToTensor):
//   [0:35]   q              — joint positions
//   [35:70]  dq             — joint velocities
//   [70:73]  relative_goal  — goal - robot position  (NOT used by classifier)
//   [73:76]  velocity       — linear velocity
//   [76:79]  accel
//   [79:83]  orientation    — quaternion
//   [83:86]  angular_velocity
//   [86+]    obstacle data
//
// A state from a successful gestation episode, stored in two forms:
//   classifier_vec — 13D feature used by the SVM (pos, vel, orient, ang_vel in world frame)
//   state          — full state for env interactions (setGoal, resetTo)
// Both encode the same physical state; classifier_vec uses absolute position
// while AbstractedState is used directly by the environment API.
struct GestationRecord
{
    std::vector<float> classifier_vec; // 13-dim: [pos(3), vel(3), quat(4), ang_vel(3)]
    AbstractedState    state;          // (pos, quat, vel, ang_vel)
};

class Skill
{
public:
    Skill(
        std::shared_ptr<TrainEnvironment> env,
        const std::vector<int> &actor_layer_sizes,
        std::vector<int> &critic_layer_sizes,
        torch::Device device,
        float lr_actor, float lr_critic,
        float tau, float gamma,
        int batch_size, int actor_update_freq)
        : _env(env)
        , _agent(env, actor_layer_sizes, critic_layer_sizes, device,
                 lr_actor, lr_critic, tau, gamma, batch_size, actor_update_freq)
        , _rng(std::random_device{}())
    {}

    // DSC/DSG 3-phase initiation set learning.
    //
    // next_skill: the skill whose initiation set defines "success" for this skill.
    //   - nullptr → terminal skill: success = reach global goal (env-controlled)
    //   - non-null → non-terminal: success = next_skill->canStart(next_state)
    //     Each episode's goal is sampled from next_skill's gestation records so that
    //     the dense reward and the relative_goal state input point in the right direction.
    void learn(
        int steps_per_episode,
        int gestation_n          = 5, 
        int last_k               = 10,
        int refinement_eps       = 20,
        double nu                = 0.1,
        const Skill *next_skill  = nullptr,
        float start_noise_radius = 2.0f)
    {
        std::cout << "=== Phase 1: Gestation ===" << std::endl;
        _gestation_records = _gestation(steps_per_episode, gestation_n, last_k, next_skill, start_noise_radius);

        std::cout << "=== Phase 2: Training one-class classifier ("
                  << _gestation_records.size() << " states) ===" << std::endl;
        _learnInitialClassifier(_gestation_records, nu);
        std::cout << "One-class classifier trained. Support vectors: "
                  << _classifier.getSupportVectors(13).size() << std::endl;

        std::cout << "=== Phase 3: Refining classifier ("
                  << refinement_eps << " rollouts from SVM boundary) ===" << std::endl;
        _refineClassifier(_gestation_records, refinement_eps, steps_per_episode, next_skill);
        std::cout << "Binary classifier trained." << std::endl;
    }

    // Returns true if the given state is in this skill's initiation set.
    // Uses the current absolute robot position from the shared environment.
    bool canStart(const torch::Tensor &state) const
    {
        auto [pos, quat] = _env->getRobotPose();
        return _classifier.classify(_classifierVec(state, pos));
    }

    // Returns a random gestation record as a full AbstractedState (pos, quat, vel, ang_vel).
    // Used as the per-episode subgoal for the skill preceding this one in the chain,
    // so the reward gradient and velocity error term point at realistic entry conditions
    // rather than a zero-velocity target derived from position alone.
    AbstractedState sampleSubgoalState() const
    {
        std::uniform_int_distribution<size_t> dist(0, _gestation_records.size() - 1);
        return _gestation_records[dist(_rng)].state;
    }

    // Returns a random gestation-record position with Gaussian noise applied in x/y.
    // Used to spawn the preceding skill's episodes near this skill's initiation set.
    std::array<float, 3> sampleStartPosition(float noise_radius = 2.0f) const
    {
        std::uniform_int_distribution<size_t> dist(0, _gestation_records.size() - 1);
        std::normal_distribution<float> gauss(0.0f, noise_radius);
        auto pos = _gestation_records[dist(_rng)].state.position;
        pos[0] += gauss(_rng);
        pos[1] += gauss(_rng);
        return pos;
    }

    // Save agent and classifier to files
    void save(const std::string &actor_path,
              const std::string &critic1_path,
              const std::string &critic2_path,
              const std::string &classifier_path) const
    {
        torch::save(_agent.actor_local,    actor_path);
        torch::save(_agent.critic_local_1, critic1_path);
        torch::save(_agent.critic_local_2, critic2_path);
        _classifier.save(classifier_path);
    }

    // Load agent and classifier from files
    void load(const std::string &actor_path,
              const std::string &critic1_path,
              const std::string &critic2_path,
              const std::string &classifier_path)
    {
        torch::load(_agent.actor_local,    actor_path);
        torch::load(_agent.critic_local_1, critic1_path);
        torch::load(_agent.critic_local_2, critic2_path);
        _classifier.load(classifier_path);
    }

    TD3Agent &agent() { return _agent; }

private:
    std::shared_ptr<TrainEnvironment> _env;
    TD3Agent                          _agent;
    InitiationSetClassifier           _classifier;
    std::vector<GestationRecord>      _gestation_records; // persisted after learn()
    mutable std::mt19937              _rng;

    // Build the 13-dim classifier feature vector.
    // abs_pos: absolute robot position (NOT relative to goal — classifier is goal-independent).
    std::vector<float> _classifierVec(const torch::Tensor &full_state,
                                       const std::array<float, 3> &abs_pos) const
    {
        auto flat = full_state.flatten().contiguous();
        const float *d = flat.data_ptr<float>();
        std::vector<float> out;
        out.reserve(13);
        out.push_back(abs_pos[0]);                        // absolute x
        out.push_back(abs_pos[1]);                        // absolute y
        out.push_back(abs_pos[2]);                        // absolute z
        for (int i = 73; i < 76; ++i) out.push_back(d[i]); // velocity
        for (int i = 79; i < 83; ++i) out.push_back(d[i]); // orientation
        for (int i = 83; i < 86; ++i) out.push_back(d[i]); // angular_velocity
        return out;
    }

    // Unified termination check used in both _gestation and _refineClassifier.
    // Returns {should_terminate, success}.
    std::pair<bool, bool> _checkTermination(const torch::Tensor &next_state,
                                             const torch::Tensor &reward,
                                             const torch::Tensor &done,
                                             const Skill *next_skill) const
    {
        bool in_next_set = (next_skill != nullptr) && next_skill->canStart(next_state);
        bool env_done    = done.data_ptr<float>()[0] > 0.5f;
        float r          = reward.data_ptr<float>()[0];

        // For non-terminal skills: suppress the environment's goal-proximity termination
        // (reward > 0 from distance < 0.5m). The sampled subgoal is just a shaping target;
        // only canStart(), collision (r <= 0 + done), or timeout actually end the episode.
        bool hard_done = env_done && (r <= 0.0f);         // collision or timeout
        bool terminal_goal = (next_skill == nullptr) && env_done; // terminal: env controls all

        bool should_terminate = in_next_set || hard_done || terminal_goal;
        bool success = in_next_set || (terminal_goal && r > 0.0f);
        return {should_terminate, success};
    }

    // Phase 1: train policy until gestation_n successes; collect last_k states per success
    std::vector<GestationRecord> _gestation(int steps_per_episode, int gestation_n, int last_k,
                                             const Skill *next_skill, float start_noise_radius)
    {
        std::vector<GestationRecord> all_records; // storage for all collected gestation states across episodes
        int success_count = 0;
        int episode = 0;

        while (success_count < gestation_n)
        {
            // For non-terminal skills, sample a fresh full-state subgoal each episode.
            // Using vel/ang_vel from the gestation record aligns the reward gradient with
            // the actual entry dynamics that trigger next_skill->canStart().
            // The terminal skill's goal is the fixed global goal set by DSC.
            if (next_skill != nullptr)
            {
                auto sg = next_skill->sampleSubgoalState();
                _env->setGoal(sg.position, sg.orientation, sg.velocity, sg.angular_velocity);
            }

            // Spawn near the target region:
            // - non-terminal: near next skill's initiation set
            // - terminal: near the fixed global goal
            std::array<float, 3> target_pos = (next_skill != nullptr)
                ? next_skill->sampleStartPosition(start_noise_radius)
                : _env->getGoalPosition();
            std::normal_distribution<float> gauss(0.0f, start_noise_radius);
            std::array<float, 3> start_pos = {
                target_pos[0] + gauss(_rng),
                target_pos[1] + gauss(_rng),
                target_pos[2]
            };
            torch::Tensor state = _env->resetTo(start_pos, {1.0f, 0.0f, 0.0f, 0.0f});
            std::deque<GestationRecord> window; // rolling window, max size last_k

            bool success = false;
            for (int step = 0; step < steps_per_episode; ++step)
            {
                auto [scaled_action, action] = _agent.getAction(state);
                auto [next_state, reward, done] = _env->step(scaled_action);
                _agent.addExperience(state, action, reward, next_state, done);
                _agent.learn();

                auto [pos, quat] = _env->getRobotPose();
                auto cv = _classifierVec(state, pos);
                // cv layout: [0:3]=pos, [3:6]=vel, [6:10]=quat, [10:13]=ang_vel
                AbstractedState rec_state = {
                    pos,
                    quat,
                    {cv[3], cv[4], cv[5]},   // velocity
                    {cv[10], cv[11], cv[12]}  // angular_velocity
                };
                window.push_back({cv, rec_state});
                if ((int)window.size() > last_k)
                    window.pop_front();

                auto [terminate, suc] = _checkTermination(next_state, reward, done, next_skill);
                if (terminate)
                {
                    success = suc;
                    break;
                }
                state = next_state;
            }

            if (success)
            {
                for (auto &r : window)
                    all_records.push_back(std::move(r));
                success_count++;
                std::cout << "  Gestation success " << success_count << "/" << gestation_n
                          << " (episode " << episode + 1 << ")" << std::endl;
            }
            episode++;
        }
        return all_records;
    }

    // Phase 2: one-class SVM on collected gestation states
    void _learnInitialClassifier(const std::vector<GestationRecord> &records, double nu)
    {
        std::vector<std::vector<float>> states;
        states.reserve(records.size());
        for (const auto &r : records)
            states.push_back(r.classifier_vec);
        _classifier.trainOneClass(states, nu);
    }

    // Phase 3: rollout from SVM boundary states (support vectors), label, train binary SVM
    void _refineClassifier(const std::vector<GestationRecord> &records,
                            int refinement_eps,
                            int steps_per_episode,
                            const Skill *next_skill)
    {
        std::vector<std::vector<float>> all_states;
        std::vector<int>               all_labels;

        for (int ep = 0; ep < refinement_eps; ++ep)
        {
            auto [sv_pos, sv_quat, sv_vel, sv_ang_vel] = _sampleSupportVector(records);

            if (next_skill != nullptr)
            {
                auto [sg_pos, sg_quat, sg_vel, sg_ang_vel] = next_skill->sampleSubgoalState();
                _env->setGoal(sg_pos, sg_quat, sg_vel, sg_ang_vel);
            }

            torch::Tensor state = _env->resetTo(sv_pos, sv_quat, sv_vel, sv_ang_vel);

            std::vector<std::vector<float>> visited;
            bool success = false;

            for (int step = 0; step < steps_per_episode; ++step)
            {
                auto [cur_pos, cur_quat] = _env->getRobotPose();
                visited.push_back(_classifierVec(state, cur_pos));

                auto [scaled_action, dummy] = _agent.getAction(state, /*eval=*/true);
                auto [next_state, reward, done] = _env->step(scaled_action);

                auto [terminate, suc] = _checkTermination(next_state, reward, done, next_skill);
                if (terminate)
                {
                    success = suc;
                    break;
                }
                state = next_state;
            }

            int label = success ? +1 : -1;
            for (auto &s : visited)
            {
                all_states.push_back(s);
                all_labels.push_back(label);
            }

            std::cout << "  Refinement episode " << ep + 1 << "/" << refinement_eps
                      << " — " << (success ? "success" : "failure") << std::endl;
        }

        if (!all_states.empty())
            _classifier.train(all_states, all_labels);
        else
            std::cerr << "Warning: no refinement data collected." << std::endl;
    }

    // Randomly pick a Phase-2 SVM support vector and decode it into an AbstractedState.
    // SVs are the literal boundary of the one-class SVM — the most informative spawn states
    // for Phase 3 refinement. Each call picks a different SV, giving episode diversity.
    // Classifier vec layout: [0:3]=pos, [3:6]=vel, [6:10]=quat(w,x,y,z), [10:13]=ang_vel
    // Falls back to the gestation record closest to the boundary if SVs unavailable.
    AbstractedState _sampleSupportVector(const std::vector<GestationRecord> &records) const
    {
        auto svs = _classifier.getSupportVectors(13);
        if (!svs.empty())
        {
            std::uniform_int_distribution<size_t> dist(0, svs.size() - 1);
            const auto &sv = svs[dist(_rng)];
            return {
                std::array<float,3>{sv[0], sv[1], sv[2]},
                std::array<float,4>{sv[6], sv[7], sv[8], sv[9]},
                std::array<float,3>{sv[3], sv[4], sv[5]},
                std::array<float,3>{sv[10], sv[11], sv[12]}
            };
        }
        // Fallback: gestation record with minimum |decisionValue| - closest to the boundary, i.e., most ambiguous under the current classifier
        size_t best_idx = 0;
        double best_dv  = std::numeric_limits<double>::max();
        for (size_t i = 0; i < records.size(); ++i)
        {
            double dv = std::abs(_classifier.decisionValue(records[i].classifier_vec));
            if (dv < best_dv) { best_dv = dv; best_idx = i; }
        }
        return records[best_idx].state;
    }
};
