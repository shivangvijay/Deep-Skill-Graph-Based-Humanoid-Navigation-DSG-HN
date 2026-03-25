#pragma once

#include "skill.h"
#include "agent.h"
#include "environment.h"
#include <torch/torch.h>
#include <deque>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <iostream>

// DeepSkillChaining — outer DSC loop.
//
// Skills are chained backward from the global goal:
//   _skills[0] = oG  (terminal, seeks global goal, next_skill = nullptr)
//   _skills[i]       (seeks _skills[i-1]'s initiation set)
//
// A std::deque is used so that references to existing elements remain stable
// after emplace_back — &_skills[i-1] stays valid when _skills[i] is added.

class DeepSkillChaining
{
public:
    struct Config
    {
        int steps_per_episode     = 1000; // max steps per episode during skill learning and execution
        int gestation_n           = 5; // number of successes to achieve during gestation phase of skill learning before moving on to classifier training
        int last_k                = 10; // only keep the last k successful states from each episode during gestation to encourage diversity in the initiation set (initation states = k * gestation_n)
        int refinement_eps        = 20; // eval-mode rollouts from SVM boundary states used to refine the binary classifier (policy is frozen, no TD3 updates)
        double nu                 = 0.1; // outlier fraction for initial one-class SVM classifier (nu * initiation_states is upper bound on number of outliers & lower bound on the fraction of training data that become support vectors, i.e., inside or on the boundary of the classifier)
        /* Claude explanation:
        It controls tightness of the classifier boundary around the gestation initiation states. The right value depends on the quality/diversity of gestation states and the desired conservativeness of the initiation set:

        Low nu (e.g. 0.05) → tight boundary, only the densest cluster of gestation states is enclosed. Fewer states qualify as "in the initiation set" — conservative but risks excluding valid entry states
        High nu (e.g. 0.3) → loose boundary, more of the space around the gestation states is included. More permissive but risks including states the policy can't actually succeed from
        
        The value matters most for Phase 3: 
        the support vectors that _sampleSupportVector() spawns from at skill.h:353 are directly determined by where nu draws the boundary. 
        Too tight → refinement episodes all start very close to the gestation cluster. 
        Too loose → some refinement spawn points may be in physically unreachable states.
        */
        int max_skills            = 6; // maximum number of skills to chain before giving up (to prevent infinite loops in edge cases where start is not covered)
        float start_noise_radius  = 2.0f; // std-dev (metres) of Gaussian noise added to parent's gestation positions when spawning for non-terminal skill episodes

        // tune these - just chose simple architectures
        std::vector<int> actor_layers  = {128, 256, 128};
        std::vector<int> critic_layers = {128, 256, 128};
        std::vector<int> poo_layers    = {128, 256, 128};

        // tune these learning hyperparameters as well
        float lr_actor        = 1e-4f;
        float lr_critic       = 3e-4f;
        float lr_poo          = 1e-4f;
        float tau             = 0.005f; // for soft updates of target networks
        float gamma           = 0.99f; // discount factor
        int   batch_size      = 256; 
        int   actor_update_freq = 4; // how often to update the actor network (compared to critic) during learning. 
    };

    DeepSkillChaining(
        std::shared_ptr<TrainEnvironment> env,
        torch::Device device,
        std::array<float, 3> global_goal,
        Config cfg = {})
        : _env(env)
        , _device(device)
        , _global_goal(global_goal)
        , _cfg(std::move(cfg))
        , _poo(env, _cfg.poo_layers, device, _cfg.lr_poo, _cfg.tau, _cfg.gamma, _cfg.batch_size)
    {}

    // Train the full DSC chain backward from the global goal.
    // Returns the number of skills trained.
    int train()
    {
        // Fix the global goal once for the entire training run.
        _env->setGoal(_global_goal);

        // === Skill 0 (oG): terminal skill ===
        std::cout << "\n=== Training terminal skill (oG) ===" << std::endl;
        _skills.emplace_back(_makeSkill());
        _skills[0].learn(
            _cfg.steps_per_episode, _cfg.gestation_n, _cfg.last_k,
            _cfg.refinement_eps, _cfg.nu,
            /*next_skill=*/nullptr, _cfg.start_noise_radius);
        _env->setGoal(_global_goal); // restore after learn() may have changed it
        _poo.addOption(0.0f);
        _poo.hardCopy();

        if (_startCovered())
        {
            std::cout << "Start state covered after terminal skill. Done." << std::endl;
            return 1;
        }

        // === Skills 1..max_skills-1: chain backward ===
        for (int i = 1; i < _cfg.max_skills; ++i)
        {
            const Skill *next = &_skills[i - 1];
            std::cout << "\n=== Training skill " << i
                      << " (terminates at skill " << i - 1 << "'s initiation set) ===" << std::endl;

            _skills.emplace_back(_makeSkill());
            _skills[i].learn(
                _cfg.steps_per_episode, _cfg.gestation_n, _cfg.last_k,
                _cfg.refinement_eps, _cfg.nu,
                next, _cfg.start_noise_radius);
            _env->setGoal(_global_goal); // restore after per-episode subgoal overrides
            _poo.addOption(0.0f);

            if (_startCovered())
            {
                std::cout << "Start state covered by skill " << i
                          << ". DSC chain complete." << std::endl;
                return i + 1;
            }
        }

        std::cout << "Max skills (" << _cfg.max_skills
                  << ") reached without covering start." << std::endl;
        return (int)_skills.size();
    }

    // Execute one episode using the policy over options (πO).
    // Returns the cumulative reward for the episode.
    float execute()
    {
        torch::Tensor state = _env->reset();
        float total_reward  = 0.0f;
        bool  episode_done  = false;

        while (!episode_done)
        {
            int    opt_idx    = _poo.getOption(state);
            Skill &skill      = _skills[opt_idx];
            float  cum_reward = 0.0f;
            int    steps      = 0;
            torch::Tensor last_state = state;

            // execute_option: run until β or timeout
            for (int t = 0; t < _cfg.steps_per_episode; ++t)
            {
                auto [scaled_action, _] = skill.agent().getAction(state, /*eval=*/true);
                auto [next_state, reward, done_t] = _env->step(scaled_action);
                cum_reward += reward.item<float>();
                steps++;
                last_state = next_state;

                bool env_done = done_t.item<float>() > 0.5f;

                // β: terminal skill ends when env says done;
                //    non-terminal ends when the preceding skill can start.
                bool beta = (opt_idx == 0)
                    ? env_done
                    : _skills[opt_idx - 1].canStart(next_state);

                state = next_state;
                if (beta || env_done)
                {
                    episode_done = env_done;
                    break;
                }
            }

            total_reward += cum_reward;
            _poo.addExperience(
                state, opt_idx,
                torch::tensor({cum_reward}, torch::kFloat32),
                last_state,
                torch::tensor({episode_done ? 1.0f : 0.0f}, torch::kFloat32),
                steps);
            _poo.learn();
        }

        return total_reward;
    }

    // Save all skill models and classifiers to <dir>/skill_<i>_*.
    void save(const std::string &dir) const
    {
        std::filesystem::create_directories(dir);
        for (size_t i = 0; i < _skills.size(); ++i)
        {
            std::string prefix = dir + "/skill_" + std::to_string(i);
            _skills[i].save(
                prefix + "_actor.pt",
                prefix + "_critic1.pt",
                prefix + "_critic2.pt",
                prefix + "_classifier.svm");
        }
        std::cout << "Saved " << _skills.size() << " skills to " << dir << std::endl;
    }

    // Load num_skills previously saved skills from <dir>.
    void load(const std::string &dir, int num_skills)
    {
        _skills.clear();
        for (int i = 0; i < num_skills; ++i)
        {
            _skills.emplace_back(_makeSkill());
            std::string prefix = dir + "/skill_" + std::to_string(i);
            _skills[i].load(
                prefix + "_actor.pt",
                prefix + "_critic1.pt",
                prefix + "_critic2.pt",
                prefix + "_classifier.svm");
            _poo.addOption(0.0f);
        }
        _poo.hardCopy();
        std::cout << "Loaded " << num_skills << " skills from " << dir << std::endl;
    }

    int numSkills() const { return (int)_skills.size(); }

private:
    std::shared_ptr<TrainEnvironment> _env;
    torch::Device                     _device;
    std::array<float, 3>              _global_goal;
    Config                            _cfg; 
    std::deque<Skill>                 _skills; // deque so that references to existing skills remain valid after emplace_back
    PolicyOverOptionsAgent            _poo; // single policy over options agent that learns to select among the growing skill set

    Skill _makeSkill()
    {
        return Skill(_env, _cfg.actor_layers, _cfg.critic_layers, _device,
                     _cfg.lr_actor, _cfg.lr_critic, _cfg.tau, _cfg.gamma,
                     _cfg.batch_size, _cfg.actor_update_freq);
    }

    // Returns true if the outermost (most distal) trained skill can start
    // from a freshly-sampled start state — meaning the chain covers s0.
    bool _startCovered()
    {
        torch::Tensor state = _env->reset();
        return _skills.back().canStart(state);
    }
};
