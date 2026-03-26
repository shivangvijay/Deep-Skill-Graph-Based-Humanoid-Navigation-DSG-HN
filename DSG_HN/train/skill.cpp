#include "skill.h"


Skill::Skill(
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

void Skill::learn(
    int steps_per_episode,
    int train_steps_per_epoch,
    int gestation_n,
    int last_k,
    int refinement_eps,
    double nu,
    const Skill *next_skill,
    float start_noise_radius)
{
    std::cout << "=== Phase 1: Gestation ===" << std::endl;
    _gestation_records = _gestation(steps_per_episode, train_steps_per_epoch, gestation_n, last_k, next_skill, start_noise_radius);

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

bool Skill::canStart(const torch::Tensor &state) const
{
    if (_always_available) return true;
    auto [pos, quat] = _env->getRobotPose();
    return _classifier.classify(_classifierVec(state, pos));
}

void Skill::setAlwaysAvailable()
{
    _always_available = true;
}

void Skill::initFromSkill(const Skill &other)
{
    auto copy_params = [](auto &dst, const auto &src) {
        torch::NoGradGuard no_grad;
        auto dp = dst->parameters();
        auto sp = src->parameters();
        for (size_t i = 0; i < dp.size(); ++i)
            dp[i].copy_(sp[i]);
    };
    copy_params(_agent.actor_local,    other._agent.actor_local);
    copy_params(_agent.critic_local_1, other._agent.critic_local_1);
    copy_params(_agent.critic_local_2, other._agent.critic_local_2);
    _agent.hardCopy();
}

AbstractedState Skill::sampleSubgoalState() const
{
    std::uniform_int_distribution<size_t> dist(0, _gestation_records.size() - 1);
    return _gestation_records[dist(_rng)].state;
}

std::array<float, 3> Skill::sampleStartPosition(float noise_radius) const
{
    std::uniform_int_distribution<size_t> dist(0, _gestation_records.size() - 1);
    std::normal_distribution<float> gauss(0.0f, noise_radius);
    auto pos = _gestation_records[dist(_rng)].state.position;
    pos[0] += gauss(_rng);
    pos[1] += gauss(_rng);
    return pos;
}

void Skill::save(const std::string &actor_path,
                 const std::string &critic1_path,
                 const std::string &critic2_path,
                 const std::string &classifier_path) const
{
    torch::save(_agent.actor_local,    actor_path);
    torch::save(_agent.critic_local_1, critic1_path);
    torch::save(_agent.critic_local_2, critic2_path);
    _classifier.save(classifier_path);
}

void Skill::load(const std::string &actor_path,
                 const std::string &critic1_path,
                 const std::string &critic2_path,
                 const std::string &classifier_path)
{
    torch::load(_agent.actor_local,    actor_path);
    torch::load(_agent.critic_local_1, critic1_path);
    torch::load(_agent.critic_local_2, critic2_path);
    _classifier.load(classifier_path);
}

TD3Agent &Skill::agent()
{
    return _agent;
}

/*** Private ***/

std::vector<float> Skill::_classifierVec(const torch::Tensor &full_state,
                                          const std::array<float, 3> &abs_pos) const
{
    auto flat = full_state.flatten().contiguous();
    const float *d = flat.data_ptr<float>();
    std::vector<float> out;
    out.reserve(13);
    out.push_back(abs_pos[0]);
    out.push_back(abs_pos[1]);
    out.push_back(abs_pos[2]);
    for (int i = 73; i < 76; ++i) out.push_back(d[i]); // velocity
    for (int i = 79; i < 83; ++i) out.push_back(d[i]); // orientation
    for (int i = 83; i < 86; ++i) out.push_back(d[i]); // angular_velocity
    return out;
}

std::pair<bool, bool> Skill::_checkTermination(const torch::Tensor &next_state,
                                                 const torch::Tensor &reward,
                                                 const torch::Tensor &done,
                                                 const Skill *next_skill) const
{
    bool in_next_set  = (next_skill != nullptr) && next_skill->canStart(next_state);
    bool env_done     = done.data_ptr<float>()[0] > 0.5f;
    float r           = reward.data_ptr<float>()[0];

    // For non-terminal skills, suppress env proximity termination (r > 0 + done).
    // Only canStart(), collision (r <= 0 + done), or timeout end the episode.
    bool hard_done     = env_done && (r <= 0.0f);
    bool terminal_goal = (next_skill == nullptr) && env_done;

    bool should_terminate = in_next_set || hard_done || terminal_goal;
    bool success          = in_next_set || (terminal_goal && r > 0.0f);
    return {should_terminate, success};
}

std::vector<GestationRecord> Skill::_gestation(
    int steps_per_episode, int train_steps_per_epoch, int gestation_n, int last_k,
    const Skill *next_skill, float start_noise_radius)
{
    // Spawn near the target region and set the subgoal for the episode.
    auto spawn = [&]() -> torch::Tensor {
        if (next_skill != nullptr) // non-terminal skill: spawn near next_skill's initiation set and set subgoal to next_skill's subgoal
        {
            auto sg = next_skill->sampleSubgoalState();
            _env->setGoal(sg.position, sg.orientation, sg.velocity, sg.angular_velocity);
        }
        std::array<float, 3> target_pos = (next_skill != nullptr)
            ? next_skill->sampleStartPosition(start_noise_radius) // non-terminal skill: set subgoal to next_skill's initiation set + noise
            : _env->getGoalPosition(); // terminal skill: set goal to global goal
        std::normal_distribution<float> gauss(0.0f, start_noise_radius);
        std::array<float, 3> start_pos = {
            target_pos[0] + gauss(_rng),
            target_pos[1] + gauss(_rng),
            target_pos[2]
        }; // start position with Gaussian noise around target_pos
        return _env->resetTo(start_pos, {1.0f, 0.0f, 0.0f, 0.0f}); // reset with start_pos and fixed orientation
    };

    int epoch = 0;
    while (true)
    {
        // === Training phase: train_steps_per_epoch steps with exploration ===
        torch::Tensor state = spawn();
        float total_reward   = 0.0f;
        int train_episodes   = 0;
        int train_successes  = 0;

        for (int step = 0; step < train_steps_per_epoch; ++step)
        {
            auto [scaled_action, action] = _agent.getAction(state);
            auto [next_state, reward, done] = _env->step(scaled_action);
            total_reward += reward.item<float>();
            _agent.addExperience(state, action, reward, next_state, done);
            _agent.learn();

            if (done.data_ptr<float>()[0] > 0.5f)
            {
                train_episodes++;
                auto [_, suc] = _checkTermination(next_state, reward, done, next_skill);
                if (suc) train_successes++;
                state = spawn();
            }
            else
                state = next_state;
        }

        std::cout << "  Epoch " << epoch + 1
                  << " | avg_reward=" << total_reward / train_steps_per_epoch
                  << " | train_success=" << train_successes << "/" << train_episodes << std::endl;

        // === Validation phase: 2*gestation_n eval episodes, policy frozen ===
        int val_successes = 0;
        std::vector<GestationRecord> epoch_records;

        // for now, validate for only 2*gestation_n episodes, but we should probably validate for gestation_n episodes and wait for all successes
        for (int trial = 0; trial < 2 * gestation_n; ++trial)
        {
            state = spawn();
            std::deque<GestationRecord> window;
            bool success = false;

            for (int step = 0; step < steps_per_episode; ++step)
            {
                auto [scaled_action, _] = _agent.getAction(state, /*eval=*/true);
                auto [next_state, reward, done] = _env->step(scaled_action);

                auto [pos, quat] = _env->getRobotPose();
                auto cv = _classifierVec(state, pos);
                AbstractedState rec_state = {pos, quat, {cv[3], cv[4], cv[5]}, {cv[10], cv[11], cv[12]}};
                window.push_back({cv, rec_state});
                if ((int)window.size() > last_k)
                    window.pop_front();

                auto [terminate, suc] = _checkTermination(next_state, reward, done, next_skill);
                if (terminate) { success = suc; break; }
                state = next_state;
            }

            if (success)
            {
                val_successes++;
                for (auto &r : window)
                    epoch_records.push_back(std::move(r));
            }
        }

        std::cout << "  Validation: " << val_successes << "/" << (2 * gestation_n) << " successes";
        if (val_successes >= gestation_n)
        {
            std::cout << " — gestation complete." << std::endl;
            return epoch_records;
        }
        std::cout << " — training more." << std::endl;
        epoch++;
    }
}

// train initial one-class SVM with gestation_n trajectories' last k states as positives
void Skill::_learnInitialClassifier(const std::vector<GestationRecord> &records, double nu)
{
    std::vector<std::vector<float>> states;
    states.reserve(records.size());
    for (const auto &r : records)
        states.push_back(r.classifier_vec);
    _classifier.trainOneClass(states, nu); 
}

// iteratively collects rollouts from the current SVM decision boundary and retrains a binary classifier until convergence or max refinement_eps episodes
void Skill::_refineClassifier(const std::vector<GestationRecord> &records,
                               int refinement_eps, int steps_per_episode,
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

// Sample a support vector from the current SVM as a starting point for refinement rollouts. If no support vectors, sample gestation state closest to the decision boundary.
AbstractedState Skill::_sampleSupportVector(const std::vector<GestationRecord> &records) const
{
    auto svs = _classifier.getSupportVectors(13);
    if (!svs.empty())
    {
        std::uniform_int_distribution<size_t> dist(0, svs.size() - 1);
        const auto &sv = svs[dist(_rng)];
        return {
            std::array<float, 3>{sv[0], sv[1], sv[2]},
            std::array<float, 4>{sv[6], sv[7], sv[8], sv[9]},
            std::array<float, 3>{sv[3], sv[4], sv[5]},
            std::array<float, 3>{sv[10], sv[11], sv[12]}
        };
    }

    // Fallback: gestation record closest to the SVM decision boundary
    size_t best_idx = 0;
    double best_dv  = std::numeric_limits<double>::max();
    for (size_t i = 0; i < records.size(); ++i)
    {
        double dv = std::abs(_classifier.decisionValue(records[i].classifier_vec));
        if (dv < best_dv) { best_dv = dv; best_idx = i; }
    }
    return records[best_idx].state;
}
