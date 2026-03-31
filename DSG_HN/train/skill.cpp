#include "skill.h"
#include <fstream>

Skill::Skill(
    int id,
    std::shared_ptr<TrainEnvironment> env,
    const std::vector<int> &actor_layer_sizes,
    const std::vector<int> &critic_layer_sizes,
    torch::Device device,
    float lr_actor, float lr_critic,
    float tau, float gamma, int max_obstacles, int actor_warmup_steps,
    int batch_size, int actor_update_freq, int k, int max_steps, double nu,
    std::shared_ptr<Skill> parent, int gestation_period, bool is_global, AbstractedState global_goal, std::shared_ptr<Skill> global_option)
    : _id(id), _env(env), _parent(parent), _is_global(is_global), _gestation_period(gestation_period), _k(k), _max_steps(max_steps), _agent(env, actor_layer_sizes, critic_layer_sizes, device,
                                                                                                                                            lr_actor, lr_critic, tau, gamma, batch_size, actor_update_freq, max_obstacles, actor_warmup_steps),
      _rng(std::random_device{}()), _global_goal(global_goal), _nu(nu), _global_option(global_option)
{
}

// TODO: perhaps we need something where instead of just tracking goal hits, we only mark it as ready
// when it meets a certain success percentage
std::string Skill::getTrainingPhase() const
{
    if (_is_global)
        return "global";
    if (_goal_hits < _gestation_period || !_classifier.trained())
        return "gestation";
    if (!_validated)
        return "validation";
    return "mature";
}

AbstractedState Skill::getLocalGoal()
{
    if (_parent == nullptr || _is_global)
    {
        return _global_goal;
    }
    else
    {
        return _parent->sampleSubgoalState(false);
    }
}

bool Skill::atTermination(const AbstractedState& goal) const
{
    auto [reward, done] = _env->computeReward(goal);
    if (!_parent)
    {
        // if the goal option, return if we ended up at the goal
        return (done.data_ptr<float>()[0] > 0.5f && reward.data_ptr<float>()[0] > 45);
    }
    else // return true if the current state is within the parents initiation set
    {
        // TODO: collision check - but maybe that should be done when sampling a subgoal. can add it here for redundancy
        // Use pessimistic boundary for termination — tighter condition for termination
        // which calls parent.pessimistic_is_init_true()
        if (reward.data_ptr<float>()[0] > 45&& !_parent->canStartPessimistic(_env->getAbstractedState()))
        {
            std::cout << "Reached goal but not termination condition" << std::endl;
        }
        return _parent->canStartPessimistic(_env->getAbstractedState()) && reward.data_ptr<float>()[0] > -5;
    }
}

// TODO: Add HER for updating TD3 agent
std::tuple<int, float, bool, torch::Tensor, torch::Tensor> Skill::rollout(const AbstractedState &goal)
{
    // need to get start and end states w.r.t the global goal, which is what is expected as the input to the policy over options
    torch::Tensor start_state_poo = _env->getStateRelativeToGoal(_global_goal);

    _env->setGoal(goal);
    torch::Tensor state = _env->getState();
    RobotState underlying_state = _env->getUnderlyingState().first;

    int num_steps = 0;
    float total_reward = 0;
    bool should_terminate = false;

    bool train = getTrainingPhase() == "gestation" || _is_global;

    std::vector<GestationRecord> visited;
    if (!_is_global)
    {
        visited.push_back({_classifierVec(_env->getAbstractedState()), _env->getAbstractedState()});
    }

    std::vector<Transition> her_transitions;

    while (num_steps < _max_steps && !_atLocalGoal(goal))
    {
        auto [scaled_action, action] = _agent.getAction(state, !train);
        auto [next_state, reward, done] = _env->step(scaled_action);

        auto [next_underlying_state, collision] = _env->getUnderlyingState();
        if (train) // if training instability, perhaps look at putting this outside the while loop like they have it elsewhere
        {
            her_transitions.push_back({underlying_state, action, next_underlying_state, collision});
            _agent.addExperience(state, action, reward, next_state, done);
            _agent.learn();
        }
        if (!_is_global) // replicating global agent logic
        {
            auto& global_agent = _global_option->agent();
            global_agent.addExperience(state, action, reward, next_state, done);
            global_agent.learn();
        }

        state = next_state;
        if (!_is_global)
        {
            visited.push_back({_classifierVec(_env->getAbstractedState()), _env->getAbstractedState()});
        }
        num_steps++;
        total_reward += _env->computeReward(_global_goal).first.data_ptr<float>()[0]; // for policy over options, compute reward w.r.t global goal
    }

    if (train)
    {
        _herUpdate(her_transitions);
    }

    if (!_is_global && atTermination(goal)) // can not reach goal, but still reach next option
    {
        _goal_hits++;
        if (train)
            std::cout << "\rOption: " << _id << " | Success: " << _goal_hits << "/" << _gestation_period << std::flush;
    }

    if (!_is_global && train)
    {
        _fitClassifier(visited, atTermination(goal));
    }

    torch::Tensor end_state_poo = _env->getStateRelativeToGoal(_global_goal);

    return {num_steps, total_reward, _atLocalGoal(goal), start_state_poo, end_state_poo};
}

void Skill::validateSkill(bool success)
{
    if (_validated) return;

    if (success)
    {
        _validated = true;
    }
    else
    {
        _validated = false;
        _goal_hits = 0; // zero out goal hits
    }
}

bool Skill::canStart(const RobotState &state) const
{
    if (getTrainingPhase() == "gestation")
        return true;
    auto vec = _classifierVec(state);
    return _classifier.classify(vec) || _pessimistic_classifier.classify(vec);
}

bool Skill::canStart(const AbstractedState &state) const
{
    if (getTrainingPhase() == "gestation")
        return true;
    auto vec = _classifierVec(state);
    return _classifier.classify(vec) || _pessimistic_classifier.classify(vec);
}

bool Skill::canStartPessimistic(const RobotState &state) const
{
    if (getTrainingPhase() == "gestation")
        return true;
    return _pessimistic_classifier.classify(_classifierVec(state));
}

bool Skill::canStartPessimistic(const AbstractedState &state) const
{
    if (getTrainingPhase() == "gestation")
        return true;
    return _pessimistic_classifier.classify(_classifierVec(state));
}

void Skill::initFromSkill(std::shared_ptr<Skill> other)
{
    auto &other_agent = other->agent();
    auto copy_params = [](auto &dst, const auto &src)
    {
        torch::NoGradGuard no_grad;
        auto dp = dst->parameters();
        auto sp = src->parameters();
        for (size_t i = 0; i < dp.size(); ++i)
            dp[i].copy_(sp[i]);
    };
    copy_params(_agent.actor_local, other_agent.actor_local);
    copy_params(_agent.critic_local_1, other_agent.critic_local_1);
    copy_params(_agent.critic_local_2, other_agent.critic_local_2);
    _agent.hardCopy();
}

// TODO: maybe want to improve the sampling logic for greater training efficiency
AbstractedState Skill::sampleSubgoalState(bool uniform) const
{
    // Prefer states inside the pessimistic classifier (confident region) as subgoals.
    // Falls back to any positive record if pessimistic classifier has no confirmed samples.
    if (_pessimistic_classifier.trained() && !uniform)
    {
        std::vector<size_t> pessimistic_indices;
        for (size_t i = 0; i < _positive_gestation_records.size(); i++)
            if (_pessimistic_classifier.classify(_positive_gestation_records[i].classifier_vec))
                pessimistic_indices.push_back(i);

        if (!pessimistic_indices.empty())
        {
            std::uniform_int_distribution<size_t> dist(0, pessimistic_indices.size() - 1);
            AbstractedState subgoal = _positive_gestation_records[pessimistic_indices[dist(_rng)]].state;
            if (!kUseVelocityInClassifier)
            {
                subgoal.velocity = {0.0f, 0.0f, 0.0f};
                subgoal.angular_velocity = {0.0f, 0.0f, 0.0f};
            }
            return subgoal;
        }
    }
    // fallback: sample uniformly from all positive records
    std::uniform_int_distribution<size_t> dist(0, _positive_gestation_records.size() - 1);
    AbstractedState subgoal = _positive_gestation_records[dist(_rng)].state;
    if (!kUseVelocityInClassifier)
    {
        subgoal.velocity = {0.0f, 0.0f, 0.0f};
        subgoal.angular_velocity = {0.0f, 0.0f, 0.0f};
    }
    return subgoal;
}

void Skill::save(const std::string &actor_path,
                 const std::string &critic1_path,
                 const std::string &critic2_path,
                 const std::string &classifier_path) const
{
    torch::save(_agent.actor_local, actor_path);
    torch::save(_agent.critic_local_1, critic1_path);
    torch::save(_agent.critic_local_2, critic2_path);
    _classifier.save(classifier_path);
    if (_pessimistic_classifier.trained())
        _pessimistic_classifier.save(classifier_path + "_pessimistic");
}

void Skill::load(const std::string &actor_path,
                 const std::string &critic1_path,
                 const std::string &critic2_path,
                 const std::string &classifier_path)
{
    torch::load(_agent.actor_local, actor_path);
    torch::load(_agent.critic_local_1, critic1_path);
    torch::load(_agent.critic_local_2, critic2_path);
    _classifier.load(classifier_path);
    // load pessimistic only if file exists (graceful for models saved before this change)
    std::ifstream f(classifier_path + "_pessimistic");
    if (f.good())
        _pessimistic_classifier.load(classifier_path + "_pessimistic");
}

TD3Agent &Skill::agent()
{
    return _agent;
}

/*** Private ***/

void Skill::_herUpdate(const std::vector<Transition>& trajectory)
{
    if (trajectory.size() == 0 || trajectory.back().in_collision == true)
        return;

    AbstractedState augmented_goal;
    augmented_goal.position = trajectory.back().state.position;
    augmented_goal.orientation = trajectory.back().state.orientation;
    augmented_goal.velocity = trajectory.back().state.velocity;
    augmented_goal.angular_velocity = trajectory.back().state.angular_velocity;

    for (const auto& t: trajectory)
    {
        auto augmented_state = _env->transformState(t.state, augmented_goal);
        auto augmented_next_state = _env->transformState(t.next_state, augmented_goal);

        auto [augmented_reward, augmented_done] = _env->computeReward(t.state, t.in_collision, augmented_goal);
        _agent.addExperience(augmented_state, t.action, augmented_reward, augmented_next_state, augmented_done);
        _agent.learn();

        if (!_is_global)
        {
            auto &global_agent = _global_option->agent();
            global_agent.addExperience(augmented_state, t.action, augmented_reward, augmented_next_state, augmented_done);
            global_agent.learn();
        }
    }

}


// TODO: add max buffer size at which we start to pop off the front
// TODO: generally, this logic will need to really be refined to ensure that it is not too pessimistic/optimistic
void Skill::_fitClassifier(const std::vector<GestationRecord> &visited, bool term_success)
{

    if (term_success)
    {
        // if successful, add the last k states to the positive set.
        for (int t = visited.size() - 1; t >= std::max(0, (int)visited.size() - _k); t--)
        {
            _positive_gestation_records.push_back(visited[t]);
            _gestation_vecs.push_back(visited[t].classifier_vec);
            _gestation_labels.push_back(1);
        }
    }
    else
    {
        // try and make SVM a bit more optimistic by not putting the entire traj in the negative catagory
        _gestation_vecs.push_back(visited.front().classifier_vec);
        _gestation_labels.push_back(-1);
        _has_negative_gestation = true;
    }

    // always attempt to fit after every rollout
    if (_positive_gestation_records.empty())
        return;

    if (!_has_negative_gestation)
    {
        // only positive data available — use one-class SVMs on positive vecs only.
        std::vector<std::vector<float>> pos_vecs;
        for (size_t i = 0; i < _gestation_vecs.size(); i++)
            if (_gestation_labels[i] == 1)
                pos_vecs.push_back(_gestation_vecs[i]);

        if (!pos_vecs.empty())
        {
            bool first_phase1 = !_classifier.trained();
            _pessimistic_classifier.trainOneClass(pos_vecs, _nu);       // tight
            _classifier.trainOneClass(pos_vecs, _nu / 10.0);            // loose / optimistic
            if (first_phase1)
                std::cout << "\n[Skill " << _id << "] Classifier Phase 1: OneClass init. Pos=" << pos_vecs.size() << "\n";
        }
    }
    else
    {
        // binary SVC as optimistic, then one-class re-fit on SVC-positive predictions as pessimistic.
        int neg_count = std::count(_gestation_labels.begin(), _gestation_labels.end(), -1);
        _classifier.train(_gestation_vecs, _gestation_labels,
                          /*C=*/1.0, /*gamma=*/-1.0, /*balance_classes=*/true);

        // Re-fit pessimistic on only the states the optimistic SVC predicts as positive
        std::vector<std::vector<float>> svc_positive_vecs;
        for (const auto &vec : _gestation_vecs)
            if (_classifier.classify(vec))
                svc_positive_vecs.push_back(vec);

        if (!svc_positive_vecs.empty())
            _pessimistic_classifier.trainOneClass(svc_positive_vecs, _nu);

        if (neg_count == 1) // first failure — log Phase 1→2 transition
        {
            int pos_count = std::count(_gestation_labels.begin(), _gestation_labels.end(), 1);
            std::cout << "\n[Skill " << _id << "] Classifier Phase 1→2: binary SVC. Pos=" << pos_count << " Neg=1\n";
        }
    }
}

bool Skill::_atLocalGoal(const AbstractedState& goal) const
{
    auto [reward, done] = _env->computeReward(goal);
    bool env_done = done.data_ptr<float>()[0] > 0.5f;
    float r = reward.data_ptr<float>()[0];

    if (_is_global || !_parent)
    {
        // global option: success = reached the actual goal
        bool success = env_done && (r > 45);
        bool hard_done = env_done && (r < 45);
        return success || hard_done;
    }

    // non-global: exit as soon as we enter the parent's pessimistic init set —
    // matches Python's is_at_local_goal which uses is_term_true (pessimistic).
    // This keeps the robot inside the region when atTermination() is called.
    bool in_pessimistic_set = _parent->canStartPessimistic(_env->getAbstractedState());
    bool hard_done = env_done && (r < 45);
    return in_pessimistic_set || hard_done;
}

std::vector<float> Skill::_classifierVec(const AbstractedState &state) const
{
    std::vector<float> out;
    auto scaling_factors = _env->env_scaling_factors;
    out.reserve(13);
    // global pos
    out.push_back(state.position[0] / scaling_factors.position[0]);
    out.push_back(state.position[1] / scaling_factors.position[1]);
    out.push_back(state.position[2] / scaling_factors.position[2]);
    // global vel — zeroed when kUseVelocityInClassifier is false (environment spawns with zero velocity)
    out.push_back(kUseVelocityInClassifier ? state.velocity[0] / scaling_factors.velocity[0] : 0.0f);
    out.push_back(kUseVelocityInClassifier ? state.velocity[1] / scaling_factors.velocity[1] : 0.0f);
    out.push_back(kUseVelocityInClassifier ? state.velocity[2] / scaling_factors.velocity[2] : 0.0f);
    // orientation
    if (state.orientation[0] < 0)
    {
        out.push_back(-state.orientation[0] / scaling_factors.orientation[0]);
        out.push_back(-state.orientation[1] / scaling_factors.orientation[1]);
        out.push_back(-state.orientation[2] / scaling_factors.orientation[2]);
        out.push_back(-state.orientation[3] / scaling_factors.orientation[3]);
    }
    else
    {
        out.push_back(state.orientation[0] / scaling_factors.orientation[0]);
        out.push_back(state.orientation[1] / scaling_factors.orientation[1]);
        out.push_back(state.orientation[2] / scaling_factors.orientation[2]);
        out.push_back(state.orientation[3] / scaling_factors.orientation[3]);
    }
    // ang vel — zeroed when kUseVelocityInClassifier is false
    out.push_back(kUseVelocityInClassifier ? state.angular_velocity[0] / scaling_factors.angular_velocity[0] : 0.0f);
    out.push_back(kUseVelocityInClassifier ? state.angular_velocity[1] / scaling_factors.angular_velocity[1] : 0.0f);
    out.push_back(kUseVelocityInClassifier ? state.angular_velocity[2] / scaling_factors.angular_velocity[2] : 0.0f);
    return out;
}

std::vector<float> Skill::_classifierVec(const RobotState &state) const
{
    std::vector<float> out;
    auto scaling_factors = _env->env_scaling_factors;
    out.reserve(13);
    // global pos
    out.push_back(state.position[0] / scaling_factors.position[0]);
    out.push_back(state.position[1] / scaling_factors.position[1]);
    out.push_back(state.position[2] / scaling_factors.position[2]);
    // global vel — zeroed when kUseVelocityInClassifier is false (environment spawns with zero velocity)
    out.push_back(kUseVelocityInClassifier ? state.velocity[0] / scaling_factors.velocity[0] : 0.0f);
    out.push_back(kUseVelocityInClassifier ? state.velocity[1] / scaling_factors.velocity[1] : 0.0f);
    out.push_back(kUseVelocityInClassifier ? state.velocity[2] / scaling_factors.velocity[2] : 0.0f);
    // orientation
    if (state.orientation[0] < 0)
    {
        out.push_back(-state.orientation[0] / scaling_factors.orientation[0]);
        out.push_back(-state.orientation[1] / scaling_factors.orientation[1]);
        out.push_back(-state.orientation[2] / scaling_factors.orientation[2]);
        out.push_back(-state.orientation[3] / scaling_factors.orientation[3]);
    }
    else
    {
        out.push_back(state.orientation[0] / scaling_factors.orientation[0]);
        out.push_back(state.orientation[1] / scaling_factors.orientation[1]);
        out.push_back(state.orientation[2] / scaling_factors.orientation[2]);
        out.push_back(state.orientation[3] / scaling_factors.orientation[3]);
    }
    // ang vel — zeroed when kUseVelocityInClassifier is false
    out.push_back(kUseVelocityInClassifier ? state.angular_velocity[0] / scaling_factors.angular_velocity[0] : 0.0f);
    out.push_back(kUseVelocityInClassifier ? state.angular_velocity[1] / scaling_factors.angular_velocity[1] : 0.0f);
    out.push_back(kUseVelocityInClassifier ? state.angular_velocity[2] / scaling_factors.angular_velocity[2] : 0.0f);
    return out;
}

float Skill::_euclideanDistance(const std::array<float, 3> &a, const std::array<float, 3> &b, bool sqrt) const
{
    float dist = 0;
    for (int i = 0; i < a.size(); i++)
    {
        dist += (a[i] - b[i]) * (a[i] - b[i]);
    }
    if (sqrt)
    {
        return std::sqrt(dist);
    }
    return dist;
}

float Skill::_euclideanDistance(const std::array<float, 4> &a, const std::array<float, 4> &b, bool sqrt) const
{
    float dist = 0;
    for (int i = 0; i < a.size(); i++)
    {
        dist += (a[i] - b[i]) * (a[i] - b[i]);
    }
    if (sqrt)
    {
        return std::sqrt(dist);
    }
    return dist;
}

// TODO: perhaps should normalzie this?
float Skill::distanceToState(const AbstractedState &state) const
{
    float max_dist = 0; // gonna just do euclidian distance between vectors

    for (const auto &start : _positive_gestation_records)
    {
        float dist = 0;
        // doing this euclid distance metric is not really right, but I am lazy so am just leaving it for now
        dist += _euclideanDistance(state.position, start.state.position, false);
        dist += _euclideanDistance(state.orientation, start.state.orientation, false);
        dist += _euclideanDistance(state.velocity, start.state.velocity, false);
        dist += _euclideanDistance(state.angular_velocity, start.state.angular_velocity, false);

        max_dist = std::max(dist, max_dist);
    }
    return max_dist;
}
