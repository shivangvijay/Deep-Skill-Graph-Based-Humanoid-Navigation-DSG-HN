#include "skill.h"

Skill::Skill(
    std::shared_ptr<TrainEnvironment> env,
    const std::vector<int> &actor_layer_sizes,
    const std::vector<int> &critic_layer_sizes,
    torch::Device device,
    float lr_actor, float lr_critic,
    float tau, float gamma, int max_obstacles, int actor_warmup_steps,
    int batch_size, int actor_update_freq, int k, int max_steps,
    std::shared_ptr<Skill> parent, int gestation_period, bool is_global, AbstractedState global_goal)
    : _env(env), _parent(parent), _is_global(is_global), _gestation_period(gestation_period), _k(k), _max_steps(max_steps), _agent(env, actor_layer_sizes, critic_layer_sizes, device,
                                                                                                                                   lr_actor, lr_critic, tau, gamma, batch_size, actor_update_freq, max_obstacles, actor_warmup_steps),
      _rng(std::random_device{}()), _global_goal(global_goal)
{
}

std::string Skill::getTrainingPhase() const
{
    if (_goal_hits < _gestation_period)
    {
        return "gestation";
    }
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
        return _parent->sampleSubgoalState();
    }
}

std::tuple<int, float, bool, torch::Tensor, torch::Tensor> Skill::rollout(const AbstractedState &goal)
{
    // need to get start and end states w.r.t the global goal, which is what is expected as the input to the policy over options
    _env->setGoal(goal);
    torch::Tensor start_state_poo = _env->getState();

    _env->setGoal(goal);
    torch::Tensor state = _env->getState();
    int num_steps = 0;
    float total_reward = 0;
    bool should_terminate = false;

    torch::Tensor reward;
    torch::Tensor done;

    auto initial_check = _env->computeReward();
    reward = initial_check.first;
    done = initial_check.second;

    bool train = getTrainingPhase() == "gestation" || _is_global;

    std::vector<GestationRecord> visited;
    if (!_is_global)
    {
        visited.push_back({_classifierVec(_env->getAbstractedState()), _env->getAbstractedState()});
    }

    while (num_steps < _max_steps && !_atLocalGoal(reward, done))
    {
        auto [scaled_action, action] = _agent.getAction(state, !train);
        auto [next_state, next_reward, next_done] = _env->step(scaled_action);

        reward = next_reward;
        done = next_done;

        if (train) // if training instability, perhaps look at putting this outside the while loop like they have it elsewhere
        {
            _agent.addExperience(state, action, reward, next_state, done);
            _agent.learn();
        }

        state = next_state;
        if (!_is_global)
        {
            visited.push_back({_classifierVec(_env->getAbstractedState()), _env->getAbstractedState()});
        }
        num_steps++;
        total_reward += reward.data_ptr<float>()[0];
    }

    if (!_is_global && _atTermination(reward, done)) // can not reach goal, but still reach next option
    {
        _goal_hits++;
    }

    if (!_is_global)
    {
        _fitClassifier(visited, _atTermination(reward, done));
    }

    _env->setGoal(goal);
    torch::Tensor end_state_poo = _env->getState();

    return {num_steps, total_reward, _atLocalGoal(reward, done), start_state_poo, end_state_poo};
}

void Skill::_fitClassifier(const std::vector<GestationRecord> &visited, bool term_success)
{
    std::vector<int> labels(visited.size(), -1);
    std::vector<std::vector<float>> class_vecs;

    if (term_success)
    {
        for (int t = visited.size() - 1; t >= std::max(0, (int)visited.size() - _k); t--)
        {
            labels[t] = 1;
            _gestation_records.push_back(visited[t]); // save this is a positive example which you can sample from
        }
    }

    for (const auto &v : visited)
    {
        class_vecs.push_back(v.classifier_vec);
    }

    _classifier.train(class_vecs, labels);
}

bool Skill::canStart(const RobotState &state) const
{
    if (getTrainingPhase() == "gestation")
        return true;
    return _classifier.classify(_classifierVec(state));
}

bool Skill::canStart(const AbstractedState &state) const
{
    if (getTrainingPhase() == "gestation")
        return true;
    return _classifier.classify(_classifierVec(state));
}

bool Skill::_atTermination(const torch::Tensor &reward, const torch::Tensor &done) const
{
    if (!_parent)
    {
        // if the goal option, return if we ended up at the goal
        return (done.data_ptr<float>()[0] > 0.5f && reward.data_ptr<float>()[0] > 45);
    }
    else // return true if the current state is within the parents initiation set
    {
        return _parent->canStart(_env->getAbstractedState());
    }
}

bool Skill::_atLocalGoal(const torch::Tensor &reward,
                         const torch::Tensor &done) const
{
    bool env_done = done.data_ptr<float>()[0] > 0.5f;
    bool in_next_set = env_done; // default case for when the parent is none or is in the global option
    if (!_is_global && _parent)
    {
        in_next_set = _parent->canStart(_env->getAbstractedState());
    }
    float r = reward.data_ptr<float>()[0];
    bool hard_done = env_done && (r < 45);
    bool success = in_next_set && (r > 45);

    return success || hard_done; // if we are succesful, but not in next set, should continue searching. Contrariliy, if the env has reached max steps, should just time out
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

AbstractedState Skill::sampleSubgoalState() const
{
    std::uniform_int_distribution<size_t> dist(0, _gestation_records.size() - 1);
    return _gestation_records[dist(_rng)].state;
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
}

TD3Agent &Skill::agent()
{
    return _agent;
}

/*** Private ***/

std::vector<float> Skill::_classifierVec(const AbstractedState &state) const
{
    std::vector<float> out;
    out.reserve(13);
    // global pos
    out.push_back(state.position[0]);
    out.push_back(state.position[1]);
    out.push_back(state.position[2]);
    // global vel
    out.push_back(state.velocity[0]);
    out.push_back(state.velocity[1]);
    out.push_back(state.velocity[2]);
    // orientation
    out.push_back(state.orientation[0]);
    out.push_back(state.orientation[1]);
    out.push_back(state.orientation[2]);
    out.push_back(state.orientation[3]);
    // ang vel
    out.push_back(state.angular_velocity[0]);
    out.push_back(state.angular_velocity[1]);
    out.push_back(state.angular_velocity[2]);
    return out;
}

std::vector<float> Skill::_classifierVec(const RobotState &state) const
{
    std::vector<float> out;
    out.reserve(13);
    // global pos
    out.push_back(state.position[0]);
    out.push_back(state.position[1]);
    out.push_back(state.position[2]);
    // global vel
    out.push_back(state.velocity[0]);
    out.push_back(state.velocity[1]);
    out.push_back(state.velocity[2]);
    // orientation
    out.push_back(state.orientation[0]);
    out.push_back(state.orientation[1]);
    out.push_back(state.orientation[2]);
    out.push_back(state.orientation[3]);
    // ang vel
    out.push_back(state.angular_velocity[0]);
    out.push_back(state.angular_velocity[1]);
    out.push_back(state.angular_velocity[2]);
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


float Skill::distanceToState(const AbstractedState &state) const
{
    float max_dist = 0; // gonna just do euclidian distance between vectors

    for (const auto &start : _gestation_records)
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
