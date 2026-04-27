#include "skill.h"
#include <fstream>
#include <random>
#include <numeric>

Skill::Skill(
    int id,
    std::shared_ptr<TrainEnvironment> env,
    const std::vector<int> &actor_layer_sizes,
    const std::vector<int> &critic_layer_sizes,
    torch::Device device,
    float lr_actor, float lr_critic,
    float tau, float gamma, int actor_warmup_steps,
    int batch_size, int actor_update_freq, int k, int max_steps, double nu,
    std::shared_ptr<Skill> parent, int gestation_period, bool is_global, AbstractedState global_goal, std::shared_ptr<Skill> global_option, bool eval)
    : _id(id), _env(env), _parent(parent), _is_global(is_global), _gestation_period(gestation_period), _k(k), _max_steps(max_steps), _agent(env, actor_layer_sizes, critic_layer_sizes, device,
                                                                                                                                            lr_actor, lr_critic, tau, gamma, batch_size, actor_update_freq, actor_warmup_steps),
      _rng(std::random_device{}()), _global_goal(global_goal), _nu(nu), _global_option(global_option), _gamma(gamma), _eval(eval), _lr_actor(lr_actor), _lr_critic(lr_critic)
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
    // The global option has no meaningful initiation region to sample from.
    // The first skill in a DSC chain (parent = global option) should target
    // its own _global_goal (the goal it was trained to reach), not a random
    // state from the global option's empty gestation records.
    if (_parent == nullptr || _is_global || _parent->getTrainingPhase() == "global")
        return _global_goal;
    return _parent->sampleSubgoalState();
}

bool Skill::atTermination(const AbstractedState &goal) const
{
    if (!_parent)
    {
        auto [reward, done] = _env->computeReward(goal);
        return (done.data_ptr<float>()[0] > 0.5f && reward.data_ptr<float>()[0] > 45);
    }

    auto state = _env->getAbstractedState();
    bool collision = _env->getUnderlyingState().second;

    // The key change: Relax the decision value threshold from -0.001 to -0.1
    // to allow for a smoother handover between skills.
    bool pessimistic = _parent->canStartPessimistic(state);
    bool near_boundary = _parent->canStart(state) && _parent->getDecisionValue(state) > -0.01;

    return (pessimistic || near_boundary) && !collision;
}

// TODO: clear training buffer after you move to mature phase
std::tuple<int, float, bool, torch::Tensor, torch::Tensor> Skill::rollout(const AbstractedState &goal)
{
    // need to get start and end states w.r.t the global goal, which is what is expected as the input to the policy over options
    torch::Tensor start_state_poo = _env->getStateRelativeToGoal(_global_goal);

    _env->setGoal(goal);
    torch::Tensor state = _env->getState();
    RobotState underlying_state = _env->getUnderlyingState().first;

    int num_steps = 0;
    float total_reward = 0.0f;
    float current_gamma = 1.0f;
    bool should_terminate = false;

    bool train = !_eval && !(getTrainingPhase() == "validation" && !_is_global); // (getTrainingPhase() != "validation") || _is_global; //getTrainingPhase() == "gestation" || _is_global;
    if (_is_global || (getTrainingPhase() != "gestation"))
    {
        _agent.setExplorationNoise(0.1f);
    }
    else
    {
        _agent.setExplorationNoise(0.3f);
    }

    std::vector<GestationRecord> visited;
    if (!_is_global)
    {
        visited.push_back({_classifierVec(_env->getAbstractedState()), _env->getAbstractedState()});
    }

    std::vector<Transition> her_transitions;

    int max_steps = (_is_global) ? 1 : _max_steps;
    while (num_steps < max_steps && !_atLocalGoal(goal))
    {
        auto [scaled_action, action] = _agent.getAction(state, !train);
        auto [next_state, reward, done] = _env->step(scaled_action);

        auto [next_underlying_state, collision] = _env->getUnderlyingState();

        // if (!_is_global && _parent)
        // {
        //     // Recompute reward without the 0.5m goal radius — termination is
        //     // determined by the parent's initiation set, not proximity to subgoal.
        //     std::tie(reward, done) = _env->computeReward(next_underlying_state, collision, goal, false);

        //     if (_parent->canStartPessimistic(_env->getAbstractedState()) && !collision)
        //     {
        //         reward = torch::tensor({50.0f}, torch::kFloat32);
        //         done = torch::tensor({1.0f}, torch::kFloat32);
        //     }
        // }
        if (train) // if training instability, perhaps look at putting this outside the while loop like they have it elsewhere
        {
            // her_transitions.push_back({underlying_state, action, next_underlying_state, collision});
            _agent.addExperience(state, action, reward, next_state, done);
            _agent.learn();
        }
        if (!_is_global && train) // replicating global agent logic
        {
            auto &global_agent = _global_option->agent();
            global_agent.addExperience(state, action, reward, next_state, done);
            global_agent.learn();
        }

        state = next_state;
        if (!_is_global && train)
        {
            visited.push_back({_classifierVec(_env->getAbstractedState()), _env->getAbstractedState()});
        }
        num_steps++;
        total_reward += current_gamma * _env->computeReward(_global_goal).first.data_ptr<float>()[0];
        current_gamma *= _gamma;
    }

    // if (train)
    // {
    //     _herUpdate(her_transitions);
    // }

    if (!_is_global && atTermination(goal) && num_steps > 0) // can not reach goal, but still reach next option
    {
        _goal_hits++;
        if (train)
            std::cout << "Option: " << _id << " | Success: " << _goal_hits << "/" << _gestation_period << std::endl;
    }

    if (!_is_global && train) // && train)
    {
        bool term = atTermination(goal);
        bool valid = !term || isValidInitData(visited); // always accept failures; gate successes by sibling overlap
        _fitClassifier(visited, term && valid);
    }

    torch::Tensor end_state_poo = _env->getStateRelativeToGoal(_global_goal);

    return {num_steps, total_reward, atTermination(goal), start_state_poo, end_state_poo};
}

void Skill::validateSkill(bool success)
{
    if (_validated)
        return;

    if (success)
    {
        // _agent.setLearningRates(_lr_actor / 5, _lr_critic / 5);
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
    if (_is_global || getTrainingPhase() == "gestation")
        return true;
    auto vec = _classifierVec(state);
    return _classifier.classify(vec) || _pessimistic_classifier.classify(vec);
}

bool Skill::canStart(const AbstractedState &state) const
{
    if (_is_global || getTrainingPhase() == "gestation")
        return true;
    auto vec = _classifierVec(state);
    return _classifier.classify(vec) || _pessimistic_classifier.classify(vec);
}

bool Skill::canStartPessimistic(const RobotState &state) const
{
    if (_is_global || getTrainingPhase() == "gestation")
        return true;
    return _pessimistic_classifier.classify(_classifierVec(state));
}

bool Skill::canStartPessimistic(const AbstractedState &state) const
{
    if (_is_global || getTrainingPhase() == "gestation")
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

double Skill::getDecisionValue(const AbstractedState &state) const
{
    auto vec = _classifierVec(state);
    return _pessimistic_classifier.decisionValue(vec);
}

// AbstractedState Skill::sampleSubgoalState() const
// {
//     if (_positive_gestation_records.empty())
//         return _env->getRandomValidAbstractedState();

//     // Build candidate indices: prefer pessimistic classifier region when available.
//     std::vector<size_t> candidate_indices;
//     if (_pessimistic_classifier.trained())
//     {
//         for (size_t i = 0; i < _positive_gestation_records.size(); i++)
//             if (_pessimistic_classifier.classify(_positive_gestation_records[i].classifier_vec))
//                 candidate_indices.push_back(i);
//     }
//     if (candidate_indices.empty())
//     {
//         std::cout << "nothing in pessimistic classifier, sampling from optimistic positives\n";
//         candidate_indices.resize(_positive_gestation_records.size());
//         std::iota(candidate_indices.begin(), candidate_indices.end(), 0); // iota can fill vector with sequentually increasing elements
//     }

//     // Filter to k-nearest and reject obstacle crossings.
//     if (candidate_indices.size() > 1)
//     {
//         constexpr size_t kNearest = 10;
//         AbstractedState current_state = _env->getAbstractedState();

//         // Sort candidates by distance to current state
//         std::sort(candidate_indices.begin(), candidate_indices.end(),
//                   [&](size_t a, size_t b)
//                   {
//                       float da = _euclideanDistance2D(current_state.position, // if we change the goal representation, need this to be larger
//                                                       _positive_gestation_records[a].state.position, false);
//                       float db = _euclideanDistance2D(current_state.position,
//                                                       _positive_gestation_records[b].state.position, false);
//                       return da < db;
//                   });

//         if (candidate_indices.size() > kNearest)
//             candidate_indices.resize(kNearest);

//         // Reject candidates whose line-of-sight crosses an obstacle
//         std::vector<size_t> reachable;
//         for (size_t idx : candidate_indices)
//         {
//             if (!_segmentIntersectsObstacle(current_state.position,
//                                             _positive_gestation_records[idx].state.position))
//                 reachable.push_back(idx);
//         }

//         if (!reachable.empty())
//             candidate_indices = std::move(reachable);
//     }

//     std::uniform_int_distribution<size_t> dist(0, candidate_indices.size() - 1);
//     AbstractedState subgoal = _positive_gestation_records[candidate_indices[dist(_rng)]].state;
//     if (!kUseVelocityInClassifier)
//     {
//         subgoal.velocity = {0.0f, 0.0f, 0.0f};
//         subgoal.angular_velocity = {0.0f, 0.0f, 0.0f};
//     }
//     return subgoal;
// }

AbstractedState Skill::sampleSubgoalState() const
{
    if (_positive_gestation_records.empty())
        return _env->getRandomValidAbstractedState();

    // 1. Identify candidate indices based on the parent's pessimistic region
    std::vector<size_t> candidate_indices;
    for (size_t i = 0; i < _positive_gestation_records.size(); ++i)
    {
        if (_parent && _parent->canStartPessimistic(_positive_gestation_records[i].state))
        {
            candidate_indices.push_back(i);
        }
    }

    // Fallback if the pessimistic set is too small/empty during early training
    if (candidate_indices.empty())
    {
        candidate_indices.resize(_positive_gestation_records.size());
        std::iota(candidate_indices.begin(), candidate_indices.end(), 0);
    }

    // 2. Perform Epsilon-Tolerance Check (The "Landing Pad" logic)
    // Only accept states where a small neighborhood is also considered "Positive"
    float tolerance = 0.25f; // Matches the Python 'tolerance' logic
    std::vector<size_t> robust_indices;

    for (size_t idx : candidate_indices)
    {
        const auto &pos = _positive_gestation_records[idx].state.position;
        bool neighborhood_valid = true;

        // Check 4 points around the candidate (Cross pattern)
        std::vector<std::array<float, 3>> neighbors = {
            {pos[0] + tolerance, pos[1], pos[2]},
            {pos[0] - tolerance, pos[1], pos[2]},
            {pos[0], pos[1] + tolerance, pos[2]},
            {pos[0], pos[1] - tolerance, pos[2]}};

        for (const auto &n_pos : neighbors)
        {
            AbstractedState n_state = _positive_gestation_records[idx].state;
            n_state.position = n_pos;
            // The Python reference requires the neighborhood to be valid in the classifier
            if (_parent && !_parent->canStartPessimistic(n_state))
            {
                neighborhood_valid = false;
                break;
            }
        }

        if (neighborhood_valid)
            robust_indices.push_back(idx);
    }

    // 3. Final selection from robust candidates
    // if (robust_indices.empty())
    // {
    //     std::cout << "No robust candidates found, sampling from all positives\n";
    // }
    const auto &final_pool = robust_indices.empty() ? candidate_indices : robust_indices;

    // Nearest-neighbor filtering to keep the chain tight
    AbstractedState current_state = _env->getAbstractedState();
    std::vector<size_t> k_nearest = final_pool;

    std::sort(k_nearest.begin(), k_nearest.end(), [&](size_t a, size_t b)
              { return _euclideanDistance2D(current_state.position, _positive_gestation_records[a].state.position, false) <
                       _euclideanDistance2D(current_state.position, _positive_gestation_records[b].state.position, false); });

    // empirically, keeping the largest 5% of the positives seems to balance exploration and convergence well
    // before, kept at fixed size of 10, but this often led to oversampling the same few points near to obstacles
    size_t k_value = std::max((size_t)10, final_pool.size() / 10);
    if (k_nearest.size() > k_value)
        k_nearest.resize(k_value);

    std::uniform_int_distribution<size_t> dist(0, k_nearest.size() - 1);
    return _positive_gestation_records[k_nearest[dist(_rng)]].state;
}

// 2D line-segment vs cylinder (circle) intersection test.
// Returns true if the segment from a to b passes through any obstacle.
bool Skill::_segmentIntersectsObstacle(const std::array<float, 3> &a, const std::array<float, 3> &b) const
{
    const auto &obstacles = _env->getObstacles();
    float dx = b[0] - a[0];
    float dy = b[1] - a[1];
    float seg_len_sq = dx * dx + dy * dy;

    for (const auto &obs : obstacles)
    {
        float radius = obs.size[0];
        // Vector from a to obstacle center
        float fx = a[0] - obs.position[0];
        float fy = a[1] - obs.position[1];

        // Quadratic: t^2*(d.d) + 2t*(f.d) + (f.f - r^2) = 0
        float a_coeff = seg_len_sq;
        float b_coeff = 2.0f * (fx * dx + fy * dy);
        float c_coeff = fx * fx + fy * fy - radius * radius;
        float discriminant = b_coeff * b_coeff - 4.0f * a_coeff * c_coeff;

        if (discriminant < 0)
            continue;

        float sqrt_disc = std::sqrt(discriminant);
        float t1 = (-b_coeff - sqrt_disc) / (2.0f * a_coeff);
        float t2 = (-b_coeff + sqrt_disc) / (2.0f * a_coeff);

        // Intersection if either root is in [0, 1] or the segment is fully inside
        if (t1 <= 1.0f && t2 >= 0.0f)
            return true;
    }
    return false;
}

void Skill::save(const std::string &actor_path,
                 const std::string &critic1_path,
                 const std::string &critic2_path,
                 const std::string &classifier_path) const
{
    torch::save(_agent.actor_local, actor_path);
    torch::save(_agent.critic_local_1, critic1_path);
    torch::save(_agent.critic_local_2, critic2_path);

    if (!_is_global && _classifier.trained())
    {
        _classifier.save(classifier_path);
        std::cout << "[Skill " << _id << "] Saved optimistic classifier to " << classifier_path << "\n";
    }

    if (_pessimistic_classifier.trained())
    {
        _pessimistic_classifier.save(classifier_path + "_pessimistic");
        std::cout << "[Skill " << _id << "] Saved pessimistic classifier to " << classifier_path + "_pessimistic" << "\n";
    }

    auto write_records = [](const std::string &path, const std::vector<GestationRecord> &records) {
        std::ofstream out(path);
        out << records.size() << "\n";
        for (const auto &record : records)
        {
            for (float v : record.state.position)         out << v << " ";
            for (float v : record.state.orientation)      out << v << " ";
            for (float v : record.state.velocity)         out << v << " ";
            for (float v : record.state.angular_velocity) out << v << " ";
            out << "\n";
        }
    };

    if (!_is_global && !_positive_gestation_records.empty())
        write_records(classifier_path + "_positives.txt", _positive_gestation_records);

    if (!_is_global && !_effect_records.empty())
        write_records(classifier_path + "_effect.txt", _effect_records);
}

void Skill::load(const std::string &actor_path,
                 const std::string &critic1_path,
                 const std::string &critic2_path,
                 const std::string &classifier_path)
{
    _positive_gestation_records.clear();

    torch::load(_agent.actor_local, actor_path);
    torch::load(_agent.critic_local_1, critic1_path);
    torch::load(_agent.critic_local_2, critic2_path);

    if (!_is_global && std::filesystem::exists(classifier_path))
    {
        try
        {
            _classifier.load(classifier_path);
            std::cout << "[Skill " << _id << "] Loaded optimistic classifier from " << classifier_path << "\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << "[Skill " << _id << "] Failed to load optimistic classifier: " << e.what() << "\n";
        }
    }

    if (std::filesystem::exists(classifier_path + "_pessimistic"))
    {
        try
        {
            _pessimistic_classifier.load(classifier_path + "_pessimistic");
            std::cout << "[Skill " << _id << "] Loaded pessimistic classifier from " << classifier_path + "_pessimistic" << "\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << "[Skill " << _id << "] Failed to load pessimistic classifier: " << e.what() << "\n";
        }
    }

    std::ifstream f_pos(classifier_path + "_positives.txt");
    if (f_pos.good() && !_is_global)
    {
        size_t count = 0;
        f_pos >> count;
        for (size_t i = 0; i < count; i++)
        {
            AbstractedState state;
            for (auto &v : state.position)
                f_pos >> v;
            for (auto &v : state.orientation)
                f_pos >> v;
            for (auto &v : state.velocity)
                f_pos >> v;
            for (auto &v : state.angular_velocity)
                f_pos >> v;
            _positive_gestation_records.push_back({_classifierVec(state), state});
        }
    }

    auto load_records = [&](const std::string &path, std::vector<GestationRecord> &records) {
        std::ifstream f(path);
        if (!f.good() || _is_global) return;
        size_t count = 0; f >> count;
        for (size_t i = 0; i < count; i++)
        {
            AbstractedState state;
            for (auto &v : state.position)         f >> v;
            for (auto &v : state.orientation)      f >> v;
            for (auto &v : state.velocity)         f >> v;
            for (auto &v : state.angular_velocity) f >> v;
            records.push_back({_classifierVec(state), state});
        }
    };

    _effect_records.clear();
    load_records(classifier_path + "_effect.txt", _effect_records);

    if (!_is_global && _classifier.trained())
    {
        _validated = true;
        _goal_hits = _gestation_period;
    }
}

TD3Agent &Skill::agent()
{
    return _agent;
}

// if we change the initiation set, will need to change this as well
float Skill::distanceToState(const AbstractedState &state) const
{
    float min_dist = std::numeric_limits<float>::max();

    for (const auto &start : _positive_gestation_records)
    {
        float dist = 0;

        dist += _euclideanDistance2D(state.position, start.state.position, false);

        min_dist = std::min(dist, min_dist);
    }
    return min_dist;
}

bool Skill::isValidInitData(const std::vector<GestationRecord> &visited) const
{
    if (!_parent || visited.empty())
        return true;

    // gather mature siblings with a trained pessimistic classifier
    std::vector<std::shared_ptr<Skill>> siblings;
    for (const auto &s : _parent->children)
        if (s.get() != this && s->getTrainingPhase() != "gestation" && s->_pessimistic_classifier.trained())
            siblings.push_back(s);

    if (siblings.empty())
        return true;

    // count visited states inside a sibling's pessimistic region but outside parent's pessimistic region
    // mirrors Python: accept if 0 < ratio <= 0.35
    float sibling_count = 0.0f;
    for (const auto &rec : visited)
        for (const auto &sib : siblings)
            if (sib->canStartPessimistic(rec.state) && !_parent->canStartPessimistic(rec.state))
            {
                sibling_count += 1.0f;
                break; // count each state once even if multiple siblings match
            }

    float ratio = sibling_count / static_cast<float>(visited.size());
    return ratio > 0.0f && ratio <= 0.35f;
}

/*** Private ***/

void Skill::_herUpdate(const std::vector<Transition> &trajectory)
{
    if (trajectory.size() < 2)
        return;

    constexpr int skip_last = 3;
    int end_idx = (int)trajectory.size() - 1;

    if (trajectory.back().in_collision)
    {
        for (int i = end_idx; i >= 0; i--)
        {
            if (!trajectory[i].in_collision)
            {
                end_idx = std::max(0, i - skip_last);
                break;
            }
        }
    }

    if (end_idx < 1)
        return;

    auto add_her_experience = [&](const Transition &t, const AbstractedState &goal)
    {
        auto aug_state = _env->transformState(t.state, goal);
        auto aug_next = _env->transformState(t.next_state, goal);
        auto [aug_reward, aug_done] = _env->computeReward(t.state, t.in_collision, goal);
        _agent.addExperience(aug_state, t.action, aug_reward, aug_next, aug_done);
        _agent.learn();

        if (!_is_global)
        {
            auto &global_agent = _global_option->agent();
            global_agent.addExperience(aug_state, t.action, aug_reward, aug_next, aug_done);
            global_agent.learn();
        }
    };

    // "Final" strategy
    AbstractedState final_goal;
    final_goal.position = trajectory[end_idx].next_state.position;
    final_goal.orientation = trajectory[end_idx].next_state.orientation;
    final_goal.velocity = {0, 0, 0};
    final_goal.angular_velocity = {0, 0, 0};

    for (int i = 0; i <= end_idx; i++)
        add_her_experience(trajectory[i], final_goal);

    // "Future" strategy
    static std::mt19937 rng(std::random_device{}());
    for (int i = 0; i < end_idx; i++)
    {
        std::uniform_int_distribution<int> dist(i + 1, end_idx);
        int future_idx = dist(rng);

        AbstractedState future_goal;
        future_goal.position = trajectory[future_idx].next_state.position;
        future_goal.orientation = trajectory[future_idx].next_state.orientation;
        future_goal.velocity = {0, 0, 0};
        future_goal.angular_velocity = {0, 0, 0};

        add_her_experience(trajectory[i], future_goal);
    }
}

// void Skill::_fitClassifier(const std::vector<GestationRecord> &visited, bool term_success)
// {

//     if (term_success)
//     {
//         // Include the start state as a positive example for chained options —
//         // it's a valid point the agent can reach the parent's set from.
//         // Skip for the goal option (_parent == nullptr) — its initiation set
//         // should tightly cover the goal region, not the scattered start positions.
//         if (_parent != nullptr)
//         {
//             _positive_gestation_records.push_back(visited.front());
//             _gestation_vecs.push_back(visited.front().classifier_vec);
//             _gestation_labels.push_back(1);
//         }

//         // add the last k states to the positive set
//         for (int t = visited.size() - 1; t >= std::max(0, (int)visited.size() - _k); t--)
//         {
//             _positive_gestation_records.push_back(visited[t]);
//             _gestation_vecs.push_back(visited[t].classifier_vec);
//             _gestation_labels.push_back(1);
//         }
//     }
//     else
//     {
//         _gestation_vecs.push_back(visited.front().classifier_vec);
//         _gestation_labels.push_back(-1);
//         _has_negative_gestation = true;
//     }

//     // trim positives and negatives independently so rare negatives aren't evicted
//     // int max_pos = _k * _gestation_period;
//     // int max_neg = _gestation_period;  // at most 1 negative per rollout

//     // int pos_count = std::count(_gestation_labels.begin(), _gestation_labels.end(), 1);
//     // int neg_count = std::count(_gestation_labels.begin(), _gestation_labels.end(), -1);
//     // int pos_excess = std::max(0, pos_count - max_pos);
//     // int neg_excess = std::max(0, neg_count - max_neg);

//     // if (pos_excess > 0 || neg_excess > 0)
//     // {
//     //     int pos_removed = 0, neg_removed = 0;
//     //     std::vector<std::vector<float>> new_vecs;
//     //     std::vector<int> new_labels;
//     //     new_vecs.reserve(_gestation_vecs.size() - pos_excess - neg_excess);
//     //     new_labels.reserve(new_vecs.capacity());
//     //     for (size_t i = 0; i < _gestation_vecs.size(); i++)
//     //     {
//     //         if (_gestation_labels[i] == 1 && pos_removed < pos_excess) { pos_removed++; continue; }
//     //         if (_gestation_labels[i] == -1 && neg_removed < neg_excess) { neg_removed++; continue; }
//     //         new_vecs.push_back(std::move(_gestation_vecs[i]));
//     //         new_labels.push_back(_gestation_labels[i]);
//     //     }
//     //     _gestation_vecs = std::move(new_vecs);
//     //     _gestation_labels = std::move(new_labels);

//     //     _has_negative_gestation = std::find(_gestation_labels.begin(), _gestation_labels.end(), -1)
//     //                               != _gestation_labels.end();
//     // }
//     // if ((int)_positive_gestation_records.size() > max_pos)
//     // {
//     //     int excess = _positive_gestation_records.size() - max_pos;
//     //     _positive_gestation_records.erase(_positive_gestation_records.begin(),
//     //                                       _positive_gestation_records.begin() + excess);
//     // }

//     // always attempt to fit after every rollout
//     if (_positive_gestation_records.empty())
//         return;

//     if (!_has_negative_gestation)
//     {
//         // only positive data available — use one-class SVMs on positive vecs only.
//         std::vector<std::vector<float>> pos_vecs;
//         for (size_t i = 0; i < _gestation_vecs.size(); i++)
//             if (_gestation_labels[i] == 1)
//                 pos_vecs.push_back(_gestation_vecs[i]);

//         if (!pos_vecs.empty())
//         {
//             bool first_phase1 = !_classifier.trained();
//             _pessimistic_classifier.trainOneClass(pos_vecs, _nu); // tight
//             _classifier.trainOneClass(pos_vecs, _nu / 10.0);      // loose / optimistic
//             if (first_phase1)
//                 std::cout << "\n[Skill " << _id << "] Classifier Phase 1: OneClass init. Pos=" << pos_vecs.size() << "\n";
//         }
//     }
//     else
//     {
//         // binary SVC as optimistic, then one-class re-fit on SVC-positive predictions as pessimistic.
//         int neg_count = std::count(_gestation_labels.begin(), _gestation_labels.end(), -1);
//         _classifier.train(_gestation_vecs, _gestation_labels,
//                           /*C=*/1.0, /*gamma=*/1.0, /*balance_classes=*/true);

//         // Re-fit pessimistic on only the states the optimistic SVC predicts as positive
//         std::vector<std::vector<float>> svc_positive_vecs;
//         for (const auto &vec : _gestation_vecs)
//             if (_classifier.classify(vec))
//                 svc_positive_vecs.push_back(vec);

//         if (!svc_positive_vecs.empty())
//             _pessimistic_classifier.trainOneClass(svc_positive_vecs, _nu);

//         if (neg_count == 1) // first failure — log Phase 1→2 transition
//         {
//             int pos_count = std::count(_gestation_labels.begin(), _gestation_labels.end(), 1);
//             std::cout << "\n[Skill " << _id << "] Classifier Phase 1→2: binary SVC. Pos=" << pos_count << " Neg=1\n";
//         }
//     }
// }



void Skill::_fitClassifier(const std::vector<GestationRecord> &visited, bool term_success)
{
    if (term_success)
    {
        // 1. Capture the "Entry Point": The state where the robot began this successful rollout.
        // Feeds the SVM (initiation set classifier) only — not the effect set.
        if (_parent != nullptr)
        {
            _positive_gestation_records.push_back(visited.front());
            _gestation_vecs.push_back(visited.front().classifier_vec);
            _gestation_labels.push_back(1);
        }

        // 2. Capture the "Success Buffer": The last K states of the trajectory.
        // These go into the initiation set proxy (classifier training).
        // The effect set E_o gets only the single terminal state — the state at which
        // atTermination() fired. Approach states are outside the parent's boundary and
        // would cause edge inference (E_i ⊆ I_j) to fail spuriously.
        int buffer_size = std::min((int)visited.size(), _k);
        int start_idx = (int)visited.size() - buffer_size;

        for (int t = start_idx; t < (int)visited.size(); ++t)
        {
            _positive_gestation_records.push_back(visited[t]);
            _gestation_vecs.push_back(visited[t].classifier_vec);
            _gestation_labels.push_back(1);
        }
        _effect_records.push_back(visited.back());
    }
    else
    {
        // 3. Negative Examples: If the rollout failed, we mark the start state
        // as a negative example to prevent the classifier from over-expanding here.
        _gestation_vecs.push_back(visited.front().classifier_vec);
        _gestation_labels.push_back(-1);
        _has_negative_gestation = true;
    }

    // Guard: Don't train if we have no successful data yet.
    if (_positive_gestation_records.empty())
        return;

    // --- Training Phase ---
    if (!_has_negative_gestation)
    {
        // PHASE 1: No failures yet. Train One-Class SVMs.
        std::vector<std::vector<float>> pos_vecs;
        for (size_t i = 0; i < _gestation_vecs.size(); i++)
        {
            if (_gestation_labels[i] == 1)
                pos_vecs.push_back(_gestation_vecs[i]);
        }

        if (!pos_vecs.empty())
        {
            // Nu controls tightness; Gamma (now ~1.0) controls localization.
            _pessimistic_classifier.trainOneClass(pos_vecs, _nu);
            _classifier.trainOneClass(pos_vecs, _nu / 10.0);
        }
    }
    else
    {
        // PHASE 2: Negative data exists. Train a Binary SVC as the Optimistic Classifier.
        // This allows the agent to "learn from mistakes" by carving out negative space.
        _classifier.train(_gestation_vecs, _gestation_labels,
                          /*C=*/1.0, /*gamma=*/0.5, /*balance_classes=*/true);

        // Filter: Train the Pessimistic One-Class SVM only on states the SVC confirms as Positive.
        std::vector<std::vector<float>> svc_positive_vecs;
        for (const auto &vec : _gestation_vecs)
        {
            if (_classifier.classify(vec))
                svc_positive_vecs.push_back(vec);
        }

        if (!svc_positive_vecs.empty())
        {
            _pessimistic_classifier.trainOneClass(svc_positive_vecs, _nu);
        }
    }
}

bool Skill::_atLocalGoal(const AbstractedState &goal) const
{
    auto [underlying_state, collision] = _env->getUnderlyingState();

    bool use_goal_radius = true; //(_parent == nullptr || _is_global); // only use goal radius for global option, not subgoal options
    auto [reward, done] = _env->computeReward(underlying_state, collision, goal, use_goal_radius);
    bool env_done = done.data_ptr<float>()[0] > 0.5f;
    float r = reward.data_ptr<float>()[0];

    if (_is_global || !_parent)
    {
        return env_done;
    }

    bool at_goal = env_done && (r > 45);
    bool bad_termination = env_done && (r < 45); // collision, OOB, timeout
    auto abs_state = _env->getAbstractedState();
    bool pessimistic = _parent->canStartPessimistic(abs_state);
    bool near_boundary = _parent->canStart(abs_state) && _parent->getDecisionValue(abs_state) > -0.01;
    bool in_parent = pessimistic || near_boundary;
    if (at_goal && !in_parent)
    {
        std::cout << "Warning: reached goal state outside of parent's pessimistic initiation set. dval=" << _parent->getDecisionValue(abs_state) << "\n";
    }
    return (in_parent && at_goal) || bad_termination;
}

std::vector<float> Skill::_classifierVec(const AbstractedState &state) const
{
    std::vector<float> out;
    auto scaling_factors = _env->env_scaling_factors;
    out.reserve(2);

    // global pos (x, y only — z is ~constant standing height and causes mismatch with AbstractedState z=0)
    out.push_back(state.position[0] / scaling_factors.position[0]);
    out.push_back(state.position[1] / scaling_factors.position[1]);
    // global vel — zeroed when kUseVelocityInClassifier is false (environment spawns with zero velocity)
    // out.push_back(kUseVelocityInClassifier ? state.velocity[0] / scaling_factors.velocity[0] : 0.0f);
    // out.push_back(kUseVelocityInClassifier ? state.velocity[1] / scaling_factors.velocity[1] : 0.0f);
    // out.push_back(kUseVelocityInClassifier ? state.velocity[2] / scaling_factors.velocity[2] : 0.0f);

    // // // orientation
    // if (state.orientation[0] < 0)
    // {
    //     out.push_back(-state.orientation[0] / scaling_factors.orientation[0]);
    //     out.push_back(-state.orientation[1] / scaling_factors.orientation[1]);
    //     out.push_back(-state.orientation[2] / scaling_factors.orientation[2]);
    //     out.push_back(-state.orientation[3] / scaling_factors.orientation[3]);
    // }
    // else
    // {
    //     out.push_back(state.orientation[0] / scaling_factors.orientation[0]);
    //     out.push_back(state.orientation[1] / scaling_factors.orientation[1]);
    //     out.push_back(state.orientation[2] / scaling_factors.orientation[2]);
    //     out.push_back(state.orientation[3] / scaling_factors.orientation[3]);
    // }
    // // ang vel — zeroed when kUseVelocityInClassifier is false
    // out.push_back(kUseVelocityInClassifier ? state.angular_velocity[0] / scaling_factors.angular_velocity[0] : 0.0f);
    // out.push_back(kUseVelocityInClassifier ? state.angular_velocity[1] / scaling_factors.angular_velocity[1] : 0.0f);
    // out.push_back(kUseVelocityInClassifier ? state.angular_velocity[2] / scaling_factors.angular_velocity[2] : 0.0f);
    return out;
}

std::vector<float> Skill::_classifierVec(const RobotState &state) const
{
    std::vector<float> out;
    auto scaling_factors = _env->env_scaling_factors;
    out.reserve(2);

    // global pos (x, y only — z is ~constant standing height and causes mismatch with AbstractedState z=0)
    out.push_back(state.position[0] / scaling_factors.position[0]);
    out.push_back(state.position[1] / scaling_factors.position[1]);
    return out;
}

float Skill::_euclideanDistance2D(const std::array<float, 3> &a, const std::array<float, 3> &b, bool sqrt) const
{
    float dx = a[0] - b[0];
    float dy = a[1] - b[1];
    float dist = dx * dx + dy * dy;
    if (sqrt)
    {
        return std::sqrt(dist);
    }
    return dist;
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
