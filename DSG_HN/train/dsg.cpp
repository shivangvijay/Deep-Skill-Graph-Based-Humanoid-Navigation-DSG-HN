#include "dsg.h"
#include <filesystem>
#include <fstream>
#include <queue>
#include <unordered_set>

// =============================================================================
// State conversion helpers
// =============================================================================

// Convert the full robot state (including joints) to the compact representation
// expected by the transition model's feature extractor.
static RolloutState robotStateToRolloutState(const RobotState &rs)
{
    RolloutState out;
    out.x = rs.position[0];
    out.y = rs.position[1];
    out.z = rs.position[2];

    // Extract yaw from quaternion [qw, qx, qy, qz]
    const double qw = rs.orientation[0], qx = rs.orientation[1];
    const double qy = rs.orientation[2], qz = rs.orientation[3];
    out.yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz));

    out.vx = rs.velocity[0];
    out.vy = rs.velocity[1];
    out.oz = rs.angular_velocity[2];

    out.joint_pos.assign(rs.q.begin(),  rs.q.end());
    out.joint_vel.assign(rs.dq.begin(), rs.dq.end());
    return out;
}

// Project a RolloutState back to the navigation-level AbstractedState used by DSG.
// Orientation is recovered as a yaw-only quaternion (pitch = roll = 0).
static AbstractedState rolloutStateToAbstractedState(const RolloutState &rs)
{
    AbstractedState as;
    as.position    = { static_cast<float>(rs.x),
                       static_cast<float>(rs.y),
                       static_cast<float>(rs.z) };
    const float qw = static_cast<float>(std::cos(rs.yaw * 0.5));
    const float qz = static_cast<float>(std::sin(rs.yaw * 0.5));
    as.orientation = { qw, 0.0f, 0.0f, qz };
    as.velocity    = { static_cast<float>(rs.vx),
                       static_cast<float>(rs.vy), 0.0f };
    as.angular_velocity = { 0.0f, 0.0f, static_cast<float>(rs.oz) };
    return as;
}

// =============================================================================
// Transition model loading
// =============================================================================

void DeepSkillGraph::loadTransitionModel(const std::string &model_path,
                                          const std::string &normaliser_path)
{
    _transition_model = GaussianMLP(kTM_InputDim, kTM_OutputDim, /*dropout=*/0.2);
    torch::load(_transition_model, model_path, _device);
    _transition_model->to(_device);
    _transition_model->eval();

    _normaliser = TransitionNormaliser::load(normaliser_path);

    _has_transition_model = true;
    std::cout << "[DSG] Transition model loaded from " << model_path << "\n";
}

// =============================================================================
// DSC overrides
// =============================================================================

void DeepSkillGraph::_makeSkill(bool is_global, std::shared_ptr<Skill> parent)
{
    DeepSkillChaining::_makeSkill(is_global, parent);
    if (parent)
        parent->children.push_back(_skills.back());
}

bool DeepSkillGraph::_shouldCreateNewOption()
{
    for (int o = _global_option_idx + 1; o < (int)_skills.size(); o++)
        if (_skills[o]->getTrainingPhase() != "mature") return false;

    if (_containsGlobalStartState()) return false;

    // DSG: respect max_children_per_node on the frontier parent
    if ((int)_skills[_unfinished_option_idx]->children.size() >= _dsg_cfg.max_children_per_node)
        return false;

    return true;
}

void DeepSkillGraph::_validateOption()
{
    DeepSkillChaining::_validateOption();
    _updateEdges();
}

std::pair<int, AbstractedState> DeepSkillGraph::_pickOption()
{
    // TODO: replace with policy over options
    // Current implementation runs Dijkstra from _currentNodeIdx() (single node, approximate).
    // This is the case (b) fallback used by _navigateTo() when no graph path exists.
    auto [cost, path] = _dijkstraPath(_currentNodeIdx(), _global_option_idx + 1);
    if (!path.empty())
    {
        int next = path[0];
        if (next < (int)_skills.size())
        {
            auto global_state = _env->getAbstractedState();
            if (_skills[next]->canStart(global_state) && !_skills[next]->atTermination(_global_goal))
                return {next, _skills[next]->getLocalGoal()};
        }
    }
    return DeepSkillChaining::_pickOption();
}

// =============================================================================
// Main training loop
// =============================================================================

int DeepSkillGraph::train(int max_episodes)
{
    if (_dsg_cfg.render_training)
        _robot_bridge->startRender();

    for (int episode = 0; episode < max_episodes; episode++)
    {
        _env->resetTo(_global_start); // this can either be a fixed position or come from a small set of states. 
        
        // Determine phase
        if (episode < _dsg_cfg.warmup_episodes)
        {
            _warmupRollout();
        }
        else
        {
            if (episode % _dsg_cfg.expansion_freq == 0)
            {
                bool expanded = false;
                for (int attempt = 0; attempt < _dsg_cfg.max_expansion_tries && !expanded; attempt++)
                    expanded = _graphExpansionPhase();
                if (!expanded)
                {
                    std::cout << "[DSG] Expansion exhausted " << _dsg_cfg.max_expansion_tries
                              << " attempts, falling back to consolidation.\n";
                    _graphConsolidationPhase();
                }
            }
            else
                _graphConsolidationPhase();

            // TODO: update transition model

            // _validateOption(); // run validation phase for newly matured options

        }

        if (_dsg_cfg.graph_update_freq > 0 && episode % _dsg_cfg.graph_update_freq == 0)
            // connect new options to graph and update edges
            _updateEdges();

        if (_dsg_cfg.log_interval > 0 && (episode + 1) % _dsg_cfg.log_interval == 0)
        {
            std::cout << "\n[Episode " << (episode + 1) << "]\n";
            std::cout << "=== Skill Status ===\n";
            std::cout << "  ID      Phase       GoalHits  Children\n";
            for (size_t i = 0; i < _skills.size(); ++i)
            {
                std::string label = (i == (size_t)_global_option_idx) ? "global"
                                  : (i == (size_t)_global_option_idx + 1) ? "goal"
                                  : "opt-" + std::to_string(i);
                std::string phase = (i == (size_t)_global_option_idx) ? "pre-trained"
                                  : _skills[i]->getTrainingPhase();
                std::cout << "  " << label << "   " << phase;
                if (i == (size_t)_global_option_idx)
                    std::cout << "\n";
                else
                    std::cout << "   " << _skills[i]->goalHits() << "/" << _skills[i]->gestationPeriod()
                              << "  children=" << _skills[i]->children.size() << "\n";
            }
            std::cout << "  GoalRegions: " << _goal_regions.size()
                      << "  Edges: " << _edges.size() << "\n";
            if (_dsg_cfg.visualize_initiation_sets)
                visualizeInitiationSets();
        }

        if (_containsGlobalStartState())
        {
            std::cout << "Success!\n";
            break;
        }
    }
    return (int)_skills.size() - 1;
}

// =============================================================================
// save / load
// =============================================================================

void DeepSkillGraph::save(const std::string &dir) const
{
    DeepSkillChaining::save(dir);

    // Parent topology
    std::ofstream topo(dir + "/parent_ids.txt");
    for (size_t i = 0; i < _skills.size(); ++i)
    {
        int parent_id = -1;
        for (size_t j = 0; j < _skills.size(); ++j)
            for (const auto &child : _skills[j]->children)
                if (child == _skills[i]) { parent_id = (int)j; break; }
        topo << parent_id << "\n";
    }

    // Explicit edges
    std::ofstream edge_file(dir + "/edges.txt");
    edge_file << _edges.size() << "\n";
    for (const auto &e : _edges)
        edge_file << e.from << " " << e.to << " " << e.weight << "\n";

    // Goal regions
    std::ofstream gr_file(dir + "/goal_regions.txt");
    gr_file << _goal_regions.size() << "\n";
    for (const auto &gr : _goal_regions)
    {
        gr_file << gr.epsilon;
        for (float v : gr.center.position)         gr_file << " " << v;
        for (float v : gr.center.orientation)      gr_file << " " << v;
        for (float v : gr.center.velocity)         gr_file << " " << v;
        for (float v : gr.center.angular_velocity) gr_file << " " << v;
        gr_file << "\n";
    }

    std::cout << "Saved graph: " << _edges.size() << " edges, "
              << _goal_regions.size() << " goal regions to " << dir << "\n";
}

void DeepSkillGraph::load(const std::string &dir, const std::string &scene_file)
{
    // Read parent topology before base load creates skills
    std::vector<int> parent_ids;
    {
        std::ifstream topo(dir + "/parent_ids.txt");
        if (topo.is_open()) { int pid; while (topo >> pid) parent_ids.push_back(pid); }
    }

    DeepSkillChaining::load(dir, scene_file);

    // Restore children links
    for (size_t i = 0; i < _skills.size() && i < parent_ids.size(); ++i)
    {
        int pid = parent_ids[i];
        if (pid >= 0 && pid < (int)_skills.size())
            _skills[pid]->children.push_back(_skills[i]);
    }

    // Restore edges
    _edges.clear();
    {
        std::ifstream edge_file(dir + "/edges.txt");
        if (edge_file.is_open())
        {
            int n; edge_file >> n;
            for (int i = 0; i < n; i++)
            {
                Edge e; edge_file >> e.from >> e.to >> e.weight;
                _edges.push_back(e);
            }
        }
    }

    // Restore goal regions
    _goal_regions.clear();
    {
        std::ifstream gr_file(dir + "/goal_regions.txt");
        if (gr_file.is_open())
        {
            int n; gr_file >> n;
            for (int i = 0; i < n; i++)
            {
                GoalRegion gr;
                gr_file >> gr.epsilon;
                for (float &v : gr.center.position)         gr_file >> v;
                for (float &v : gr.center.orientation)      gr_file >> v;
                for (float &v : gr.center.velocity)         gr_file >> v;
                for (float &v : gr.center.angular_velocity) gr_file >> v;
                _goal_regions.push_back(gr);
            }
        }
    }

    std::cout << "Restored graph: " << _edges.size() << " edges, "
              << _goal_regions.size() << " goal regions from " << dir << "\n";
}

// =============================================================================
// Graph edge management
// =============================================================================

void DeepSkillGraph::_updateEdges()
{
    int n_skills = (int)_skills.size();
    int n_gr     = (int)_goal_regions.size();

    auto edge_exists = [&](int from, int to) {
        for (const auto &e : _edges) if (e.from == from && e.to == to) return true;
        return false;
    };

    // skill → skill edges: effect_set(i) ⊆ initiation_set(j)
    // O(n^2) connecting skills based on effect set containment in initiation set
    for (int i = _global_option_idx + 1; i < n_skills; i++)
    {
        if (_skills[i]->getTrainingPhase() != "mature") continue;
        const auto &records = _skills[i]->getPositiveGestationRecords();
        if (records.empty()) continue;

        for (int j = _global_option_idx + 1; j < n_skills; j++)
        {
            if (i == j || edge_exists(i, j)) continue;
            bool all_covered = true;
            for (const auto &rec : records)
                if (!_skills[j]->canStartPessimistic(rec.state)) { all_covered = false; break; }
            if (all_covered)
            {
                _edges.push_back({i, j, 0.0f});
                std::cout << "[DSG] Edge " << i << " → " << j << "\n";
            }
        }

        // skill → goal_region edges
        for (int r = 0; r < n_gr; r++)
        {
            int gr_idx = n_skills + r;
            if (edge_exists(i, gr_idx)) continue;
            bool all_covered = true;
            for (const auto &rec : records)
                if (!_nodeCanStart(gr_idx, rec.state)) { all_covered = false; break; }
            if (all_covered)
            {
                _edges.push_back({i, gr_idx, 0.0f});
                std::cout << "[DSG] Edge " << i << " → GR" << r << "\n";
            }
        }
    }
}

std::pair<float, std::vector<int>> DeepSkillGraph::_dijkstraPath(int from_node, int to_node) const
{
    int N = _totalNodes();

    std::vector<float> dist(N, std::numeric_limits<float>::infinity());
    std::vector<int>   prev(N, -1);
    dist[from_node] = 0.0f;

    auto neighbors = [&](int u) -> std::vector<std::pair<int, float>> {
        std::vector<std::pair<int, float>> nbrs;
        for (const auto &e : _edges)
            if (e.from == u) nbrs.push_back({e.to, e.weight});
        // implicit parent→child edges for skill nodes
        if (u < (int)_skills.size())
            for (int k = 0; k < (int)_skills.size(); k++)
                for (const auto &child : _skills[u]->children)
                    if (_skills[k] == child) { nbrs.push_back({k, 0.0f}); break; }
        return nbrs;
    };

    using P = std::pair<float, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0.0f, from_node});

    while (!pq.empty())
    {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        for (auto [v, w] : neighbors(u))
            if (dist[u] + w < dist[v]) { dist[v] = dist[u] + w; prev[v] = u; pq.push({dist[v], v}); }
    }

    if (dist[to_node] == std::numeric_limits<float>::infinity())
        return {std::numeric_limits<float>::infinity(), {}}; // unreachable

    std::vector<int> path;
    for (int v = to_node; v != -1 && v != from_node; v = prev[v])
        path.push_back(v);
    std::reverse(path.begin(), path.end());
    return {dist[to_node], path};
}

// =============================================================================
// Unified node dispatch helpers
// =============================================================================

int DeepSkillGraph::_totalNodes() const
{
    return (int)(_skills.size() + _goal_regions.size());
}

bool DeepSkillGraph::_nodeCanStart(int node_idx, const AbstractedState &s) const
{
    if (node_idx < (int)_skills.size())
        return _skills[node_idx]->canStart(s);

    const GoalRegion &gr = _goal_regions[node_idx - (int)_skills.size()];
    float dx = s.position[0] - gr.center.position[0];
    float dy = s.position[1] - gr.center.position[1];
    float dz = s.position[2] - gr.center.position[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz) <= gr.epsilon;
}

float DeepSkillGraph::_nodeDistanceToState(int node_idx, const AbstractedState &s) const
{
    if (node_idx < (int)_skills.size())
        return _skills[node_idx]->distanceToState(s);

    const GoalRegion &gr = _goal_regions[node_idx - (int)_skills.size()];
    float dx = s.position[0] - gr.center.position[0];
    float dy = s.position[1] - gr.center.position[1];
    float dz = s.position[2] - gr.center.position[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

int DeepSkillGraph::_currentNodeIdx() const
{
    auto s = _env->getAbstractedState();
    // deepest skill whose initiation set contains s
    for (int o = (int)_skills.size() - 1; o > _global_option_idx; o--)
        if (_skills[o]->canStart(s)) return o;
    // check goal regions
    for (int r = 0; r < (int)_goal_regions.size(); r++)
        if (_nodeCanStart((int)_skills.size() + r, s)) return (int)_skills.size() + r;
    return _global_option_idx;
}

// =============================================================================
// Graph queries
// =============================================================================

std::vector<int> DeepSkillGraph::_getV(const AbstractedState &s) const
{
    std::vector<int> V;
    // O(s): skill nodes (excluding global) whose initiation set contains s
    for (int o = _global_option_idx + 1; o < (int)_skills.size(); o++)
        if (_skills[o]->canStart(s))
            V.push_back(o);
    // B(s): goal region nodes containing s
    for (int r = 0; r < (int)_goal_regions.size(); r++)
        if (_nodeCanStart((int)_skills.size() + r, s))
            V.push_back((int)_skills.size() + r);
    return V;
}

std::vector<int> DeepSkillGraph::_getReachableDescendants(int node_idx) const
{
    std::unordered_set<int> visited;
    std::queue<int> q;
    visited.insert(node_idx);
    q.push(node_idx);
    while (!q.empty())
    {
        int u = q.front(); q.pop();
        // follow explicit edges
        for (const auto &e : _edges)
            if (e.from == u && !visited.count(e.to)) { visited.insert(e.to); q.push(e.to); }
        // follow implicit parent→child edges for skill nodes
        if (u < (int)_skills.size())
            for (int k = 0; k < (int)_skills.size(); k++)
                for (const auto &child : _skills[u]->children)
                    if (_skills[k] == child && !visited.count(k)) { visited.insert(k); q.push(k); break; }
    }
    return std::vector<int>(visited.begin(), visited.end());
}

std::vector<int> DeepSkillGraph::_getAncestors(int node_idx) const
{
    std::unordered_set<int> visited;
    std::queue<int> q;
    visited.insert(node_idx);
    q.push(node_idx);
    while (!q.empty())
    {
        int u = q.front(); q.pop();
        // reverse explicit edges
        for (const auto &e : _edges)
            if (e.to == u && !visited.count(e.from)) { visited.insert(e.from); q.push(e.from); }
        // reverse implicit child→parent edges for skill nodes
        for (int k = 0; k < (int)_skills.size(); k++)
            for (const auto &child : _skills[k]->children)
                if (child == _skills[u] && !visited.count(k)) { visited.insert(k); q.push(k); break; }
    }
    return std::vector<int>(visited.begin(), visited.end());
}

int DeepSkillGraph::_nearestNodeToState(const AbstractedState &s) const
{
    int   best = 0;
    float best_dist = std::numeric_limits<float>::infinity();
    for (int i = 0; i < _totalNodes(); i++)
    {
        float d = _nodeDistanceToState(i, s);
        if (d < best_dist) { best = i; best_dist = d; }
    }
    return best;
}

int DeepSkillGraph::_closestDisconnectedNode() const
{
    auto s = _env->getAbstractedState();
    int cur = _currentNodeIdx();
    auto reachable = _getReachableDescendants(cur);
    std::unordered_set<int> reachable_set(reachable.begin(), reachable.end());

    int   best = -1;
    float best_dist = std::numeric_limits<float>::infinity();
    for (int i = 0; i < _totalNodes(); i++)
    {
        if (reachable_set.count(i)) continue;
        float d = _nodeDistanceToState(i, s);
        if (d < best_dist) { best = i; best_dist = d; }
    }
    return best;
}

std::pair<int,int> DeepSkillGraph::_closestPair(const std::vector<int> &D,
                                                  const std::vector<int> &A) const
{
    int   best_d = -1, best_a = -1;
    float best_dist = std::numeric_limits<float>::infinity();

    for (int vd : D)
    {
        // representative state for vd: sample from inside its region
        AbstractedState s_vd;
        if (vd < (int)_skills.size())
            s_vd = _skills[vd]->sampleSubgoalState();
        else
            s_vd = _goal_regions[vd - (int)_skills.size()].center;

        for (int va : A)
        {
            float d = _nodeDistanceToState(va, s_vd);
            if (d < best_dist) { best_d = vd; best_a = va; best_dist = d; }
        }
    }
    return {best_d, best_a};
}

// =============================================================================
// Navigation and training primitives
// =============================================================================

void DeepSkillGraph::_navigateTo(int node_idx, int max_steps)
{
    int step = 0;
    while (step < max_steps)
    {
        auto current_state = _env->getAbstractedState();
        if (_nodeCanStart(node_idx, current_state))
            return;

        // V(s_t) = O(s_t) ∪ B(s_t): all graph nodes whose region contains current state
        auto V_s = _getV(current_state);

        // Case (a): find least-cost skill in O(s_t) with a path to node_idx
        int best_option = -1;
        AbstractedState best_goal;
        float best_cost = std::numeric_limits<float>::infinity();

        for (int v : V_s)
        {
            if (v >= (int)_skills.size()) continue; // only skill nodes can be executed, i.e., skip goal regions in V(s_t)
            auto [cost, path] = _dijkstraPath(v, node_idx);
            if (!path.empty() && cost < best_cost)
            {
                best_cost   = cost;
                best_option = v;
                best_goal   = _skills[v]->getLocalGoal();
            }
        }

        // Case (b): no path in G, or V(s_t) contains only goal region nodes
        // — use DSG's own _pickOption()
        if (best_option == -1)
        {
            auto [o, g] = _pickOption();
            best_option = o;
            best_goal   = g;
        }

        auto [steps_taken, cum_reward, done, first_poo, last_poo] =
            _skills[best_option]->rollout(best_goal);

        if (steps_taken == 0) { step++; continue; }

        _poo.addExperience(first_poo, best_option, cum_reward, last_poo, false, steps_taken);
        _poo.learn();
        step += steps_taken;
    }
}

AbstractedState DeepSkillGraph::_runMPC(const AbstractedState &target)
{
    // ── Fallback: no transition model loaded ─────────────────────────────────
    if (!_has_transition_model)
    {
        int steps_remaining = _dsg_cfg.mpc_steps;
        while (steps_remaining > 0)
        {
            auto [steps_taken, _r, _d, _fp, _lp] =
                _skills[_global_option_idx]->rollout(target);
            steps_remaining -= std::max(1, steps_taken);
        }
        return _env->getAbstractedState();
    }

    // ── Real receding-horizon MPC ─────────────────────────────────────────────
    // At each of mpc_steps real environment steps:
    //   1. Observe current full robot state.
    //   2. Run random-shooting MPC (imagined rollouts via transition model) to
    //      find the best action sequence toward `target`.
    //   3. Execute only the first action in the real environment.
    //   4. Stop early on collision / episode termination.
    const double goal_x = target.position[0];
    const double goal_y = target.position[1];
    const auto obstacles = _robot_bridge->getObstacles();

    for (int step = 0; step < _dsg_cfg.mpc_steps; ++step)
    {
        auto [robot_state, in_collision] = _env->getUnderlyingState();
        if (in_collision)
            break;

        RolloutState rs = robotStateToRolloutState(robot_state);

        auto plan = randomShootMPC(
            _transition_model, _normaliser, rs, obstacles, _device,
            goal_x, goal_y,
            _dsg_cfg.mpc_horizon, _dsg_cfg.mpc_candidates, _dsg_cfg.mpc_action_limits,
            _dsg_cfg.mpc_base_radius, _dsg_cfg.mpc_clearance,
            _dsg_cfg.mpc_collision_penalty, _dsg_cfg.mpc_action_penalty,
            _dsg_cfg.mpc_smoothness_penalty, _dsg_cfg.mpc_goal_weight,
            _rng()   // seed derived from DSG's own RNG for reproducibility
        );

        if (plan.actions.empty())
            break;

        // Execute the first action of the best plan in the real environment
        const auto &a = plan.actions.front();
        auto action_tensor = torch::tensor(
            { static_cast<float>(a.vx),
              static_cast<float>(a.vy),
              static_cast<float>(a.yaw) }, torch::kFloat32);

        auto [_ns, _r, done_t] = _env->step(action_tensor);
        if (done_t.item<float>() > 0.5f)
            break;
    }

    return _env->getAbstractedState();
}

void DeepSkillGraph::_trainDSCBridge(int v_a_skill_idx)
{
    // Goal regions cannot serve as termination targets (no SVM initiation set).
    if (v_a_skill_idx >= (int)_skills.size())
    {
        std::cout << "[DSG] _trainDSCBridge: v_a is a goal region, skipping bridge.\n";
        return;
    }

    // Create a new skill whose termination condition is v_a's initiation set.
    _makeSkill(false, _skills[v_a_skill_idx]);
    int bridge_idx = (int)_skills.size() - 1;

    std::cout << "[DSG] Training bridge skill " << bridge_idx
              << " toward skill " << v_a_skill_idx << "\n";

    // Run rollouts until the bridge skill matures or the step budget is exhausted.
    int step = 0;
    while (_skills[bridge_idx]->getTrainingPhase() != "mature" &&
           step < _dsg_cfg.steps_per_episode)
    {
        AbstractedState goal = _skills[bridge_idx]->getLocalGoal();
        auto [steps_taken, cum_reward, done, first_poo, last_poo] =
            _skills[bridge_idx]->rollout(goal);

        if (steps_taken == 0) { step++; continue; }

        _poo.addExperience(first_poo, bridge_idx, cum_reward, last_poo, done, steps_taken);
        _poo.learn();
        step += steps_taken;
    }
}

// =============================================================================
// Phase methods
// =============================================================================

bool DeepSkillGraph::_graphExpansionPhase()
{
    // 1. Sample random reachable state — exploration target - TODO: implement sampling strategy
    AbstractedState s_rand = _env->getRandomValidAbstractedState();

    // 2. Find nearest graph node to s_rand
    int v_nn = _nearestNodeToState(s_rand);

    // 3. Navigate to v_nn using current graph plan / POO fallback
    _navigateTo(v_nn, _dsg_cfg.steps_per_episode / 2);

    // 4. Extend graph: run receding-horizon MPC toward s_rand for mpc_steps real steps.
    //    Uses the learned transition model if loaded; falls back to the global option otherwise.
    AbstractedState s_mpc = _runMPC(s_rand);

    // 5. Rejection sampling: reject if s_mpc is already inside any node in the graph
    // (paper B.1: reject if βo(st)=1 or Io(st)=1 for any o in V)
    for (int o = _global_option_idx + 1; o < (int)_skills.size(); o++)
        if (_skills[o]->canStart(s_mpc))
        {
            std::cout << "[DSG Expansion] Rejected — s_mpc covered by skill " << o << "\n";
            return false;
        }
    for (int r = 0; r < (int)_goal_regions.size(); r++)
        if (_nodeCanStart((int)_skills.size() + r, s_mpc))
        {
            std::cout << "[DSG Expansion] Rejected — s_mpc covered by goal region " << r << "\n";
            return false;
        }

    // 6. Accept: add s_mpc as a new goal region node
    _goal_regions.push_back({s_mpc, _dsg_cfg.goal_region_epsilon});
    std::cout << "[DSG Expansion] GoalRegion " << (_goal_regions.size() - 1)
              << " at (" << s_mpc.position[0] << ", " << s_mpc.position[1] << ")\n";
    return true;
}

void DeepSkillGraph::_graphConsolidationPhase()
{
    // 1. Find closest node not reachable from current state
    int v_g = _closestDisconnectedNode();
    if (v_g == -1)
    {
        std::cout << "[DSG Consolidation] Graph fully connected, skipping.\n";
        return;
    }

    // 2. Identify reachable descendants of current position and ancestors of v_g
    int cur_node = _currentNodeIdx();
    auto D_s  = _getReachableDescendants(cur_node);
    auto A_vg = _getAncestors(v_g);

    // 3. Find closest bridgeable pair (v_d ∈ D_s, v_a ∈ A_vg)
    auto [v_d, v_a] = _closestPair(D_s, A_vg);
    if (v_d == -1 || v_a == -1)
    {
        std::cout << "[DSG Consolidation] No bridgeable pair found.\n";
        return;
    }

    std::cout << "[DSG Consolidation] Bridging " << v_d << " → " << v_a
              << " to reach node " << v_g << "\n";

    // 4. Navigate to v_d
    _navigateTo(v_d, _dsg_cfg.steps_per_episode / 3);

    // 5. Train a DSC bridge from current position toward v_a
    _trainDSCBridge(v_a);

    // 6. Navigate to v_g
    _navigateTo(v_g, _dsg_cfg.steps_per_episode / 3);
}

// =============================================================================
// main
// =============================================================================

#ifndef DSG_BUILD
#error "dsg.cpp must be compiled with -DDSG_BUILD to suppress DSC main"
#endif

#define SCENE_FILE       "../config/scene/umaze_scene_obs_free.xml"
#define OG_ACTOR         "../models/best_actor.pt"
#define OG_CRITIC1       "../models/best_critic_1.pt"
#define OG_CRITIC2       "../models/best_critic_2.pt"
#define DSG_SAVE_PATH    "../dsg_models"
#define TM_CHECKPOINT    "../transition_model/transition_gaussian_model_best.pt"
#define TM_NORMALISER    "../transition_model/normaliser.txt"
#define TEST false

#define X_MIN -7.0f
#define X_MAX  7.0f
#define Y_MIN -7.0f
#define Y_MAX  7.0f

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    torch::Device device(torch::kCPU);
    if (torch::cuda::is_available())
    {
        std::cout << "CUDA available — training on GPU.\n";
        device = torch::Device(torch::kCUDA);
    }
    else if (torch::mps::is_available())
    {
        std::cout << "MPS is available! Training on Apple GPU.\n";
        device = torch::Device(torch::kMPS);
    }

    auto robot_bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, /*render=*/false);

    DeepSkillGraph::Config cfg;
    cfg.gestation_n            = 50;
    cfg.last_k                 = 20;
    cfg.max_option_steps       = 50;
    cfg.nu                     = 0.1;
    cfg.actor_warmup_steps     = 0;
    cfg.warmup_episodes        = 0;
    cfg.verbose                = true;
    cfg.log_interval           = 5;
    cfg.visualize_initiation_sets = true;
    cfg.max_children_per_node  = 3;
    cfg.expansion_freq         = 5;
    cfg.mpc_steps              = 50;
    cfg.goal_region_epsilon    = 1.0f;

    AbstractedState global_goal  = {{-4.5, 4.1, 0}, {0, 0, 0, -1}, {0, 0, 0}, {0, 0, 0}};
    AbstractedState global_start = {{-5.3, -4.5, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    // This initializes policy over options and global option because DSG inherits from DSC
    DeepSkillGraph dsg(robot_bridge, device, global_goal, global_start,
                       OG_ACTOR, OG_CRITIC1, OG_CRITIC2, SCENE_FILE, cfg);

    // Load transition model if checkpoints exist (graceful skip if not yet trained)
    if (std::filesystem::exists(TM_CHECKPOINT) && std::filesystem::exists(TM_NORMALISER))
        dsg.loadTransitionModel(TM_CHECKPOINT, TM_NORMALISER);
    else
        std::cout << "[DSG] No transition model found at " << TM_CHECKPOINT
                  << " — using global-option proxy for graph expansion.\n";

    if (!TEST)
    {
        int n = dsg.train(20000);
        std::cout << "\nTraining complete: " << n << " skill(s) in graph.\n";
        dsg.save(DSG_SAVE_PATH);
    }
    else
    {
        dsg.load(DSG_SAVE_PATH, SCENE_FILE);
    }

    float total = 0.0f;
    robot_bridge->startRender();
    for (int i = 0; i < 40; ++i)
    {
        float r = dsg.execute();
        total += r;
        std::cout << "  Episode " << i + 1 << ": reward = " << r << "\n";
    }
    std::cout << "Mean reward: " << (total / 40.0f) << "\n";

    return 0;
}
