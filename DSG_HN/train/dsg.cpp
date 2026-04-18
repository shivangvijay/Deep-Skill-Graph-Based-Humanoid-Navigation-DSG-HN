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

    // Extract yaw from quaternion [qw, qx, qy, qz]
    const double qw = rs.orientation[0], qx = rs.orientation[1];
    const double qy = rs.orientation[2], qz = rs.orientation[3];
    out.yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz));

    out.vx = rs.velocity[0];
    out.vy = rs.velocity[1];
    out.oz = rs.angular_velocity[2];

    out.joint_pos.assign(rs.q.begin(), rs.q.end());
    out.joint_vel.assign(rs.dq.begin(), rs.dq.end());
    return out;
}

// Project a RolloutState back to the navigation-level AbstractedState used by DSG.
// Orientation is recovered as a yaw-only quaternion (pitch = roll = 0).
static AbstractedState rolloutStateToAbstractedState(const RolloutState &rs)
{
    AbstractedState as;
    // RolloutState has no z; use 0 (navigation planning is 2-D)
    as.position = {static_cast<float>(rs.x),
                   static_cast<float>(rs.y),
                   0.0f};
    const float qw = static_cast<float>(std::cos(rs.yaw * 0.5));
    const float qz = static_cast<float>(std::sin(rs.yaw * 0.5));
    as.orientation = {qw, 0.0f, 0.0f, qz};
    as.velocity = {static_cast<float>(rs.vx),
                   static_cast<float>(rs.vy), 0.0f};
    as.angular_velocity = {0.0f, 0.0f, static_cast<float>(rs.oz)};
    return as;
}

// =============================================================================
// Transition model loading
// =============================================================================

void DeepSkillGraph::loadTransitionModel(const std::string &model_path,
                                         const std::string &normaliser_path)
{
    MpcConfig mpc_cfg;
    mpc_cfg.horizon = _dsg_cfg.mpc_horizon;
    mpc_cfg.candidates = _dsg_cfg.mpc_candidates;
    mpc_cfg.cem_rounds = _dsg_cfg.mpc_cem_rounds;
    mpc_cfg.cem_elites = _dsg_cfg.mpc_cem_elites;
    mpc_cfg.w_pos = _dsg_cfg.mpc_w_pos;
    mpc_cfg.w_heading = _dsg_cfg.mpc_w_heading;
    mpc_cfg.w_terminal = _dsg_cfg.mpc_w_terminal;
    mpc_cfg.w_smooth = _dsg_cfg.mpc_w_smooth;
    mpc_cfg.w_backward = _dsg_cfg.mpc_w_backward;
    mpc_cfg.w_collision = _dsg_cfg.mpc_w_collision;
    mpc_cfg.base_radius = _dsg_cfg.mpc_base_radius;
    mpc_cfg.clearance = _dsg_cfg.mpc_clearance;

    // Transformer architecture: seq_len=10, d_model=128, n_heads=4, n_layers=4
    // (must match the checkpoint produced by farnaz/transition training script)
    _mpc_ctx = mpc_create(model_path, normaliser_path,
                          /*history=*/10, /*d_model=*/128, /*n_heads=*/4, /*n_layers=*/4,
                          mpc_cfg);

    std::cout << "[DSG] Transformer transition model loaded from " << model_path << "\n";
}

// =============================================================================
// DSC overrides
// =============================================================================

AbstractedState DeepSkillGraph::_sampleSpawnState()
{
    // Case 1: _global_start is inside any Node — sample uniformly within its epsilon-ball/initiation set
    for (int i = 0; i < (int)_nodes.size(); ++i)
    {
        if (_nodeCanStart(i, _global_start))
        {
            float epsilon = _nodes[i].is_goal_region ? _nodes[i].goal_region.epsilon : 0.5f;
            std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * M_PI);
            std::uniform_real_distribution<float> radius_dist(0.0f, epsilon);
            float angle = angle_dist(_rng);
            float radius = radius_dist(_rng);

            AbstractedState spawn = _nodeRepresentativeState(i);
            spawn.position[0] += radius * std::cos(angle);
            spawn.position[1] += radius * std::sin(angle);
            return spawn;
        }
    }

    // Case 2: Fallback — Small XY perturbation near global start
    AbstractedState spawn = _global_start;
    std::normal_distribution<float> gauss(0.0f, 0.3f);
    spawn.position[0] += gauss(_rng);
    spawn.position[1] += gauss(_rng);
    return spawn;
}

void DeepSkillGraph::_makeSkill(bool is_global, std::shared_ptr<Skill> parent)
{

    AbstractedState target = _nodeRepresentativeState(_current_dsc_problem->v_a);
    DeepSkillChaining::_makeSkill(is_global, parent, target);

    if (!_dsg_cfg.save_path.empty())
    {
        std::filesystem::create_directories(_dsg_cfg.save_path);
        save(_dsg_cfg.save_path);
    }
}

bool DeepSkillGraph::_shouldCreateNewOption(int v_d, const std::vector<int> &dsc_chain)
{
    for (auto o : dsc_chain)
        if (_skills[o]->getTrainingPhase() != "mature")
            return false;

    if (_containsStart(v_d, dsc_chain))
        return false;

    return true;
}

bool DeepSkillGraph::_containsStart(int v_d, const std::vector<int> &dsc_chain)
{
    // for (auto o : dsc_chain)
    //     if (_skills[o]->getTrainingPhase() != "mature")
    //         return false;

    for (auto o : dsc_chain)
    {
        int can_start_count = 0;
        for (int i = 0; i < _dsg_cfg.gestation_n; i++)
            if (_skills[o]->canStart(_nodeRepresentativeState(v_d)) && _skills[o]->getTrainingPhase() == "mature")
                can_start_count++;

        if (static_cast<float>(can_start_count) / static_cast<float>(_dsg_cfg.gestation_n) > 0.5f)
            return true;
    }
    return false;
}

void DeepSkillGraph::_validateOption()
{
    // 1. Run the base DSC validation (gestation -> validation -> mature)
    DeepSkillChaining::_validateOption();

    // 2. Scan for skills that are now "mature" but not yet in our graph
    for (int i = 0; i < (int)_skills.size(); ++i)
    {
        if (_skills[i]->getTrainingPhase() == "mature")
        {
            // Check if this skill is already represented in _nodes
            bool in_graph = false;
            for (const auto& node : _nodes) {
                if (!node.is_goal_region && node.skill == _skills[i]) {
                    in_graph = true;
                    break;
                }
            }

            if (!in_graph)
            {
                // ADD MATURE SKILL TO GRAPH
                Node n;
                n.is_goal_region = false;
                n.skill = _skills[i];
                n.id = (int)_nodes.size();
                _nodes.push_back(n);

                std::cout << "[DSG] Skill promoted to Node " << n.id 
                          << " (Opt-" << i << " is now Mature)\n";

                // 3. Immediately update edges to connect this new node to the graph
                _updateEdges();
                
                // 4. Save the new structural state
                if (!_dsg_cfg.save_path.empty()) {
                    std::filesystem::create_directories(_dsg_cfg.save_path);
                    save(_dsg_cfg.save_path);
                }
            }
        }
    }
}

float DeepSkillGraph::execute()
{
    _env->resetTo(_global_start);

    // Target the most recently added node (the current frontier)
    if (_nodes.empty())
        return -1.0f;
    int target_node = (int)_nodes.size() - 1;

    _navigateTo(target_node, _dsg_cfg.steps_per_episode);

    auto s = _env->getAbstractedState();
    return -_nodeDistanceToState(target_node, s);
}

void DeepSkillGraph::_warmupRollout()
{
    // In DSG there is no fixed global goal — roll the global option toward a random valid state.
    AbstractedState target = _env->getRandomValidAbstractedState();
    _skills[_global_option_idx]->rollout(target);
}

// =============================================================================
// Main training loop
// =============================================================================

int DeepSkillGraph::train(int max_episodes)
{
    if (_dsg_cfg.render_training)
        _robot_bridge->startRender();

    // Seed the graph with the start state as the first GoalRegion (G = (V, E, W)).
    // This makes _getV(s_t) return a non-empty set when the robot is at s_0, so
    // D(s_t) is non-empty from episode 0 and consolidation can find (v_d, v_a) pairs.
    if (_nodes.empty())
    {
        Node n;
        n.is_goal_region = true;
        n.goal_region = {_global_start, _dsg_cfg.goal_region_epsilon};
        n.id = 0;
        _nodes.push_back(n);
        std::cout << "[DSG] Seeded graph with Node 0 at (" << _global_start.position[0] << ", " << _global_start.position[1] << ")\n";
    }

    for (int episode = 0; episode < max_episodes; episode++)
    {
        _env->resetTo(_global_start); // this can either be a fixed position or come from a small set of states.

        // Per-episode header: episode, phase, and current graph size
        {
            std::string phase_label;
            if (episode < _dsg_cfg.warmup_episodes)
                phase_label = "warmup";
            else if (episode % _dsg_cfg.expansion_freq == 0)
                phase_label = "expansion";
            else
                phase_label = "consolidation";

            // std::cout << "[Ep " << (episode + 1) << "/" << max_episodes
            //           << " | " << phase_label
            //           << " | V=" << (_skills.size() + _goal_regions.size())
            //           << " (S=" << _skills.size() << " GR=" << _goal_regions.size() << ")"
            //   << " E=" << _edges.size() << "]\n";
        }

        // Determine phase
        if (episode < _dsg_cfg.warmup_episodes)
        {
            _warmupRollout(); // this only warms up the polict over options by rolling out the global option - maybe remove?
        }
        else
        {
            if (episode % _dsg_cfg.expansion_freq == 0) // expand every _dsg_cfg.expansion_freq episodes, otherwise consolidate
            {
                // first episode will be expansion
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
                std::string label = (i == (size_t)_global_option_idx)       ? "global"
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
            // std::cout << "  GoalRegions: " << _goal_regions.size()
            //           << "  Edges: " << _edges.size() << "\n";
            if (_dsg_cfg.visualize_initiation_sets)
                visualizeInitiationSets();
        }
    }
    return (int)_skills.size() - 1;
}

// =============================================================================
// save / load
// =============================================================================

void DeepSkillGraph::save(const std::string &dir) const
{
    DeepSkillChaining::save(dir); // Saves policy weights
    std::ofstream f(dir + "/graph_structure.txt");
    f << _nodes.size() << "\n";
    for (const auto &n : _nodes)
    {
        f << n.id << " " << n.is_goal_region << " " << n.children.size() << " " << n.parents.size() << "\n";
        // Edges
        for (auto &c : n.children)
            f << c.first << " " << c.second << " ";
        f << "\n";
        for (auto &p : n.parents)
            f << p.first << " " << p.second << " ";
        f << "\n";
        // Goal Region Data
        if (n.is_goal_region)
        {
            f << n.goal_region.epsilon << " ";
            for (float v : n.goal_region.center.position)
                f << v << " ";
            for (float v : n.goal_region.center.orientation)
                f << v << " ";
            for (float v : n.goal_region.center.velocity)
                f << v << " ";
            for (float v : n.goal_region.center.angular_velocity)
                f << v << " ";
        }
        f << "\n";
    }
}

void DeepSkillGraph::load(const std::string &dir, const std::string &scene_file)
{
    DeepSkillChaining::load(dir, scene_file);
    _nodes.clear();
    std::ifstream f(dir + "/graph_structure.txt");
    int n_size;
    f >> n_size;

    // Skill index tracker to map _skills back to nodes
    int skill_ptr = 0;

    for (int i = 0; i < n_size; i++)
    {
        Node n;
        int n_children, n_parents;
        f >> n.id >> n.is_goal_region >> n_children >> n_parents;
        // Load Edges
        for (int j = 0; j < n_children; j++)
        {
            int id;
            float w;
            f >> id >> w;
            n.children.push_back({id, w});
        }
        for (int j = 0; j < n_parents; j++)
        {
            int id;
            float w;
            f >> id >> w;
            n.parents.push_back({id, w});
        }

        if (n.is_goal_region)
        {
            f >> n.goal_region.epsilon;
            for (float &v : n.goal_region.center.position)
                f >> v;
            for (float &v : n.goal_region.center.orientation)
                f >> v;
            for (float &v : n.goal_region.center.velocity)
                f >> v;
            for (float &v : n.goal_region.center.angular_velocity)
                f >> v;
        }
        else
        {
            n.skill = _skills[skill_ptr++];
        }
        _nodes.push_back(n);
    }
}

// =============================================================================
// Graph edge management
// =============================================================================

void DeepSkillGraph::_updateEdges()
{
    // Clear both to prevent stale/duplicate entries
    for (auto &node : _nodes)
    {
        node.children.clear();
        node.parents.clear();
    }

    for (int i = 0; i < _totalNodes(); ++i)
    {
        if (_nodes[i].is_goal_region || _nodes[i].skill->getTrainingPhase() != "mature")
            continue;

        const auto &effect_set = _nodes[i].skill->getEffectSet();

        for (int j = 0; j < _totalNodes(); ++j)
        {
            if (i == j)
                continue;

            bool all_covered = true;
            for (const auto &rec : effect_set)
            {
                if (!_nodeCanStart(j, rec.state))
                {
                    all_covered = false;
                    break;
                }
            }

            if (all_covered)
            {
                float initial_w = 1.0f;
                _nodes[i].children.push_back({j, initial_w});
                _nodes[j].parents.push_back({i, initial_w});
            }
        }
    }
}

void DeepSkillGraph::_updateEdgeWeight(int from, int to, bool success)
{
    // Calculate the multiplicative factor: w *= kappa (success) or w *= 1/kappa (failure)
    const float factor = success ? _dsg_cfg.edge_weight_kappa : (1.0f / _dsg_cfg.edge_weight_kappa);

    // 1. Update the weight in the 'from' node's children list
    for (auto &child : _nodes[from].children)
    {
        if (child.first == to)
        {
            child.second *= factor;
            break;
        }
    }

    // 2. Update the weight in the 'to' node's parents list
    for (auto &parent : _nodes[to].parents)
    {
        if (parent.first == from)
        {
            parent.second *= factor;
            break;
        }
    }
}

std::pair<float, std::vector<int>> DeepSkillGraph::_dijkstraPath(int from, int to) const
{
    int N = _totalNodes();
    std::vector<float> dist(N, std::numeric_limits<float>::infinity());
    std::vector<int> prev(N, -1);
    dist[from] = 0.0f;

    using P = std::pair<float, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0.0f, from});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u])
            continue;
        if (u == to)
            break;

        for (auto [v, w] : _nodes[u].children)
        {
            if (dist[u] + w < dist[v])
            {
                dist[v] = dist[u] + w;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    std::vector<int> path;
    if (dist[to] == std::numeric_limits<float>::infinity())
        return {dist[to], path};
    for (int v = to; v != -1 && v != from; v = prev[v])
        path.push_back(v);
    std::reverse(path.begin(), path.end());
    return {dist[to], path};
}

// void DeepSkillGraph::_makeSkill(bool is_global, std::shared_ptr<Skill> parent)
// {
//     DeepSkillChaining::_makeSkill(is_global, parent);
//     Node n;
//     n.is_goal_region = false;
//     n.skill = _skills.back();
//     n.id = (int)_nodes.size(); // Stable ID matches index in unified list
//     _nodes.push_back(n);
// }

// =============================================================================
// Unified node dispatch helpers
// =============================================================================

int DeepSkillGraph::_totalNodes() const
{
    return (int)_nodes.size();
}

bool DeepSkillGraph::_nodeCanStart(int node_idx, const AbstractedState &s) const
{
    const auto &n = _nodes[node_idx];
    if (!n.is_goal_region)
        return n.skill->canStart(s);

    // Goal region check
    float dx = s.position[0] - n.goal_region.center.position[0];
    float dy = s.position[1] - n.goal_region.center.position[1];
    return std::sqrt(dx * dx + dy * dy) <= n.goal_region.epsilon;
}

float DeepSkillGraph::_nodeDistanceToState(int node_idx, const AbstractedState &s) const
{
    const auto &n = _nodes[node_idx];
    if (!n.is_goal_region)
        return n.skill->distanceToState(s);

    float dx = s.position[0] - n.goal_region.center.position[0];
    float dy = s.position[1] - n.goal_region.center.position[1];
    return std::sqrt(dx * dx + dy * dy);
}

std::string DeepSkillGraph::_nodeLabel(int node_idx) const
{
    if (node_idx < 0 || node_idx >= (int)_nodes.size())
        return "none";
    const auto &n = _nodes[node_idx];
    if (n.is_goal_region)
        return "GR-" + std::to_string(n.id);
    return (n.id == _global_option_idx) ? "GlobalOpt" : "Opt-" + std::to_string(n.id);
}

// =============================================================================
// Graph queries
// =============================================================================

std::vector<int> DeepSkillGraph::_getV(const AbstractedState &s) const
{
    std::vector<int> V;
    for (int i = 0; i < (int)_nodes.size(); ++i)
    {
        if (_nodeCanStart(i, s))
        {
            // Logic check: Only include mature skills or Goal Regions
            if (_nodes[i].is_goal_region || _nodes[i].skill->getTrainingPhase() == "mature")
            {
                V.push_back(i);
            }
        }
    }
    return V;
}

std::vector<int> DeepSkillGraph::_getDSt(const AbstractedState &s) const
{
    auto V_s = _getV(s);
    std::unordered_set<int> D_set;
    for (int v : V_s)
    {
        auto desc = _getReachableDescendants(v);
        D_set.insert(desc.begin(), desc.end());
    }
    return std::vector<int>(D_set.begin(), D_set.end());
}

std::vector<int> DeepSkillGraph::_getReachableDescendants(int node_idx) const
{
    std::unordered_set<int> visited;
    std::queue<int> q;
    visited.insert(node_idx);
    q.push(node_idx);

    while (!q.empty())
    {
        int u = q.front();
        q.pop();

        for (const auto &c : _nodes[u].children)
        {
            if (visited.insert(c.first).second)
                q.push(c.first);
        }

        // Implicit DSC parent link
        if (!_nodes[u].is_goal_region && _nodes[u].skill->getParent())
        {
            auto p_skill = _nodes[u].skill->getParent();
            for (int i = 0; i < _totalNodes(); ++i)
            {
                if (!_nodes[i].is_goal_region && _nodes[i].skill == p_skill)
                {
                    if (visited.insert(i).second)
                        q.push(i);
                    break;
                }
            }
        }
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
        int u = q.front();
        q.pop();

        // 1. Follow explicit parent edges
        for (const auto &p : _nodes[u].parents)
        {
            if (visited.insert(p.first).second)
                q.push(p.first);
        }

        // 2. Follow implicit child -> parent edges (reverse logic)
        // If node 'i' is a child of node 'u' in DSC, then 'u' is 'i's ancestor.
        for (int i = 0; i < _totalNodes(); ++i)
        {
            if (!visited.count(i) && !_nodes[i].is_goal_region &&
                _nodes[i].skill->getParent() == _nodes[u].skill)
            {
                visited.insert(i);
                q.push(i);
            }
        }
    }
    return std::vector<int>(visited.begin(), visited.end());
}

int DeepSkillGraph::_nearestNodeToState(const AbstractedState &s) const
{
    int best = 0;
    float best_dist = std::numeric_limits<float>::infinity();
    for (int i = 0; i < _totalNodes(); i++)
    {
        float d = _nodeDistanceToState(i, s);
        if (d < best_dist)
        {
            best = i;
            best_dist = d;
        }
    }
    return best;
}

int DeepSkillGraph::_closestDisconnectedNode() const
{
    auto s = _env->getAbstractedState();
    // Use the BFS-based descendant set
    auto reachable_indices = _getDSt(s); 
    std::unordered_set<int> reachable_set(reachable_indices.begin(), reachable_indices.end());

    int best = -1;
    float best_dist = std::numeric_limits<float>::infinity();

    for (int i = 0; i < (int)_nodes.size(); i++)
    {
        // If the BFS says we can reach it, it is NOT disconnected!
        if (reachable_set.count(i)) continue;

        if (!_nodes[i].is_goal_region && _nodes[i].skill->getTrainingPhase() != "mature")
            continue;

        float d = _nodeDistanceToState(i, s);
        if (d < best_dist) { best = i; best_dist = d; }
    }
    return best;
}

std::pair<int, int> DeepSkillGraph::_closestPair(const std::vector<int> &D,
                                                 const std::vector<int> &A) const
{
    int best_d = -1, best_a = -1;
    float best_dist = std::numeric_limits<float>::infinity();

    for (int vd : D)
    {
        AbstractedState s_vd = _nodeRepresentativeState(vd);

        for (int va : A)
        {
            // NEW: Skip if they are the same node. There is no gap to bridge here.
            if (vd == va) continue;

            float d = _nodeDistanceToState(va, s_vd);

            if (d < best_dist)
            {
                best_d = vd;
                best_a = va;
                best_dist = d;
            }
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

        // V(s_t): all nodes containing the current state
        auto V_s = _getV(current_state);

        int best_node_idx = -1;
        int next_hop_idx = -1;
        AbstractedState best_goal;
        float best_cost = std::numeric_limits<float>::infinity();

        // Case (a): find least-cost path to node_idx starting from a skill node in V_s
        for (int v : V_s)
        {
            if (_nodes[v].is_goal_region)
                continue; // Only skills have policies we can execute

            auto [cost, path] = _dijkstraPath(v, node_idx);
            if (!path.empty() && cost < best_cost)
            {
                best_cost = cost;
                best_node_idx = v;
                next_hop_idx = path.front();
                best_goal = _nodeRepresentativeState(next_hop_idx);
            }
        }

        // Case (b): Recovery — use Global Option to steer toward target if no graph path exists
        if (best_node_idx == -1)
        {
            best_node_idx = _global_option_idx; // Use the base class global skill index
            best_goal = _nodeRepresentativeState(node_idx);

            auto [steps_taken, cum_reward, done, first_poo, last_poo] = // reak if done?
                _skills[_global_option_idx]->rollout(best_goal);

            step += steps_taken;
            if (steps_taken == 0)
            {
                step++;
                continue;
            }
            if (_cfg.verbose)
                std::cout << "[DSG Navigation] No path found. Using Global Option recovery toward "
                          << _nodeLabel(node_idx) << "\n";
        }
        else
        {
            // Execute the chosen skill
            // Access the skill object directly from the Node to avoid index-mismatch bugs
            auto [steps_taken, cum_reward, done, first_poo, last_poo] =
                _nodes[best_node_idx].skill->rollout(best_goal);

            if (steps_taken == 0)
            {
                step++;
                continue;
            }
            step += steps_taken;
        }

        // Update weights if we were following an explicit graph edge
        if (next_hop_idx != -1)
        {
            bool success = _nodeCanStart(next_hop_idx, _env->getAbstractedState());
            _updateEdgeWeight(best_node_idx, next_hop_idx, success);
        }
    }
}

AbstractedState DeepSkillGraph::_runMPC(const AbstractedState &target)
{
    // ── Fallback: no transition model loaded, use global option ──────────────
    if (!_mpc_ctx || true)
    {
        std::cout << "[MPC Fallback] No model — global option proxy for "
                  << _dsg_cfg.mpc_steps << " steps toward ("
                  << target.position[0] << ", " << target.position[1] << ")\n";

        int steps_remaining = _dsg_cfg.mpc_steps;
        while (steps_remaining > 0)
        {
            auto [steps_taken, _r, _d, _fp, _lp] =
                _skills[_global_option_idx]->rollout(target);
            steps_remaining -= std::max(1, steps_taken);
        }

        auto s_reached = _env->getAbstractedState();
        float dx = s_reached.position[0] - target.position[0];
        float dy = s_reached.position[1] - target.position[1];
        std::cout << "[MPC Fallback] Reached (" << s_reached.position[0] << ", "
                  << s_reached.position[1] << ") dist=" << std::sqrt(dx * dx + dy * dy) << "\n";
        return s_reached;
    }

    // ── Transformer+CEM receding-horizon MPC ─────────────────────────────────
    // Clear history: we don't have context from the preceding _navigateTo() call.
    mpc_clear_history(*_mpc_ctx);
    // Obstacles are static within one expansion step — set once up front.
    mpc_set_obstacles(*_mpc_ctx, _robot_bridge->getObstacles());

    const double goal_x = target.position[0];
    const double goal_y = target.position[1];

    std::cout << "[MPC] target=(" << goal_x << ", " << goal_y << ")"
              << " horizon=" << _dsg_cfg.mpc_horizon
              << " candidates=" << _dsg_cfg.mpc_candidates
              << " steps=" << _dsg_cfg.mpc_steps << "\n";

    for (int step = 0; step < _dsg_cfg.mpc_steps; ++step)
    {
        auto [robot_state, in_collision] = _env->getUnderlyingState();
        if (in_collision)
        {
            std::cout << "[MPC] step " << step << " — early stop (collision)\n";
            break;
        }

        RolloutState rs = robotStateToRolloutState(robot_state);

        // CEM plan: returns the first action of the best sequence
        ActionCmd cmd = mpc_plan(*_mpc_ctx, rs, goal_x, goal_y, _rng());

        if (_cfg.verbose)
        {
            float dx = static_cast<float>(rs.x - goal_x);
            float dy = static_cast<float>(rs.y - goal_y);
            std::cout << "[MPC] step " << (step + 1) << "/" << _dsg_cfg.mpc_steps
                      << " pos=(" << rs.x << ", " << rs.y << ")"
                      << " dist=" << std::sqrt(dx * dx + dy * dy)
                      << " act=[" << cmd.vx << ", " << cmd.vy << ", " << cmd.yaw << "]\n";
        }

        auto action_tensor = torch::tensor(
            {static_cast<float>(cmd.vx),
             static_cast<float>(cmd.vy),
             static_cast<float>(cmd.yaw)},
            torch::kFloat32);

        auto [_ns, _r, done_t] = _env->step(action_tensor);

        // Update Transformer history with the (state, action) pair just executed
        mpc_update_history(*_mpc_ctx, rs, cmd);

        if (done_t.item<float>() > 0.5f)
            break;
    }

    auto s_reached = _env->getAbstractedState();
    float dx = s_reached.position[0] - static_cast<float>(goal_x);
    float dy = s_reached.position[1] - static_cast<float>(goal_y);
    std::cout << "[MPC] reached (" << s_reached.position[0] << ", " << s_reached.position[1]
              << ") dist=" << std::sqrt(dx * dx + dy * dy) << "\n";
    return s_reached;
}

AbstractedState DeepSkillGraph::_nodeRepresentativeState(int node_idx) const
{
    const auto &n = _nodes[node_idx];
    if (n.is_goal_region)
        return n.goal_region.center;
    if (n.skill->getTrainingPhase() == "mature")
        return n.skill->sampleSubgoalState();
    return _env->getRandomValidAbstractedState();
}

// =============================================================================
// Phase methods
// =============================================================================

bool DeepSkillGraph::_graphExpansionPhase()
{
    // 1. Sample random reachable state (exploration target)
    AbstractedState s_rand = _env->getRandomValidAbstractedState();

    // 2. Find nearest node to s_rand within D(s_t)
    // D(s_t) provides a list of unified indices reachable from current state
    auto D_st = _getDSt(_env->getAbstractedState());
    int v_nn = -1;
    float best_dist = std::numeric_limits<float>::infinity();

    for (int v : D_st)
    {
        float d = _nodeDistanceToState(v, s_rand);
        if (d < best_dist)
        {
            best_dist = d;
            v_nn = v;
        }
    }

    // Fallback: Use globally nearest node if D(s_t) is empty or unreachable
    if (v_nn == -1)
        v_nn = _nearestNodeToState(s_rand);

    std::cout << "[DSG Expansion] Target: (" << s_rand.position[0] << ", " << s_rand.position[1]
              << ") | Nearest Node: " << _nodeLabel(v_nn) << "\n";

    // 3. Navigate to v_nn
    _navigateTo(v_nn, _dsg_cfg.steps_per_episode / 2);

    // 4. Extend graph via MPC toward s_rand
    AbstractedState s_mpc = _runMPC(s_rand);

    // 5. Rejection sampling: Is s_mpc already covered by an existing node?
    for (int i = 0; i < (int)_nodes.size(); ++i)
    {
        // Only reject if s_mpc falls into a Goal Region or a MATURE skill.
        // We skip gestating skills because their classifiers aren't trustworthy yet.
        if (_nodes[i].is_goal_region || _nodes[i].skill->getTrainingPhase() == "mature")
        {
            if (_nodeCanStart(i, s_mpc))
            {
                if (_cfg.verbose)
                    std::cout << "[DSG Expansion] Rejected — s_mpc covered by " << _nodeLabel(i) << "\n";
                return false;
            }
        }
    }

    // 6. Acceptance: Add s_mpc as a new Goal Region Node
    Node n;
    n.is_goal_region = true;
    n.goal_region = {s_mpc, _dsg_cfg.goal_region_epsilon};
    n.id = (int)_nodes.size();

    _nodes.push_back(n);

    std::cout << "[DSG Expansion] Success! Added " << _nodeLabel(n.id)
              << " at (" << s_mpc.position[0] << ", " << s_mpc.position[1] << ")\n";

    return true;
}

float DeepSkillGraph::_dscRollout()
{
    int step = 0;
    bool env_done = false;
    float total_reward = 0;

    auto eng = _robot_bridge->getEngine();

    auto rollout_goal = _nodeRepresentativeState(_current_dsc_problem->v_a);

    while (step < _dsg_cfg.steps_per_episode && !env_done)
    {
        auto [option, goal] = _pickOption();

        if (eng->render_m)
            eng->setGoalMarker(goal.position[0], goal.position[1], 0.5f);

        auto [steps_taken, cum_reward, local_done, first_state_poo, last_state_poo] = _skills[option]->rollout(goal);

        auto [g_reward, g_done] = _env->computeReward(rollout_goal);
        env_done = g_done.data_ptr<float>()[0] > 0.5f;

        if (steps_taken == 0) // this condition occurs when we just finished training a new skill, but then find ourselves in the initiation set of that skill while trying to train the new skill
            break;

        step += steps_taken;
        total_reward += cum_reward;

        // float clipped_reward = std::clamp(cum_reward, -100.0f, 100.0f); // clipping reward since sometimes not terminating makes the reward spike and want to limit that effect.
        if (!_eval) // only train poo during training, not evaluation
        {
            _poo.addExperience(first_state_poo, option, cum_reward, last_state_poo, env_done, steps_taken);
            _poo.learn();
        }

        if (_cfg.verbose)
            std::cout << "  [Rollout] option=" << option
                      << " phase=" << _skills[option]->getTrainingPhase()
                      << " steps=" << steps_taken
                      << " reward=" << cum_reward
                      << " local_done=" << local_done
                      << " global_done=" << env_done << "\n";

        // make a new skill if we have finished training the current option, but still have not reached the end goal
        if (_shouldCreateNewOption(_current_dsc_problem->v_d, _current_dsc_problem->dsc_chain) && !_eval)
        {
            std::cout << "\n[DSC] Finished training option " << _unfinished_option_idx << "\n";
            float total_dist = 0;
            for (int i = 0; i < _cfg.gestation_n; i++)
            {
                AbstractedState sample = _skills[_unfinished_option_idx]->sampleSubgoalState();
                float dx = sample.position[0] - _global_start.position[0];
                float dy = sample.position[1] - _global_start.position[1];
                float dz = sample.position[2] - _global_start.position[2];
                total_dist += std::sqrt(dx * dx + dy * dy + dz * dz);
            }
            std::cout << "\n[Option " << _unfinished_option_idx << " matured] Average distance of initiation region to start: " << (total_dist / _cfg.gestation_n) << " m\n";

            std::shared_ptr<Skill> parent;
            if (_current_dsc_problem->v_a >= (int)_skills.size())
            {
                parent = nullptr;
                std::cout << "  No parent skill (root-level option)\n";
            }
            else
            {
                parent = _skills[_current_dsc_problem->v_a];
                std::cout << "  Parent: " << _nodeLabel(_current_dsc_problem->v_a) << "\n";
            }
            _makeSkill(false, parent);
            _current_dsc_problem->dsc_chain.push_back(_unfinished_option_idx);
        }
    }
    return total_reward;
}

std::pair<int, AbstractedState> DeepSkillGraph::_pickOption()
{
    auto goal = _nodeRepresentativeState(_current_dsc_problem->v_a);
    auto poo_state = _env->getStateRelativeToGoal(goal);
    auto global_state = _env->getAbstractedState();

    torch::Tensor q_vals = _poo.getOptions(poo_state);

    int best_option = _global_option_idx;
    float best_q_val = std::numeric_limits<float>::lowest();

    // Collect valid options split by pessimistic availability
    std::vector<int> pessimistic_options;
    std::vector<int> optimistic_options;

    for (auto o : _current_dsc_problem->dsc_chain)
    {
        if (_skills[o]->canStart(global_state) && !_skills[o]->atTermination(goal))
        {
            if (_skills[o]->canStartPessimistic(global_state))
            {
                pessimistic_options.push_back(o);
            }
            else
            {
                optimistic_options.push_back(o);
            }
        }
    }

    // Pick best from pessimistic, fallback to optimistic
    const auto &candidates = pessimistic_options.empty() ? optimistic_options : pessimistic_options;

    // empirically found that just picking the earliest valid option is more consistent
    // than picking based on Q values
    best_option = candidates.empty() ? _global_option_idx : candidates[0];

    if (best_option == _global_option_idx)
    {
        // TODO: pick closest option as goal
        float min_dist = std::numeric_limits<float>::infinity();
        int closest_option = -1;
        for (auto o : _current_dsc_problem->dsc_chain)
        {
            float dist = _skills[o]->distanceToState(global_state);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_option = o;
            }
        }
        if (closest_option != -1)
        {
            return {_global_option_idx, _skills[closest_option]->sampleSubgoalState()};
        }
        else
        {
            return {_global_option_idx, _global_goal};
        }
    }
    else
    {
        return {best_option, _skills[best_option]->getLocalGoal()};
    }
}
// issue: only one option training at a time, so when we swap nodes we are training then we can no longer guarantee consistency with the one that
// is being trained
// fix for now: do not let DSC pick
void DeepSkillGraph::_graphConsolidationPhase()
{
    bool can_start_new_problem = _current_dsc_problem == nullptr || _containsStart(_current_dsc_problem->v_d, _current_dsc_problem->dsc_chain);

    if (can_start_new_problem)
    {
        // 1. Find closest node not reachable from current state
        int v_g = _closestDisconnectedNode();
        if (v_g == -1)
        {
            std::cout << "[DSG Consolidation] Graph fully connected, skipping.\n";
            return;
        }

        // 2. D(s_t): descendants of all nodes in V(s_t); A(v_g): ancestors of the target node
        auto D_s = _getDSt(_env->getAbstractedState());
        auto A_vg = _getAncestors(v_g);

        // 3. Find closest bridgeable pair (v_d ∈ D_s, v_a ∈ A_vg)
        auto [v_d, v_a] = _closestPair(D_s, A_vg);
        if (v_d == -1 || v_a == -1)
        {
            std::cout << "[DSG Consolidation] No bridgeable pair found.\n";
            return;
        }
        if (!_current_dsc_problem)
            _current_dsc_problem = std::make_unique<DSCProblem>();

        _current_dsc_problem->v_a = v_a;
        _current_dsc_problem->v_d = v_d;
        _current_dsc_problem->v_g = v_g;
        _current_dsc_problem->dsc_chain = {};

        std::shared_ptr<Skill> parent;
        if (_current_dsc_problem->v_a >= (int)_skills.size())
        {
            parent = nullptr;
            std::cout << "  No parent skill (root-level option)\n";
        }
        else
        {
            parent = _skills[_current_dsc_problem->v_a];
            std::cout << "  Parent: " << _nodeLabel(_current_dsc_problem->v_a) << "\n";
        }
        _makeSkill(false, parent);
        std::cout << _unfinished_option_idx << " is the new option being trained to bridge " << _nodeLabel(v_d) << " to " << _nodeLabel(v_g) << "\n";
        _current_dsc_problem->dsc_chain.push_back(_unfinished_option_idx);

        std::cout << "[DSG Consolidation] New problem: " << _nodeLabel(v_d) << " → " << _nodeLabel(v_a) << " → " << _nodeLabel(v_g) << "\n";
    }

    std::cout << "[DSG Consolidation] Bridging " << _nodeLabel(_current_dsc_problem->v_d)
              << " → " << _nodeLabel(_current_dsc_problem->v_a)
              << " to reach " << _nodeLabel(_current_dsc_problem->v_g) << "\n";

    AbstractedState v_a_state = _nodeRepresentativeState(_current_dsc_problem->v_a);

    // 4. Navigate to v_d
    _navigateTo(_current_dsc_problem->v_d, _dsg_cfg.steps_per_episode / 3);

    float reward = _dscRollout();
    _validateOption();

    // 6. Navigate to v_g
    _navigateTo(_current_dsc_problem->v_g, _dsg_cfg.steps_per_episode / 3);
}

// =============================================================================
// main
// =============================================================================

#ifndef DSG_BUILD
#error "dsg.cpp must be compiled with -DDSG_BUILD to suppress DSC main"
#endif

#define SCENE_FILE "../config/scene/test_scene.xml"
#define OG_ACTOR "../models/actor.pt"
#define OG_CRITIC1 "../models/critic_1.pt"
#define OG_CRITIC2 "../models/critic_2.pt"
#define DSG_SAVE_PATH "../dsg_models"
#define TM_CHECKPOINT "../checkpoints/improved/transition_transformer_delta_latest.pt"
#define TM_NORMALISER "../checkpoints/improved/normaliser.txt"
#define TEST false

#define X_MIN -7.0f
#define X_MAX 7.0f
#define Y_MIN -7.0f
#define Y_MAX 7.0f

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
    cfg.gestation_n = 30;
    cfg.last_k = 15;
    cfg.max_option_steps = 30;
    cfg.nu = 0.01;
    cfg.actor_warmup_steps = 0;
    cfg.warmup_episodes = 0; // use to warm up policy over options with global option rollouts before starting expansion/consolidation
    cfg.verbose = true;
    cfg.log_interval = 5;
    cfg.visualize_initiation_sets = true;
    cfg.max_children_per_node = 3;
    cfg.expansion_freq = 100; // frequency of expansion phase (every N episodes)
    cfg.mpc_steps = 50;
    cfg.goal_region_epsilon = 1.0f;
    cfg.save_path = DSG_SAVE_PATH;
    cfg.training_episodes = 20000;

    AbstractedState global_start = {{0.0, 0.0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    // DSG has no fixed global goal — the graph grows outward from global_start
    DeepSkillGraph dsg(robot_bridge, device, global_start,
                       OG_ACTOR, OG_CRITIC1, OG_CRITIC2, SCENE_FILE, cfg);

    // Load transition model if checkpoints exist (graceful skip if not yet trained)
    if (std::filesystem::exists(TM_CHECKPOINT) && std::filesystem::exists(TM_NORMALISER))
        dsg.loadTransitionModel(TM_CHECKPOINT, TM_NORMALISER);
    else
        std::cout << "[DSG] No transition model found at " << TM_CHECKPOINT
                  << " — using global-option proxy for graph expansion.\n";

    if (!TEST)
    {
        int n = dsg.train(cfg.training_episodes); // cfg.training_episodes default = 20000
        std::cout << "\nTraining complete: " << n << " skill(s) in graph.\n";
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
