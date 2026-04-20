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

void DeepSkillGraph::_makeSkill(bool is_global, std::shared_ptr<Skill> parent)
{

    AbstractedState global_goal = _nodeRepresentativeState(_current_dsc_problem->v_a);
    DeepSkillChaining::_makeSkill(is_global, parent, global_goal);

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
    DeepSkillChaining::_validateOption();

    for (int i = 0; i < (int)_skills.size(); ++i)
    {
        if (_skills[i]->getTrainingPhase() == "mature")
        {
            // Check if already in graph
            bool in_graph = false;
            for (const auto &node : _nodes)
            {
                if (!node.is_goal_region && node.skill == _skills[i])
                {
                    in_graph = true;
                    break;
                }
            }

            if (!in_graph)
            {
                Node n;
                n.is_goal_region = false;
                n.skill = _skills[i];
                n.id = (int)_nodes.size();
                int new_id = n.id;
                _nodes.push_back(n);

                // --- STRUCTURAL EDGE TO PARENT ---
                auto parent_skill = _skills[i]->getParent();
                if (parent_skill)
                {
                    for (int j = 0; j < (int)_nodes.size(); ++j)
                    {
                        if (!_nodes[j].is_goal_region && _nodes[j].skill == parent_skill)
                        {
                            _ensureStructuralEdge(new_id, j, "parent-link");
                            break;
                        }
                    }
                }

                // Ensure first-chain structural link to v_a 
                auto pending_it = _pending_structural_target_node.find(_skills[i].get());
                if (pending_it != _pending_structural_target_node.end())
                {
                    int target_id = pending_it->second;
                    if (target_id >= 0 && target_id < (int)_nodes.size() && target_id != new_id)
                        _ensureStructuralEdge(new_id, target_id, "first-option-to-v_a");
                    _pending_structural_target_node.erase(pending_it);
                }
                _updateEdges();
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
        _updateEdges();
        std::cout << "[DSG] Seeded graph with Node 0 at (" << _global_start.position[0] << ", " << _global_start.position[1] << ")\n";
    }

    for (int episode = 0; episode < max_episodes; episode++)
    {
        if (episode > 0)
            std::cout << "------\n";

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

        if (_dsg_cfg.graph_update_freq > 0 && (episode + 1) % _dsg_cfg.graph_update_freq == 0)
            _updateEdges();

        if (_dsg_cfg.log_interval > 0 && (episode + 1) % _dsg_cfg.log_interval == 0)
        {
            std::cout << "\n[Episode " << (episode + 1) << "]\n";
            std::cout << "=== In Progress Skill Status ===\n";
            std::cout << "  ID      Phase       GoalHits  Children\n";
            for (size_t i = 0; i < _skills.size(); ++i)
            {

                if (_skills[i]->getTrainingPhase() != "mature")
                {
                    std::string label = (i == (size_t)_global_option_idx)       ? "global"
                                        : (i == (size_t)_global_option_idx + 1) ? "goal"
                                                                                : "opt-" + std::to_string(i);
                    std::string phase = (i == (size_t)_global_option_idx) ? "pre-trained"
                                                                          : _skills[i]->getTrainingPhase();
                    std::cout << "  " << label << "   " << phase;
                    if (phase != "pre-trained")
                        std::cout << "   " << _skills[i]->goalHits() << "/" << _skills[i]->gestationPeriod();
                    std::cout << "  " << _skills[i]->children.size() << "\n";
                }
            }
            std::cout << "\n=== Graph Structure ===\n";

            for (int i = 0; i < _nodes.size(); i++)
            {
                const auto &node = _nodes[i];
                std::string label = node.is_goal_region ? "GR-" + std::to_string(i) : "Opt-" + std::to_string(i);
                std::string phase = node.is_goal_region ? "goal_region" : node.skill->getTrainingPhase();
                std::cout << "  " << label << "   " << phase;
                if (!node.is_goal_region)
                    std::cout << "   " << node.skill->goalHits() << "/" << node.skill->gestationPeriod();
                std::cout << "  children=[";
                for (const auto &child : node.children)
                    std::cout << child.first << "(w=" << child.second << ") ";
                std::cout << "] ";
                if (node.is_goal_region)
                    std::cout << " | center=(x=" << node.goal_region.center.position[0]
                              << ", y=" << node.goal_region.center.position[1]
                              << ") eps=" << node.goal_region.epsilon;
                std::cout << "\n";
            }
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
    const int N = _totalNodes();

    auto is_mature = [&](int idx) -> bool {
        return _nodes[idx].is_goal_region || _nodes[idx].skill->getTrainingPhase() == "mature";
    };

    // Returns true iff effect set of src is fully contained in initiation set of dst.
    // Use pessimistic initiation regions for robust edge creation.
    auto check_link = [&](int src, int dst) -> bool
    {
        if (src == dst) return false; 
        // Skip if edge already exists
        for (const auto &c : _nodes[src].children)
            if (c.first == dst) return false;

        if (_nodes[src].is_goal_region)
        {
            return _nodeCanStart(dst, _nodes[src].goal_region.center, true);
        }
        else
        {
            const auto &effects = _nodes[src].skill->getEffectSet();
            if (effects.empty()) return false;
            for (const auto &rec : effects)
                if (!_nodeCanStart(dst, rec.state, true))
                    return false;
            return true;
        }
    };

    // --- Addition pass: check all ordered pairs (i, j) of mature nodes ---
    for (int i = 0; i < N; ++i)
    {
        if (!is_mature(i)) continue;
        for (int j = 0; j < N; ++j)
        {
            if (!is_mature(j)) continue;
            if (check_link(i, j))
            {
                _nodes[i].children.push_back({j, 1.0f});
                _nodes[j].parents.push_back({i, 1.0f});
                std::cout << "[UpdateEdges] Edge added: " << _nodeLabel(i) << " → " << _nodeLabel(j) << "\n";
            }
        }
    }

    // --- Deletion pass: check all existing edges across all nodes ---
    // Sample K=5 effect states; delete only if >3/5 fail (conservative threshold).
    auto check_stale = [&](int src, int dst) -> bool
    {
        const int K = 5;
        int fail = 0;
        if (_nodes[src].is_goal_region)
        {
            if (!_nodeCanStart(dst, _nodes[src].goal_region.center, false))
                fail = K;
        }
        else
        {
            const auto &effects = _nodes[src].skill->getEffectSet();
            if (effects.empty()) return true;
            for (int k = 0; k < K; ++k)
                if (!_nodeCanStart(dst, effects[rand() % effects.size()].state, false))
                    fail++;
        }
        return fail > 3;
    };

    // for (int i = 0; i < N; ++i)
    // {
    //     auto &ch = _nodes[i].children;
    //     for (int ci = (int)ch.size() - 1; ci >= 0; --ci)
    //     {
    //         int j = ch[ci].first;
    //         if (check_stale(i, j))
    //         {
    //             std::cout << "[UpdateEdges] Edge removed: " << _nodeLabel(i) << " → " << _nodeLabel(j) << "\n";
    //             auto &par = _nodes[j].parents;
    //             par.erase(std::remove_if(par.begin(), par.end(),
    //                 [i](const auto &p){ return p.first == i; }), par.end());
    //             ch.erase(ch.begin() + ci);
    //         }
    //     }
    // }
}

void DeepSkillGraph::_ensureStructuralEdge(int from, int to, const std::string &reason)
{
    if (from < 0 || to < 0 || from >= (int)_nodes.size() || to >= (int)_nodes.size() || from == to)
        return;

    bool exists = false;
    for (const auto &c : _nodes[from].children)
    {
        if (c.first == to)
        {
            exists = true;
            break;
        }
    }
    if (exists)
        return;

    _nodes[from].children.push_back({to, 1.0f});
    _nodes[to].parents.push_back({from, 1.0f});
    std::cout << "[Structural] Edge added (" << reason << "): "
              << _nodeLabel(from) << " → " << _nodeLabel(to) << "\n";
}

void DeepSkillGraph::_updateEdgeWeight(int from, int to, bool success)
{
    const float factor = success ? _dsg_cfg.edge_weight_kappa : (1.0f / _dsg_cfg.edge_weight_kappa);

    // Update forward list
    for (auto &child : _nodes[from].children)
    {
        if (child.first == to)
        {
            child.second *= factor;
            break;
        }
    }

    // Update backward list
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

        for (auto [v, w] : _nodes[u].children) // TODO: ignore goal regions for cost calculation?
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

bool DeepSkillGraph::_nodeCanStart(int node_idx, const AbstractedState &s, bool pessimistic) const
{
    const auto &n = _nodes[node_idx];
    if (!n.is_goal_region)
        return (pessimistic) ? n.skill->canStartPessimistic(s) : n.skill->canStart(s);

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
        if (_nodeCanStart(i, s, false))
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
        for (const auto &p : _nodes[u].parents)
        {
            if (visited.insert(p.first).second)
                q.push(p.first);
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
        if (reachable_set.count(i))
            continue;

        if (!_nodes[i].is_goal_region && _nodes[i].skill->getTrainingPhase() != "mature")
            continue;

        float d = _nodeDistanceToState(i, s);
        if (d < best_dist)
        {
            best = i;
            best_dist = d;
        }
    }
    return best;
}

std::pair<int, int> DeepSkillGraph::_closestPair(const std::vector<int> &D,
                                                 const std::vector<int> &A) const
{
    int best_d = -1, best_a = -1;
    float best_dist = std::numeric_limits<float>::infinity();

    if (_cfg.verbose)
    {
        std::cout << "[ClosestPair] Evaluating candidates: |D|=" << D.size()
                  << " |A|=" << A.size() << "\n";
    }

    for (int vd : D)
    {
        AbstractedState s_vd = _nodeRepresentativeState(vd);

        for (int va : A)
        {
            // NEW: Skip if they are the same node. There is no gap to bridge here.
            if (vd == va)
                continue;

            float d = _nodeDistanceToState(va, s_vd);

            if (_cfg.verbose)
            {
                std::cout << "[ClosestPair] candidate vd=" << _nodeLabel(vd)
                          << " va=" << _nodeLabel(va)
                          << " dist=" << d << "\n";
            }

            if (d < best_dist)
            {
                best_d = vd;
                best_a = va;
                best_dist = d;
            }
        }
    }

    if (_cfg.verbose)
    {
        if (best_d == -1 || best_a == -1)
        {
            std::cout << "[ClosestPair] no valid pair selected\n";
        }
        else
        {
            std::cout << "[ClosestPair] selected vd=" << _nodeLabel(best_d)
                      << " va=" << _nodeLabel(best_a)
                      << " dist=" << best_dist << "\n";
        }
    }
    return {best_d, best_a};
}

// =============================================================================
// Navigation and training primitives
// =============================================================================
void DeepSkillGraph::_navigateTo(int node_idx, int max_steps)
{
    auto pathToString = [&](const std::vector<int> &path, int from) -> std::string
    {
        std::string out = _nodeLabel(from);
        for (int v : path)
        {
            out += " -> ";
            out += _nodeLabel(v);
        }
        return out;
    };

    auto start_state = _env->getAbstractedState();
    std::cout << "[Navigate] Start navigateTo from="
              << start_state.position[0] << ", " << start_state.position[1]
              << " target=" << _nodeLabel(node_idx)
              << " max_steps=" << max_steps << "\n";

    int step = 0;
    while (step < max_steps)
    {
        auto current_state = _env->getAbstractedState();
        bool env_done = _env->computeReward(current_state).first.data_ptr<float>()[0] > 0.5;
        bool already_in_target = _nodeCanStart(node_idx, current_state, false);
        if (already_in_target || env_done)
        {
            if (already_in_target)
            {
                std::cout << "[Navigate] Already in target " << _nodeLabel(node_idx) << "\n";
            }
            else
            {
                std::cout << "[Navigate] Reached point "
                          << current_state.position[0] << ", " << current_state.position[1] << "\n";
            }
            return;
        }

        // V(s_t): all nodes containing the current state
        auto V_s = _getV(current_state);

        int best_node_idx = -1;
        int next_hop_idx = -1;
        AbstractedState best_goal;
        float best_cost = std::numeric_limits<float>::infinity();
        std::vector<int> best_path;

        // Case (a): find least-cost path to node_idx starting from a skill node in V_s
        for (int v : V_s)
        {
            // if (_nodes[v].is_goal_region)
            //     continue; // Only skills have policies we can execute

            auto [cost, path] = _dijkstraPath(v, node_idx);
            if (!path.empty() && cost < best_cost)
            {
                best_cost = cost;
                best_node_idx = v;
                next_hop_idx = path.front();
                best_goal = _nodeRepresentativeState(next_hop_idx);
                best_path = path;
            }
        }

        // Case (b): Recovery — use Global Option to steer toward target if no graph path exists
        if (best_node_idx == -1)
        {
            best_node_idx = _global_option_idx; // Use the base class global skill index
            best_goal = _nodeRepresentativeState(node_idx);

            std::cout << "[Navigate] No path found, using global option\n";
            std::cout << "[Navigate] No graph path from current state. "
                      << "Executing GlobalOpt toward " << _nodeLabel(node_idx) << "\n";

            auto [steps_taken, cum_reward, done, first_poo, last_poo] = // reak if done?
                _skills[_global_option_idx]->rollout(best_goal);

            step += steps_taken;
            if (steps_taken == 0)
            {
                return;
            }
            if (_cfg.verbose)
                std::cout << "[DSG Navigation] No path found. Using Global Option recovery toward "
                          << _nodeLabel(node_idx) << "\n";
        }
        else
        {
            std::cout << "[Navigate] Path selected: "
                      << pathToString(best_path, best_node_idx)
                      << " | execute=" << _nodeLabel(best_node_idx)
                      << " -> next_hop=" << _nodeLabel(next_hop_idx) << "\n";

            // Execute the chosen skill
            // Access the skill object directly from the Node to avoid index-mismatch bugs
            if (_nodes[best_node_idx].is_goal_region) continue;
            auto [steps_taken, cum_reward, done, first_poo, last_poo] =
                _nodes[best_node_idx].skill->rollout(best_goal);

            if (steps_taken == 0)
            {
                return;
            }
            step += steps_taken;
        }

        // Update weights if we were following an explicit graph edge
        if (next_hop_idx != -1)
        {
            bool success = _nodeCanStart(next_hop_idx, _env->getAbstractedState(), false);
            _updateEdgeWeight(best_node_idx, next_hop_idx, success);
        }
    }

    auto final_state = _env->getAbstractedState();
    std::cout << "[Navigate] Stopped at step budget with state=("
              << final_state.position[0] << ", " << final_state.position[1] << ")\n";
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
    std::cout << "\n[DSG Expansion] Starting graph expansion phase...\n";
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
            if (_nodeCanStart(i, s_mpc, false))
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
    int new_node_id = n.id;
    _nodes.push_back(n);

    _updateEdges();

    std::cout << "[DSG Expansion] Added " << _nodeLabel(new_node_id)
              << " linked from " << _nodeLabel(v_nn) << "\n";
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

            std::shared_ptr<Skill> parent = nullptr;

            if (_current_dsc_problem->dsc_chain.empty())
            {
                int root_node_id = _current_dsc_problem->v_a;
                if (!_nodes[root_node_id].is_goal_region)
                {
                    parent = _nodes[root_node_id].skill;
                }
            }
            else
            {
                int last_skill_idx = _current_dsc_problem->dsc_chain.back();
                parent = _skills[last_skill_idx];
            }

            _makeSkill(false, parent);
            _current_dsc_problem->dsc_chain.push_back(_unfinished_option_idx);
        }

        auto [option, goal] = _pickOption();
        std::cout << "Option: " << option << " Goal: (" << goal.position[0] << ", " << goal.position[1] << ")\n";

        if (eng->render_m)
            eng->setGoalMarker(goal.position[0], goal.position[1], 0.5f);

        auto [steps_taken, cum_reward, local_done, first_state_poo, last_state_poo] = _skills[option]->rollout(goal);

        auto [g_reward, g_done] = _env->computeReward(rollout_goal);
        env_done = g_done.data_ptr<float>()[0] > 0.5f;

        if (steps_taken == 0) // this condition occurs when we just finished training a new skill, but then find ourselves in the initiation set of that skill while trying to train the new skill
        {
            break;
        }

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
    }
    return total_reward;
}

void DeepSkillGraph::visualizeInitiationSets()
{
    std::vector<std::vector<std::array<float, 3>>> points_per_skill;
    int max_points = 500;
    for (const auto &skill : _skills)
    {
        std::vector<std::array<float, 3>> points;
        int point = 0;
        for (const auto &record : skill->getPositiveGestationRecords())
        {
            if (point >= max_points)
                break;
            point++;
            points.push_back(record.state.position);
        }
        points_per_skill.push_back(points);
    }

    std::string temp_file = "/tmp/init_sets.txt";
    std::ofstream out(temp_file);

    int skill_idx = 0;
    for (const auto &points : points_per_skill)
    {
        for (const auto &p : points)
        {
            out << skill_idx << " " << p[0] << " " << p[1] << "\n";
        }
        skill_idx++;
    }

    // Goal-region overlay entries for visualize.py:
    // GR <id> <x> <y> <eps>
    for (const auto &node : _nodes)
    {
        if (!node.is_goal_region)
            continue;
        out << "GR " << node.id << " "
            << node.goal_region.center.position[0] << " "
            << node.goal_region.center.position[1] << " "
            << node.goal_region.epsilon << "\n";
    }
    out.close();

    std::string cmd = "python ../../visualize.py " + _scene_file_path + " " + temp_file;
    system(cmd.c_str());
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

void DeepSkillGraph::_graphConsolidationPhase()
{
    std::cout << "\n[DSG Consolidation] Starting graph consolidation phase...\n";
    auto labelsToString = [&](const std::vector<int> &ids) -> std::string
    {
        if (ids.empty())
            return "[]";
        std::string out = "[";
        for (size_t i = 0; i < ids.size(); ++i)
        {
            out += _nodeLabel(ids[i]);
            if (i + 1 < ids.size())
                out += ", ";
        }
        out += "]";
        return out;
    };

    bool solved_current_problem = false;
    if (_current_dsc_problem != nullptr)
    {
        solved_current_problem = _containsStart(_current_dsc_problem->v_d, _current_dsc_problem->dsc_chain);
        if (solved_current_problem)
        {
            int v_d = _current_dsc_problem->v_d;
            AbstractedState vd_state = _nodeRepresentativeState(v_d);

            int bridge_skill_idx = -1;
            for (int o : _current_dsc_problem->dsc_chain)
            {
                if (_skills[o]->getTrainingPhase() == "mature" && _skills[o]->canStart(vd_state))
                {
                    bridge_skill_idx = o;
                    break;
                }
            }

            if (bridge_skill_idx != -1)
            {
                for (int j = 0; j < (int)_nodes.size(); ++j)
                {
                    if (!_nodes[j].is_goal_region && _nodes[j].skill == _skills[bridge_skill_idx])
                    {
                        _ensureStructuralEdge(v_d, j, "solve-v_d-to-chain");
                        break;
                    }
                }
            }
        }
    }

    bool can_start_new_problem = _current_dsc_problem == nullptr || solved_current_problem;

    if (can_start_new_problem)
    {
        // Refresh graph connectivity before vertex selection so v_g, D(s_t), and A(v_g)
        // are computed from current initiation/effect set relationships.
        _updateEdges();

        // 1. Find closest node not reachable from current state
        auto current_state = _env->getAbstractedState();
        auto V_s = _getV(current_state);
        int v_0 = _nearestNodeToState(current_state);

        int v_g = _closestDisconnectedNode();
        if (v_g == -1)
        {
            std::cout << "[DSG Consolidation] Graph fully connected, skipping.\n";
            _updateEdges();
            return;
        }

        // 2. D(s_t): descendants of all nodes in V(s_t); A(v_g): ancestors of the target node
        auto D_s = _getDSt(current_state);
        auto A_vg = _getAncestors(v_g);

        std::cout << "[VertexSelect] v0=" << _nodeLabel(v_0)
                  << " at=(" << current_state.position[0] << ", " << current_state.position[1] << ")\n";
        std::cout << "[VertexSelect] V(s_t)=" << labelsToString(V_s) << "\n";
        std::cout << "[VertexSelect] chosen v_g=" << _nodeLabel(v_g) << "\n";
        std::cout << "[VertexSelect] D(s_t)=" << labelsToString(D_s) << "\n";
        std::cout << "[VertexSelect] A(v_g)=" << labelsToString(A_vg) << "\n";

        // 3. Find closest bridgeable pair (v_d ∈ D_s, v_a ∈ A_vg)
        auto [v_d, v_a] = _closestPair(D_s, A_vg);
        if (v_d == -1 || v_a == -1)
        {
            std::cout << "[DSG Consolidation] No bridgeable pair found.\n";
            _updateEdges();
            return;
        }
        std::cout << "[VertexSelect] selected v_d=" << _nodeLabel(v_d)
                  << " v_a=" << _nodeLabel(v_a) << " v_g=" << _nodeLabel(v_g) << "\n";
        if (!_current_dsc_problem)
            _current_dsc_problem = std::make_unique<DSCProblem>();

        _current_dsc_problem->v_a = v_a;
        _current_dsc_problem->v_d = v_d;
        _current_dsc_problem->v_g = v_g;
        _current_dsc_problem->dsc_chain = {};

        std::shared_ptr<Skill> parent = nullptr;

        if (_current_dsc_problem->dsc_chain.empty())
        {
            int root_node_id = _current_dsc_problem->v_a;
            if (!_nodes[root_node_id].is_goal_region)
            {
                parent = _nodes[root_node_id].skill;
            }
        }
        else
        {
            int last_skill_idx = _current_dsc_problem->dsc_chain.back();
            parent = _skills[last_skill_idx];
        }

        _makeSkill(false, parent);
        // Always ensure first chain option links structurally to v_a (GR or option).
        _pending_structural_target_node[_skills[_unfinished_option_idx].get()] = v_a;
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
    if (_env->getUnderlyingState().second)
    {
        _updateEdges();
        return;
    }

    auto rollout_start = _env->getAbstractedState();
    std::cout << "[Consolidation] starting dsc rollout from "
              << rollout_start.position[0] << ", " << rollout_start.position[1] << "\n";

    float reward = _dscRollout();
    _validateOption();
    // Explicit edge refresh after each consolidation rollout/validation step.
    _updateEdges();
    if (_env->getUnderlyingState().second)
        return;

    // end consolidation episode when dscRollout() terminates
    // 6. Navigate to v_g
    // _navigateTo(_current_dsc_problem->v_g, _dsg_cfg.steps_per_episode / 3);
}

// =============================================================================
// main
// =============================================================================

#ifndef DSG_BUILD
#error "dsg.cpp must be compiled with -DDSG_BUILD to suppress DSC main"
#endif

#define SCENE_FILE "../config/scene/test_scene.xml"
#define OG_ACTOR "../models/best_actor.pt"
#define OG_CRITIC1 "../models/best_critic_1.pt"
#define OG_CRITIC2 "../models/best_critic_2.pt"
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
    cfg.gestation_n = 60;
    cfg.last_k = 15;
    cfg.max_option_steps = 30;
    cfg.nu = 0.01;
    cfg.actor_warmup_steps = 0;
    cfg.warmup_episodes = 0; // use to warm up policy over options with global option rollouts before starting expansion/consolidation
    cfg.verbose = true;
    cfg.log_interval = 25;
    cfg.visualize_initiation_sets = true;
    cfg.max_children_per_node = 3;
    cfg.expansion_freq = 100; // frequency of expansion phase (every N episodes)
    cfg.mpc_steps = 50;
    cfg.goal_region_epsilon = 0.1f;
    cfg.success_radius = 0.1f;
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
