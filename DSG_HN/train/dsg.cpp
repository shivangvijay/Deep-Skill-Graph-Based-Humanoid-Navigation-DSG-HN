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

    out.joint_pos.assign(rs.q.begin(),  rs.q.end());
    out.joint_vel.assign(rs.dq.begin(), rs.dq.end());
    return out;
}

// Project a RolloutState back to the navigation-level AbstractedState used by DSG.
// Orientation is recovered as a yaw-only quaternion (pitch = roll = 0).
static AbstractedState rolloutStateToAbstractedState(const RolloutState &rs)
{
    AbstractedState as;
    // RolloutState has no z; use 0 (navigation planning is 2-D)
    as.position    = { static_cast<float>(rs.x),
                       static_cast<float>(rs.y),
                       0.0f };
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
    MpcConfig mpc_cfg;
    mpc_cfg.horizon    = _dsg_cfg.mpc_horizon;
    mpc_cfg.candidates = _dsg_cfg.mpc_candidates;
    mpc_cfg.cem_rounds = _dsg_cfg.mpc_cem_rounds;
    mpc_cfg.cem_elites = _dsg_cfg.mpc_cem_elites;
    mpc_cfg.w_pos       = _dsg_cfg.mpc_w_pos;
    mpc_cfg.w_heading   = _dsg_cfg.mpc_w_heading;
    mpc_cfg.w_terminal  = _dsg_cfg.mpc_w_terminal;
    mpc_cfg.w_smooth    = _dsg_cfg.mpc_w_smooth;
    mpc_cfg.w_backward  = _dsg_cfg.mpc_w_backward;
    mpc_cfg.w_collision = _dsg_cfg.mpc_w_collision;
    mpc_cfg.base_radius = _dsg_cfg.mpc_base_radius;
    mpc_cfg.clearance   = _dsg_cfg.mpc_clearance;

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
    int n_skills = (int)_skills.size();

    // Case 1: _global_start is inside a goal region — sample uniformly within its epsilon-ball.
    for (int r = 0; r < (int)_goal_regions.size(); r++)
    {
        if (_nodeCanStart(n_skills + r, _global_start))
        {
            const auto &gr = _goal_regions[r];
            std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * M_PI);
            std::uniform_real_distribution<float> radius_dist(0.0f, gr.epsilon);
            float angle = angle_dist(_rng);
            float radius = radius_dist(_rng);
            AbstractedState spawn = gr.center;
            spawn.position[0] += radius * std::cos(angle);
            spawn.position[1] += radius * std::sin(angle);
            return spawn;
        }
    }

    // Case 2: _global_start is the physical landing state after executing v_d (its effect set).
    // Small XY perturbation gives diversity across episodes while staying within the effect set.
    AbstractedState spawn = _global_start;
    std::normal_distribution<float> gauss(0.0f, 0.3f);
    spawn.position[0] += gauss(_rng);
    spawn.position[1] += gauss(_rng);
    return spawn;
}

void DeepSkillGraph::_makeSkill(bool is_global, std::shared_ptr<Skill> parent)
{
    DeepSkillChaining::_makeSkill(is_global, parent);
    if (parent)
    {
        parent->children.push_back(_skills.back());
        // A skill just matured and triggered backward chaining — save all artifacts.
        if (!_dsg_cfg.save_path.empty())
        {
            std::filesystem::create_directories(_dsg_cfg.save_path);
            save(_dsg_cfg.save_path);
            std::cout << "[DSG] Saved after Option " << (_skills.size() - 1) << " created → " << _dsg_cfg.save_path << "\n";
        }
    }
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



float DeepSkillGraph::execute()
{
    _env->resetTo(_global_start);

    // Navigate toward the most recently added goal region (current frontier).
    // Falls back to the first chained skill node if no goal regions exist yet.
    int target_node = (_goal_regions.empty())
        ? _global_option_idx + 1
        : (int)_skills.size() + (int)_goal_regions.size() - 1;

    _navigateTo(target_node, _dsg_cfg.steps_per_episode);

    // Return negative distance to target as a proxy reward
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
    if (_goal_regions.empty())
    {
        _goal_regions.push_back({_global_start, _dsg_cfg.goal_region_epsilon});
        std::cout << "[DSG] Seeded graph with start state as GoalRegion 0 at ("
                  << _global_start.position[0] << ", " << _global_start.position[1] << ")\n";
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

            std::cout << "[Ep " << (episode + 1) << "/" << max_episodes
                      << " | " << phase_label
                      << " | V=" << (_skills.size() + _goal_regions.size())
                      << " (S=" << _skills.size() << " GR=" << _goal_regions.size() << ")"
                      << " E=" << _edges.size() << "]\n";
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

    // skill → skill/goal_region edges: effect_set(i) ⊆ initiation_set(j)
    // E_i = _effect_records: states where skill i actually triggered its termination condition.
    // Stored separately from _positive_gestation_records (which also contains spawn states used
    // for initiation set classifier training). Only termination states participate in edge inference.
    for (int i = _global_option_idx + 1; i < n_skills; i++)
    {
        if (_skills[i]->getTrainingPhase() != "mature") continue;
        const auto &effect_set = _skills[i]->getEffectSet();
        if (effect_set.empty()) continue;

        for (int j = _global_option_idx + 1; j < n_skills; j++)
        {
            if (i == j || edge_exists(i, j)) continue;
            bool all_covered = true;
            for (const auto &rec : effect_set)
                if (!_skills[j]->canStartPessimistic(rec.state)) { all_covered = false; break; }
            if (all_covered)
            {
                _edges.push_back({i, j, 1.0f});
                std::cout << "[DSG] Edge " << _nodeLabel(i) << " → " << _nodeLabel(j) << "\n";
            }
        }

        // skill → goal_region edges
        for (int r = 0; r < n_gr; r++)
        {
            int gr_idx = n_skills + r;
            if (edge_exists(i, gr_idx)) continue;
            bool all_covered = true;
            for (const auto &rec : effect_set)
                if (!_nodeCanStart(gr_idx, rec.state)) { all_covered = false; break; }
            if (all_covered)
            {
                _edges.push_back({i, gr_idx, 1.0f});
                std::cout << "[DSG] Edge " << _nodeLabel(i) << " → " << _nodeLabel(gr_idx) << "\n";
            }
        }
    }
}

void DeepSkillGraph::_updateEdgeWeight(int from, int to, bool success)
{
    const float kappa = _dsg_cfg.edge_weight_kappa;
    for (auto &e : _edges)
    {
        if (e.from == from && e.to == to)
        {
            e.weight *= success ? kappa : (1.0f / kappa);
            return;
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
                    if (_skills[k] == child) { nbrs.push_back({k, 1.0f}); break; }
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

std::string DeepSkillGraph::_nodeLabel(int node_idx) const
{
    if (node_idx < 0) return "none";
    if (node_idx < (int)_skills.size())
        return (node_idx == _global_option_idx) ? "GlobalOption"
             : "Option " + std::to_string(node_idx - _global_option_idx);
    return "GoalRegion " + std::to_string(node_idx - (int)_skills.size());
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
    // O(s): mature skill nodes (excluding global) whose initiation set contains s.
    // Gestating skills have canStart()=true everywhere; including them would pollute
    // the navigation graph with untrained options that steer toward _global_goal=zeros.
    for (int o = _global_option_idx + 1; o < (int)_skills.size(); o++)
        if (_skills[o]->getTrainingPhase() == "mature" && _skills[o]->canStart(s))
            V.push_back(o);
    // B(s): goal region nodes containing s
    for (int r = 0; r < (int)_goal_regions.size(); r++)
        if (_nodeCanStart((int)_skills.size() + r, s))
            V.push_back((int)_skills.size() + r);
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
    // Use D(s_t): descendants of all nodes in V(s_t), not just the single current node
    auto reachable = _getDSt(s);
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
        int next_node   = -1; // first node on the Dijkstra path after best_option
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
                next_node   = path.front();
                best_goal   = _nodeRepresentativeState(next_node); // steer toward next hop, not GlobalOption's random subgoal
            }
        }

        // Case (b): no graph path from V(s_t) to node_idx (degenerate — node_idx should be in D(s_t))
        // Steer the global option directly toward the target node as a recovery primitive.
        if (best_option == -1)
        {
            best_option = _global_option_idx;
            best_goal   = (node_idx < (int)_skills.size())
                ? _skills[node_idx]->sampleSubgoalState()
                : _goal_regions[node_idx - (int)_skills.size()].center; // steer global option directly toward the target node as a recovery primitive
        }

        // The paper replans at every new state; training happens in the dedicated training phases.
        // bool prev_eval = _eval;
        // setEvalMode(true);
        auto [steps_taken, cum_reward, done, first_poo, last_poo] =
            _skills[best_option]->rollout(best_goal);
        // setEvalMode(prev_eval);

        if (steps_taken == 0) { step++; continue; }

        // Update edge weight based on whether the agent entered the next node (case a only).
        if (next_node != -1)
        {
            bool success = _nodeCanStart(next_node, _env->getAbstractedState());
            _updateEdgeWeight(best_option, next_node, success);
        }
        
        // update poo - not needed, can remove
        _poo.addExperience(first_poo, best_option, cum_reward, last_poo, false, steps_taken);
        _poo.learn();
        step += steps_taken;
    }
}

AbstractedState DeepSkillGraph::_runMPC(const AbstractedState &target)
{
    // ── Fallback: no transition model loaded, use global option ──────────────
    if (!_mpc_ctx)
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
                  << s_reached.position[1] << ") dist=" << std::sqrt(dx*dx + dy*dy) << "\n";
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
                      << " dist=" << std::sqrt(dx*dx + dy*dy)
                      << " act=[" << cmd.vx << ", " << cmd.vy << ", " << cmd.yaw << "]\n";
        }

        auto action_tensor = torch::tensor(
            { static_cast<float>(cmd.vx),
              static_cast<float>(cmd.vy),
              static_cast<float>(cmd.yaw) }, torch::kFloat32);

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
              << ") dist=" << std::sqrt(dx*dx + dy*dy) << "\n";
    return s_reached;
}

AbstractedState DeepSkillGraph::_nodeRepresentativeState(int node_idx) const
{
    if (node_idx >= (int)_skills.size())
        return _goal_regions[node_idx - (int)_skills.size()].center;
    if (_skills[node_idx]->getTrainingPhase() == "mature")
        return _skills[node_idx]->sampleSubgoalState();
    return _env->getRandomValidAbstractedState();
}


// =============================================================================
// Phase methods
// =============================================================================

bool DeepSkillGraph::_graphExpansionPhase()
{
    // 1. Sample random reachable state — exploration target - TODO: implement sampling strategy
    AbstractedState s_rand = _env->getRandomValidAbstractedState();

    // 2. Find nearest node to s_rand within D(s_t) — guarantees a path exists from current state
    auto D_st = _getDSt(_env->getAbstractedState());
    int v_nn = -1;
    {
        float best_dist = std::numeric_limits<float>::infinity();
        for (int v : D_st)
        {
            float d = _nodeDistanceToState(v, s_rand);
            if (d < best_dist) { best_dist = d; v_nn = v; }
        }
    }
    // Fallback: graph not yet reachable from current state — use globally nearest node
    if (v_nn == -1)
        v_nn = _nearestNodeToState(s_rand);

    std::cout << "[DSG Expansion] s_rand=(" << s_rand.position[0] << ", " << s_rand.position[1]
              << ") v_nn=" << _nodeLabel(v_nn) << "\n";

    // 3. Navigate to v_nn using current graph plan / POO fallback
    _navigateTo(v_nn, _dsg_cfg.steps_per_episode / 2);

    // 4. Extend graph: run receding-horizon MPC toward s_rand for mpc_steps real steps.
    //    Uses the learned transition model if loaded; falls back to the global option otherwise.
    AbstractedState s_mpc = _runMPC(s_rand);

    // 5. Rejection sampling: reject if s_mpc is already inside any node in the graph
    // Only mature skills have trained classifiers; gestating skills return canStart()=true
    // everywhere so must be excluded from this check.
    for (int o = _global_option_idx + 1; o < (int)_skills.size(); o++)
        if (_skills[o]->getTrainingPhase() == "mature" && _skills[o]->canStart(s_mpc))
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

    // 2. D(s_t): descendants of all nodes in V(s_t); A(v_g): ancestors of the target node
    auto D_s  = _getDSt(_env->getAbstractedState());
    auto A_vg = _getAncestors(v_g);

    // 3. Find closest bridgeable pair (v_d ∈ D_s, v_a ∈ A_vg)
    auto [v_d, v_a] = _closestPair(D_s, A_vg);
    if (v_d == -1 || v_a == -1)
    {
        std::cout << "[DSG Consolidation] No bridgeable pair found.\n";
        return;
    }

    std::cout << "[DSG Consolidation] Bridging " << _nodeLabel(v_d)
              << " → " << _nodeLabel(v_a)
              << " to reach " << _nodeLabel(v_g) << "\n";

    // 4. Navigate to v_d
    _navigateTo(v_d, _dsg_cfg.steps_per_episode / 3);

    // 4b. If v_d is a skill node, execute it to land in its effect set.
    //     The robot's physical state after execution is the correct spawn anchor for the
    //     bridge DSC chain: the new skill's initiation set must cover these landing states
    //     so that E_{v_d} ⊆ I_{new} holds and a valid graph edge can be inferred.
    if (v_d < (int)_skills.size() && v_d != _global_option_idx)
        _skills[v_d]->rollout(_skills[v_d]->getLocalGoal());

    // 5. Train a DSC bridge from v_d toward v_a.
    {
        AbstractedState v_d_state = _env->getAbstractedState(); // inside v_d's effect set
        AbstractedState v_a_state = _nodeRepresentativeState(v_a);

        std::cout << "[DSG Consolidation] Bridge " << _nodeLabel(v_d)
                  << " → " << _nodeLabel(v_a)
                  << " start=(" << v_d_state.position[0] << ", " << v_d_state.position[1] << ")"
                  << " goal=(" << v_a_state.position[0] << ", " << v_a_state.position[1] << ")\n";

        auto saved_start   = _global_start;
        auto saved_goal    = _global_goal;
        bool saved_strict  = _cfg.strict_sampling;
        _global_start      = v_d_state;
        _global_goal       = v_a_state;
        _cfg.strict_sampling = true; // always spawn near v_d_state; boundary heuristic is meaningless here

        DeepSkillChaining::train(1);

        _global_start        = saved_start;
        _global_goal         = saved_goal;
        _cfg.strict_sampling = saved_strict;
    }

    // 6. Navigate to v_g
    _navigateTo(v_g, _dsg_cfg.steps_per_episode / 3);
}

// =============================================================================
// main
// =============================================================================

#ifndef DSG_BUILD
#error "dsg.cpp must be compiled with -DDSG_BUILD to suppress DSC main"
#endif

#define SCENE_FILE       "../config/scene/test_scene.xml"
#define OG_ACTOR         "../models/best_actor.pt"
#define OG_CRITIC1       "../models/best_critic_1.pt"
#define OG_CRITIC2       "../models/best_critic_2.pt"
#define DSG_SAVE_PATH    "../dsg_models"
#define TM_CHECKPOINT    "../checkpoints/improved/transition_transformer_delta_latest.pt"
#define TM_NORMALISER    "../checkpoints/improved/normaliser.txt"
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
    cfg.warmup_episodes        = 0; // use to warm up policy over options with global option rollouts before starting expansion/consolidation
    cfg.verbose                = true;
    cfg.log_interval           = 5;
    cfg.visualize_initiation_sets = true;
    cfg.max_children_per_node  = 3;
    cfg.expansion_freq         = 100; // frequency of expansion phase (every N episodes)
    cfg.mpc_steps              = 50;
    cfg.goal_region_epsilon    = 1.0f;
    cfg.save_path              = DSG_SAVE_PATH;
    cfg.training_episodes      = 20000;

    AbstractedState global_start = {{-5.3, -4.5, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

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
