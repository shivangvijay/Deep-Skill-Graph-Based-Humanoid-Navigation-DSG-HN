#include "dsg.h"
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <queue>
#include <sstream>
#include <unordered_map>
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

void DeepSkillGraph::addExplicitGoalRegionsIfMissing(const std::vector<AbstractedState> &centers, float epsilon)
{
    const float eps = (epsilon > 0.0f) ? epsilon : _dsg_cfg.goal_region_epsilon;
    const float eps_sq = eps * eps;
    int added = 0;

    for (const auto &c : centers)
    {
        bool exists = false;
        for (const auto &n : _nodes)
        {
            if (!n.is_goal_region)
                continue;
            const float dx = n.goal_region.center.position[0] - c.position[0];
            const float dy = n.goal_region.center.position[1] - c.position[1];
            if ((dx * dx + dy * dy) <= eps_sq)
            {
                exists = true;
                break;
            }
        }
        if (exists)
            continue;

        Node n;
        n.is_goal_region = true;
        n.goal_region = {c, eps};
        n.id = (int)_nodes.size();
        _nodes.push_back(n);
        added++;
    }

    if (added > 0)
    {
        _updateEdges();
        std::cout << "[DSG] Injected " << added << " explicit goal region(s).\n";
    }
}

// =============================================================================
// DSC overrides
// =============================================================================

void DeepSkillGraph::_makeSkill(bool is_global, std::shared_ptr<Skill> parent)
{
    // During checkpoint load there is no active DSC problem yet. Fall back to
    // the DSG-level global goal in that case to avoid null dereference.
    AbstractedState global_goal = _global_goal;
    if (_current_dsc_problem)
        global_goal = _nodeRepresentativeState(_current_dsc_problem->v_a);

    DeepSkillChaining::_makeSkill(is_global, parent, global_goal);

    if (!_is_loading_checkpoint && !_dsg_cfg.save_path.empty())
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

std::vector<AbstractedState> DeepSkillGraph::_nodeCoverageSamples(int node_idx) const
{
    std::vector<AbstractedState> samples;
    const auto &n = _nodes[node_idx];

    if (n.is_goal_region)
    {
        const AbstractedState center = n.goal_region.center;
        const float r = std::max(0.0f, n.goal_region.epsilon);
        samples.push_back(center);
        if (r <= 0.0f)
            return samples;

        // Cover interior + boundary of GR disk using concentric deterministic rings.
        const int n_theta = std::max(12, _dsg_cfg.gestation_n / 2);
        const std::array<float, 4> ring_fracs = {0.25f, 0.5f, 0.75f, 1.0f};
        static constexpr float kTwoPi = 6.28318530718f;
        for (float rf : ring_fracs)
        {
            const float rr = r * rf;
            for (int k = 0; k < n_theta; ++k)
            {
                const float th = kTwoPi * static_cast<float>(k) / static_cast<float>(n_theta);
                AbstractedState s = center;
                s.position[0] += rr * std::cos(th);
                s.position[1] += rr * std::sin(th);
                samples.push_back(s);
            }
        }
        return samples;
    }

    // Option node: use full effect set as coverage domain.
    const auto &effects = n.skill->getEffectSet();
    if (!effects.empty())
    {
        samples.reserve(effects.size());
        for (const auto &rec : effects)
            samples.push_back(rec.state);
        return samples;
    }

    samples.push_back(_nodeRepresentativeState(node_idx));
    return samples;
}

bool DeepSkillGraph::_skillCoversNodeStrict(int skill_idx, int node_idx, bool pessimistic) const
{
    if (_skills[skill_idx]->getTrainingPhase() != "mature")
        return false;

    auto samples = _nodeCoverageSamples(node_idx);
    for (const auto &s : samples)
    {
        bool ok = pessimistic ? _skills[skill_idx]->canStartPessimistic(s)
                              : _skills[skill_idx]->canStart(s);
        if (!ok)
            return false;
    }
    return true;
}

bool DeepSkillGraph::_skillCoversNodeLoose(int skill_idx, int node_idx, bool pessimistic) const
{
    if (_skills[skill_idx]->getTrainingPhase() != "mature")
        return false;

    auto samples = _nodeCoverageSamples(node_idx);

    // Loose criterion for goal regions: any sampled point in the GR is enough.
    if (_nodes[node_idx].is_goal_region)
    {
        for (const auto &s : samples)
        {
            bool ok = pessimistic ? _skills[skill_idx]->canStartPessimistic(s)
                                  : _skills[skill_idx]->canStart(s);
            if (ok)
                return true;
        }
        return false;
    }

    // Option-node connectivity: require configurable coverage ratio over effect-set samples.
    // threshold=1.0 reproduces strict full containment.
    const float threshold = std::clamp(_dsg_cfg.option_node_cover_threshold, 0.0f, 1.0f);
    int covered = 0;
    for (const auto &s : samples)
    {
        bool ok = pessimistic ? _skills[skill_idx]->canStartPessimistic(s)
                              : _skills[skill_idx]->canStart(s);
        if (ok)
            ++covered;
    }
    const float ratio = samples.empty() ? 0.0f : static_cast<float>(covered) / static_cast<float>(samples.size());
    return ratio >= threshold;
}

bool DeepSkillGraph::_containsStart(int v_d, const std::vector<int> &dsc_chain)
{
    for (auto o : dsc_chain)
    {
        if (_skillCoversNodeLoose(o, v_d, /*pessimistic=*/false))
            return true;
    }
    return false;
}

void DeepSkillGraph::_validateOption()
{
    DeepSkillChaining::_validateOption();

    for (int i = 0; i < (int)_skills.size(); ++i)
    {
        // Global option is not a DSG graph node; keep it as controller fallback only.
        if (i == _global_option_idx)
            continue;

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
            }
        }
    }
}

float DeepSkillGraph::execute()
{
    _env->resetTo(_sampleStartStateFromGlobalStartRegion());

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

AbstractedState DeepSkillGraph::_sampleStartStateFromGlobalStartRegion(int max_attempts)
{
    // Locate the GR created for _global_start (seed node) if present.
    AbstractedState center = _global_start;
    float epsilon = std::max(0.0f, _dsg_cfg.goal_region_epsilon);
    for (const auto &node : _nodes)
    {
        if (!node.is_goal_region)
            continue;
        const float dx = node.goal_region.center.position[0] - _global_start.position[0];
        const float dy = node.goal_region.center.position[1] - _global_start.position[1];
        if (std::sqrt(dx * dx + dy * dy) <= 1e-5f)
        {
            center = node.goal_region.center;
            epsilon = std::max(0.0f, node.goal_region.epsilon);
            break;
        }
    }

    if (epsilon <= 0.0f)
        return center;

    std::uniform_real_distribution<float> unit01(0.0f, 1.0f);
    std::uniform_real_distribution<float> unitTheta(0.0f, 6.28318530718f);
    for (int i = 0; i < max_attempts; ++i)
    {
        // Uniform over disk area: r = R * sqrt(u)
        const float r = epsilon * std::sqrt(unit01(_rng));
        const float th = unitTheta(_rng);

        AbstractedState sample = center;
        sample.position[0] = center.position[0] + r * std::cos(th);
        sample.position[1] = center.position[1] + r * std::sin(th);

        // Match TrainEnvironment::resetTo() clamping bounds to avoid distorted samples.
        if (sample.position[0] < _robot_bridge->x_min + 0.5f || sample.position[0] > _robot_bridge->x_max - 0.5f)
            continue;
        if (sample.position[1] < _robot_bridge->y_min + 0.5f || sample.position[1] > _robot_bridge->y_max - 0.5f)
            continue;
        if (!_robot_bridge->isConfigurationValid(sample))
            continue;

        return sample;
    }

    // Guaranteed in-region fallback.
    return center;
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
            _env->resetTo(_sampleStartStateFromGlobalStartRegion());
            _warmupRollout(); // this only warms up the polict over options by rolling out the global option - maybe remove?
        }
        else
        {
            if (episode % _dsg_cfg.expansion_freq == 0) // expand every _dsg_cfg.expansion_freq episodes, otherwise consolidate
            {
                _env->resetTo(_sampleStartStateFromGlobalStartRegion());
                // first episode will be expansion
                bool expanded = false;
                // for (int attempt = 0; attempt < _dsg_cfg.max_expansion_tries && !expanded; attempt++)
                expanded = _graphExpansionPhase();
                if (!expanded)
                {
                    std::cout << "[DSG] Expansion exhausted " << _dsg_cfg.max_expansion_tries
                              << " attempts, falling back to consolidation.\n";
                    _graphConsolidationPhase();
                }
            }
            else
            {
                // Important: skip no-op consolidation episodes before reset so we do
                // not render unnecessary respawns when --render-training is enabled.
                // if (!_canRunConsolidationEpisode())
                // {
                //     if (_dsg_cfg.verbose)
                //         std::cout << "[DSG Consolidation] Precheck: no actionable bridge yet; skipping episode without respawn.\n";
                //     continue;
                // }
                _env->resetTo(_sampleStartStateFromGlobalStartRegion());
                _graphConsolidationPhase();
            }

            // TODO: update transition model
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
                    std::string label = _optionLabel((int)i);
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

// bool DeepSkillGraph::_canRunConsolidationEpisode()
// {
//     if (_nodes.empty())
//         return false;

//     // Keep graph connectivity fresh for the same selector logic used by consolidation.
//     _updateEdges();

//     // If there is an active unresolved problem, consolidation should run.
//     if (_current_dsc_problem)
//     {
//         const bool solved = _containsStart(_current_dsc_problem->v_d, _current_dsc_problem->dsc_chain);
//         if (!solved)
//             return true;
//     }

//     // Otherwise check whether a new (v_d, v_a, v_g) problem is currently bridgeable.
//     int v_g = _closestDisconnectedNode();
//     if (v_g == -1)
//         return false;

//     auto current_state = _env->getAbstractedState();
//     auto D_s = _getDSt(current_state);
//     auto A_vg = _getAncestors(v_g);

//     std::vector<int> A_vg_unsaturated;
//     A_vg_unsaturated.reserve(A_vg.size());
//     for (int a : A_vg)
//         if (!_targetAtBranchLimit(a))
//             A_vg_unsaturated.push_back(a);

//     auto [v_d, v_a] = _closestPair(D_s, A_vg_unsaturated);
//     return v_d != -1 && v_a != -1;
// }

// =============================================================================
// save / load
// =============================================================================

void DeepSkillGraph::save(const std::string &dir) const
{
    DeepSkillChaining::save(dir); // Saves policy weights
    std::ofstream f(dir + "/graph_structure.txt");
    if (!f.is_open())
        throw std::runtime_error("Failed to open graph_structure.txt for writing in " + dir);
    std::unordered_map<const Skill *, int> skill_to_id;
    skill_to_id.reserve(_skills.size());
    for (int i = 0; i < (int)_skills.size(); ++i)
        skill_to_id[_skills[i].get()] = i;

    int option_nodes = 0;
    int mapped_option_nodes = 0;
    std::vector<int> unmapped_node_ids;
    std::vector<int> global_option_node_ids;

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
        else if (n.skill)
        {
            if (_global_option_idx >= 0 && _global_option_idx < (int)_skills.size() &&
                n.skill == _skills[_global_option_idx])
            {
                global_option_node_ids.push_back(n.id);
            }
            option_nodes++;
            auto it = skill_to_id.find(n.skill.get());
            if (it != skill_to_id.end())
            {
                f << "skill_id " << it->second;
                mapped_option_nodes++;
            }
            else
            {
                // Keep payload line format intact, but track this as a hard error below.
                unmapped_node_ids.push_back(n.id);
            }
        }
        f << "\n";
    }

    f.flush();
    if (!f)
        throw std::runtime_error("Failed while writing graph_structure.txt in " + dir);

    if (!global_option_node_ids.empty())
    {
        std::ostringstream oss;
        oss << "Global option incorrectly present in DSG graph nodes in " << dir << ": ";
        for (size_t i = 0; i < global_option_node_ids.size(); ++i)
        {
            if (i)
                oss << ", ";
            oss << "Opt-" << global_option_node_ids[i];
        }
        throw std::runtime_error(oss.str());
    }

    if (!unmapped_node_ids.empty())
    {
        std::ostringstream oss;
        oss << "Save produced unmapped option nodes (missing skill_id) in " << dir << ": ";
        for (size_t i = 0; i < unmapped_node_ids.size(); ++i)
        {
            if (i)
                oss << ", ";
            oss << "Opt-" << unmapped_node_ids[i];
        }
        throw std::runtime_error(oss.str());
    }

    std::cout << "[SaveAudit] graph_nodes=" << _nodes.size()
              << " option_nodes=" << option_nodes
              << " mapped_skill_ids=" << mapped_option_nodes
              << " save_path=" << dir << "\n";
}

void DeepSkillGraph::load(const std::string &dir, const std::string &scene_file)
{
    _is_loading_checkpoint = true;
    try
    {
        DeepSkillChaining::load(dir, scene_file);
    }
    catch (...)
    {
        _is_loading_checkpoint = false;
        throw;
    }
    _is_loading_checkpoint = false;
    _nodes.clear();

    std::ifstream f(dir + "/graph_structure.txt");
    if (!f.is_open())
        throw std::runtime_error("Missing graph_structure.txt in " + dir);

    std::string line;
    if (!std::getline(f, line))
        throw std::runtime_error("Invalid graph_structure header in " + dir + "/graph_structure.txt");
    std::istringstream hs(line);
    int n_size = 0;
    if (!(hs >> n_size) || n_size <= 0)
        throw std::runtime_error("Invalid graph_structure header in " + dir + "/graph_structure.txt");

    // Parse exactly as save() writes: 4 lines per node.
    // 1) header: id is_goal n_children n_parents
    // 2) children line: (child_id child_w)*n_children
    // 3) parents line: (parent_id parent_w)*n_parents
    // 4) payload line: goal payload or empty line for option nodes
    int legacy_skill_ptr = 0;
    int max_referenced_skill_id = -1;
    _nodes.reserve(n_size);
    for (int i = 0; i < n_size; ++i)
    {
        Node n;
        int is_goal_i = 0, n_children = 0, n_parents = 0;

        if (!std::getline(f, line))
            throw std::runtime_error("Missing node header line in graph_structure.txt at node " + std::to_string(i));
        std::istringstream hss(line);
        if (!(hss >> n.id >> is_goal_i >> n_children >> n_parents))
            throw std::runtime_error("Corrupt node header in graph_structure.txt at node " + std::to_string(i));
        n.is_goal_region = (is_goal_i != 0);

        if (!std::getline(f, line))
            throw std::runtime_error("Missing child edge line in graph_structure.txt at node " + std::to_string(i));
        std::istringstream css(line);
        std::vector<std::string> child_tokens;
        for (std::string tok; css >> tok;)
            child_tokens.push_back(tok);
        if ((int)child_tokens.size() != 2 * n_children)
            throw std::runtime_error("Corrupt child edge list in graph_structure.txt at node " + std::to_string(i));
        n.children.reserve(std::max(0, n_children));
        for (int j = 0; j < n_children; ++j)
        {
            int id = std::stoi(child_tokens[2 * j]);
            float w = static_cast<float>(std::stod(child_tokens[2 * j + 1]));
            n.children.push_back({id, w});
        }

        if (!std::getline(f, line))
            throw std::runtime_error("Missing parent edge line in graph_structure.txt at node " + std::to_string(i));
        std::istringstream pss(line);
        std::vector<std::string> parent_tokens;
        for (std::string tok; pss >> tok;)
            parent_tokens.push_back(tok);
        if ((int)parent_tokens.size() != 2 * n_parents)
            throw std::runtime_error("Corrupt parent edge list in graph_structure.txt at node " + std::to_string(i));
        n.parents.reserve(std::max(0, n_parents));
        for (int j = 0; j < n_parents; ++j)
        {
            int id = std::stoi(parent_tokens[2 * j]);
            float w = static_cast<float>(std::stod(parent_tokens[2 * j + 1]));
            n.parents.push_back({id, w});
        }

        if (!std::getline(f, line))
            throw std::runtime_error("Missing payload line in graph_structure.txt at node " + std::to_string(i));
        std::istringstream dss(line);
        if (n.is_goal_region)
        {
            if (!(dss >> n.goal_region.epsilon))
                throw std::runtime_error("Corrupt goal payload (epsilon) in graph_structure.txt at node " + std::to_string(i));
            for (float &v : n.goal_region.center.position)
                if (!(dss >> v))
                    throw std::runtime_error("Corrupt goal payload (position) in graph_structure.txt at node " + std::to_string(i));
            for (float &v : n.goal_region.center.orientation)
                if (!(dss >> v))
                    throw std::runtime_error("Corrupt goal payload (orientation) in graph_structure.txt at node " + std::to_string(i));
            for (float &v : n.goal_region.center.velocity)
                if (!(dss >> v))
                    throw std::runtime_error("Corrupt goal payload (velocity) in graph_structure.txt at node " + std::to_string(i));
            for (float &v : n.goal_region.center.angular_velocity)
                if (!(dss >> v))
                    throw std::runtime_error("Corrupt goal payload (angular_velocity) in graph_structure.txt at node " + std::to_string(i));
        }
        else
        {
            // New format: payload line contains explicit "skill_id <id>".
            // Legacy format: payload line is empty for options, so we fall back
            // to sequential assignment in the order option nodes are listed.
            int assigned_skill_id = -1;
            std::string tag;
            int parsed_skill_id = -1;
            std::istringstream oss(line);
            if ((oss >> tag >> parsed_skill_id) && tag == "skill_id")
                assigned_skill_id = parsed_skill_id;
            else
                assigned_skill_id = legacy_skill_ptr++;

            if (assigned_skill_id < 0 || assigned_skill_id >= (int)_skills.size())
                throw std::runtime_error("graph_structure references invalid skill_id " + std::to_string(assigned_skill_id) + " in " + dir);

            n.skill = _skills[assigned_skill_id];
            max_referenced_skill_id = std::max(max_referenced_skill_id, assigned_skill_id);
        }

        _nodes.push_back(n);
    }

    // Retain only skills that are actually referenced by graph option nodes.
    // Mapping above assigns option nodes from _skills in order, so any tail
    // beyond skill_ptr is not represented in the graph.
    {
        const int retain_count = std::max(1, max_referenced_skill_id + 1); // always keep global option
        if ((int)_skills.size() > retain_count)
        {
            const int dropped = (int)_skills.size() - retain_count;
            _skills.resize(retain_count);
            _unfinished_option_idx = std::min(_unfinished_option_idx, retain_count - 1);
            std::cout << "[Load] Trimmed " << dropped
                      << " skill(s) not referenced by graph nodes.\n";
        }
    }

    // Remove option nodes still in gestation; keep goal regions and non-gestating options.
    {
        std::vector<int> old_to_new(_nodes.size(), -1);
        std::vector<Node> kept;
        kept.reserve(_nodes.size());

        for (int i = 0; i < (int)_nodes.size(); ++i)
        {
            const auto &n = _nodes[i];
            bool keep = n.is_goal_region;
            if (!n.is_goal_region && n.skill)
            {
                // Global option should never appear as a graph node.
                if (_global_option_idx >= 0 && _global_option_idx < (int)_skills.size() &&
                    n.skill == _skills[_global_option_idx])
                    keep = false;
                else
                    keep = (n.skill->getTrainingPhase() != "gestation");
            }

            if (!keep)
                continue;

            old_to_new[i] = (int)kept.size();
            kept.push_back(n);
        }

        for (int new_idx = 0; new_idx < (int)kept.size(); ++new_idx)
        {
            auto &n = kept[new_idx];
            n.id = new_idx;

            std::vector<std::pair<int, float>> new_children;
            new_children.reserve(n.children.size());
            for (const auto &c : n.children)
            {
                const int old_child = c.first;
                if (old_child < 0 || old_child >= (int)old_to_new.size())
                    continue;
                const int mapped = old_to_new[old_child];
                if (mapped == -1)
                    continue;
                new_children.push_back({mapped, c.second});
            }
            n.children.swap(new_children);

            std::vector<std::pair<int, float>> new_parents;
            new_parents.reserve(n.parents.size());
            for (const auto &p : n.parents)
            {
                const int old_parent = p.first;
                if (old_parent < 0 || old_parent >= (int)old_to_new.size())
                    continue;
                const int mapped = old_to_new[old_parent];
                if (mapped == -1)
                    continue;
                new_parents.push_back({mapped, p.second});
            }
            n.parents.swap(new_parents);
        }

        if ((int)kept.size() != (int)_nodes.size())
        {
            std::cout << "[Load] Pruned " << ((int)_nodes.size() - (int)kept.size())
                      << " gestating option node(s) from graph.\n";
        }
        _nodes.swap(kept);
    }

    // Remove goal-region nodes that are disconnected from the main component.
    // We keep node 0 as the anchor (seed/global-start GR).
    if (!_nodes.empty())
    {
        std::vector<char> reachable(_nodes.size(), 0);
        std::queue<int> q;
        reachable[0] = 1;
        q.push(0);

        while (!q.empty())
        {
            const int u = q.front();
            q.pop();

            for (const auto &c : _nodes[u].children)
            {
                const int v = c.first;
                if (v >= 0 && v < (int)_nodes.size() && !reachable[v])
                {
                    reachable[v] = 1;
                    q.push(v);
                }
            }
            for (const auto &p : _nodes[u].parents)
            {
                const int v = p.first;
                if (v >= 0 && v < (int)_nodes.size() && !reachable[v])
                {
                    reachable[v] = 1;
                    q.push(v);
                }
            }
        }

        std::vector<int> old_to_new(_nodes.size(), -1);
        std::vector<Node> kept;
        kept.reserve(_nodes.size());

        for (int i = 0; i < (int)_nodes.size(); ++i)
        {
            const auto &n = _nodes[i];
            const bool drop_disconnected_goal = (i != 0 && n.is_goal_region && !reachable[i]);
            if (drop_disconnected_goal)
                continue;
            old_to_new[i] = (int)kept.size();
            kept.push_back(n);
        }

        for (int new_idx = 0; new_idx < (int)kept.size(); ++new_idx)
        {
            auto &n = kept[new_idx];
            n.id = new_idx;

            std::vector<std::pair<int, float>> new_children;
            new_children.reserve(n.children.size());
            for (const auto &c : n.children)
            {
                const int old_child = c.first;
                if (old_child < 0 || old_child >= (int)old_to_new.size())
                    continue;
                const int mapped = old_to_new[old_child];
                if (mapped == -1)
                    continue;
                new_children.push_back({mapped, c.second});
            }
            n.children.swap(new_children);

            std::vector<std::pair<int, float>> new_parents;
            new_parents.reserve(n.parents.size());
            for (const auto &p : n.parents)
            {
                const int old_parent = p.first;
                if (old_parent < 0 || old_parent >= (int)old_to_new.size())
                    continue;
                const int mapped = old_to_new[old_parent];
                if (mapped == -1)
                    continue;
                new_parents.push_back({mapped, p.second});
            }
            n.parents.swap(new_parents);
        }

        if ((int)kept.size() != (int)_nodes.size())
        {
            std::cout << "[Load] Pruned " << ((int)_nodes.size() - (int)kept.size())
                      << " disconnected goal region node(s) from graph.\n";
        }
        _nodes.swap(kept);
    }

    int goal_count = 0, option_count = 0;
    for (const auto &n : _nodes)
    {
        if (n.is_goal_region)
            ++goal_count;
        else
            ++option_count;
    }
    std::cout << "[Load] Graph nodes: total=" << _nodes.size()
              << " goals=" << goal_count
              << " options=" << option_count
              << " loaded_skills=" << _skills.size() << "\n";

    if (_skills.size() > 1 && option_count == 0)
    {
        throw std::runtime_error(
            "Loaded graph has zero option nodes while checkpoint has multiple skills. "
            "Refusing to continue with likely-corrupted graph_structure in " + dir);
    }

    _updateEdges();

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
}

// =============================================================================
// Graph edge management
// =============================================================================

void DeepSkillGraph::_updateEdges()
{
    const int N = _totalNodes();
    const float connect_threshold = 0.1f; // 10% coverage to add an edge
    const float stale_threshold = 1.0f;   // full failure to remove an edge

    auto is_mature = [&](int idx) -> bool
    {
        return _nodes[idx].is_goal_region || _nodes[idx].skill->getTrainingPhase() == "mature";
    };

    auto inside_goal_region = [&](int gr_idx, const AbstractedState &s) -> bool
    {
        const auto &gr = _nodes[gr_idx].goal_region;
        const float dx = s.position[0] - gr.center.position[0];
        const float dy = s.position[1] - gr.center.position[1];
        return std::sqrt(dx * dx + dy * dy) <= gr.epsilon;
    };

    // --- Addition pass: check all ordered pairs (i, j) of mature nodes ---
    auto check_link = [&](int src, int dst) -> bool
    {
        if (src == dst)
            return false;

        // Skip if edge already exists
        for (const auto &c : _nodes[src].children)
            if (c.first == dst)
                return false;

        if (_nodes[src].is_goal_region)
        {
            const auto src_samples = _nodeCoverageSamples(src);
            if (src_samples.empty())
                return false;

            int covered = 0;
            for (const auto &s : src_samples)
            {
                if (_nodeCanStart(dst, s, true))
                    covered++;
            }
            const float ratio = static_cast<float>(covered) / static_cast<float>(src_samples.size());
            return ratio >= connect_threshold;
        }
        else
        {
            const auto &effects = _nodes[src].skill->getEffectSet();
            if (effects.empty())
                return false;

            int covered = 0;
            for (const auto &rec : effects)
            {
                bool ok = false;
                if (_nodes[dst].is_goal_region)
                    ok = inside_goal_region(dst, rec.state);
                else
                    ok = _nodeCanStart(dst, rec.state, true);

                if (ok)
                    covered++;
            }

            const float ratio = static_cast<float>(covered) / static_cast<float>(effects.size());
            return ratio >= connect_threshold;
        }
    };

    for (int i = 0; i < N; ++i)
    {
        if (!is_mature(i))
            continue;
        for (int j = 0; j < N; ++j)
        {
            if (!is_mature(j))
                continue;
            if (check_link(i, j))
            {
                _nodes[i].children.push_back({j, 1.0f});
                _nodes[j].parents.push_back({i, 1.0f});
                std::cout << "[UpdateEdges] Edge added: " << _nodeLabel(i) << " → " << _nodeLabel(j)
                          << " (threshold " << connect_threshold << " met)\n";
            }
        }
    }

    // --- Deletion pass: check all existing edges ---
    auto check_stale = [&](int src, int dst) -> bool
    {
        if (_nodes[src].is_goal_region)
        {
            const auto src_samples = _nodeCoverageSamples(src);
            if (src_samples.empty())
                return true;

            int fail = 0;
            for (const auto &s : src_samples)
            {
                if (!_nodeCanStart(dst, s, false))
                    fail++;
            }
            const float fail_ratio = static_cast<float>(fail) / static_cast<float>(src_samples.size());
            return fail_ratio >= stale_threshold;
        }
        else
        {
            const auto &effects = _nodes[src].skill->getEffectSet();
            if (effects.empty())
                return true;

            // Sample K=50 or all if effects.size() is small
            const int K = std::min(50, (int)effects.size());
            int fail = 0;
            for (int k = 0; k < K; ++k)
            {
                // Random sampling for efficiency on large effect sets
                const auto &sample_state = effects[rand() % effects.size()].state;
                bool ok = false;
                if (_nodes[dst].is_goal_region)
                    ok = inside_goal_region(dst, sample_state);
                else
                    ok = _nodeCanStart(dst, sample_state, false);

                if (!ok)
                    fail++;
            }

            const float fail_ratio = static_cast<float>(fail) / static_cast<float>(K);
            return fail_ratio >= stale_threshold;
        }
    };

    for (int i = 0; i < N; ++i)
    {
        auto &ch = _nodes[i].children;
        for (int ci = (int)ch.size() - 1; ci >= 0; --ci)
        {
            int j = ch[ci].first;
            if (check_stale(i, j))
            {
                std::cout << "[UpdateEdges] Edge removed: " << _nodeLabel(i) << " → " << _nodeLabel(j)
                          << " (fail ratio >= " << stale_threshold << ")\n";

                auto &par = _nodes[j].parents;
                par.erase(std::remove_if(par.begin(), par.end(),
                                         [i](const auto &p)
                                         { return p.first == i; }),
                          par.end());
                ch.erase(ch.begin() + ci);
            }
        }
    }
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

std::string DeepSkillGraph::_optionLabel(int option_idx) const
{
    if (option_idx == _global_option_idx)
        return "global";

    std::string node_part = "pending";
    if (option_idx >= 0 && option_idx < (int)_skills.size())
    {
        for (const auto &node : _nodes)
        {
            if (!node.is_goal_region && node.skill == _skills[option_idx])
            {
                node_part = "Opt-" + std::to_string(node.id);
                break;
            }
        }
    }
    return "Skill-" + std::to_string(option_idx) + "(node=" + node_part + ")";
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
// void DeepSkillGraph::_navigateTo(int node_idx, int max_steps)
// {
//     auto pathToString = [&](const std::vector<int> &path, int from) -> std::string
//     {
//         std::string out = _nodeLabel(from);
//         for (int v : path)
//         {
//             out += " -> ";
//             out += _nodeLabel(v);
//         }
//         return out;
//     };

//     auto start_state = _env->getAbstractedState();
//     std::cout << "[Navigate] Start navigateTo from="
//               << start_state.position[0] << ", " << start_state.position[1]
//               << " target=" << _nodeLabel(node_idx)
//               << " max_steps=" << max_steps << "\n";

//     int step = 0;
//     while (step < max_steps)
//     {
//         auto current_state = _env->getAbstractedState();
//         // _env->setGoal(_nodeRepresentativeState(node_idx));
//         // bool env_done = _env->computeReward(current_state).second.data_ptr<float>()[0] > 0.5;
//         bool env_done = _env->getUnderlyingState().second; // gonna just terminate if in collisions
//         bool already_in_target = _nodeCanStart(node_idx, current_state, false);

//         int best_node_idx = -1;
//         int next_hop_idx = -1;
//         AbstractedState best_goal;
//         float best_cost = std::numeric_limits<float>::infinity();
//         std::vector<int> best_path;
//         bool force_self_hop = false;

//         if (already_in_target || env_done)
//         {
//             if (already_in_target)
//             {
//                 std::cout << "[Navigate] Already in target " << _nodeLabel(node_idx) << "\n";
//                 // If target is an option and we're already in its initiation set,
//                 // synthesize a self-hop so the normal execution block runs it once.
//                 if (!_nodes[node_idx].is_goal_region && node_idx != _global_option_idx)
//                 {
//                     force_self_hop = true;
//                     best_node_idx = node_idx;
//                     next_hop_idx = node_idx;
//                     best_goal = _nodeRepresentativeState(node_idx);
//                     best_path = {node_idx};
//                 }
//                 else
//                 {
//                     return;
//                 }
//             }
//             else
//             {
//                 std::cout << "[Navigate] Reached point "
//                           << current_state.position[0] << ", " << current_state.position[1] << "\n";
//                 return;
//             }
//         }

//         // V(s_t): all nodes containing the current state
//         auto V_s = _getV(current_state);

//         // Case (a): find least-cost path to node_idx starting from a skill node in V_s
//         if (!force_self_hop)
//         {
//             for (int v : V_s)
//             {
//                 // if (_nodes[v].is_goal_region)
//                 //     continue; // Only skills have policies we can execute

//                 auto [cost, path] = _dijkstraPath(v, node_idx);
//                 if (!path.empty() && cost < best_cost)
//                 {
//                     best_cost = cost;
//                     best_node_idx = v;
//                     next_hop_idx = path.front();
//                     best_goal = _nodeRepresentativeState(next_hop_idx);
//                     best_path = path;
//                 }
//             }
//         }

//         // Case (b): Recovery — use Global Option to steer toward target if no graph path exists
//         if (!force_self_hop && best_node_idx == -1)
//         {
//             best_node_idx = _global_option_idx; // Use the base class global skill index
//             best_goal = _nodeRepresentativeState(node_idx);

//             std::cout << "[Navigate] No path found, using global option\n";
//             std::cout << "[Navigate] No graph path from current state. "
//                       << "Executing GlobalOpt toward " << _nodeLabel(node_idx) << "\n";

//             auto [steps_taken, cum_reward, done, first_poo, last_poo] = // reak if done?
//                 _skills[_global_option_idx]->rollout(best_goal);

//             step += steps_taken;
//             if (steps_taken == 0)
//             {
//                 return;
//             }
//             if (_cfg.verbose)
//                 std::cout << "[DSG Navigation] No path found. Using Global Option recovery toward "
//                           << _nodeLabel(node_idx) << "\n";
//         }
//         else
//         {
//             std::cout << "[Navigate] Path selected: "
//                       << pathToString(best_path, best_node_idx)
//                       << " | execute=" << _nodeLabel(best_node_idx)
//                       << " -> next_hop=" << _nodeLabel(next_hop_idx) << "\n";

//             // Execute the chosen skill
//             // Access the skill object directly from the Node to avoid index-mismatch bugs
            
//             while (_nodes[best_node_idx].is_goal_region && best_node_idx < _)
//             {

//             }
//             auto [steps_taken, cum_reward, done, first_poo, last_poo] =
//                 _nodes[best_node_idx].skill->rollout(best_goal);

//             if (steps_taken == 0)
//             {
//                 return;
//             }
//             step += steps_taken;
//         }

//         // Update weights if we were following an explicit graph edge
//         if (next_hop_idx != -1)
//         {
//             bool success = _nodeCanStart(next_hop_idx, _env->getAbstractedState(), false);
//             _updateEdgeWeight(best_node_idx, next_hop_idx, success);
//         }
//     }

//     auto final_state = _env->getAbstractedState();
//     std::cout << "[Navigate] Stopped at step budget with state=("
//               << final_state.position[0] << ", " << final_state.position[1] << ")\n";
// }

void DeepSkillGraph::_navigateTo(int node_idx, int max_steps)
{
    auto labelsToStringLocal = [&](const std::vector<int> &ids) -> std::string
    {
        std::string out = "[";
        for (size_t i = 0; i < ids.size(); ++i)
        {
            if (i > 0)
                out += ", ";
            out += _nodeLabel(ids[i]);
        }
        out += "]";
        return out;
    };

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
        
        // Terminate immediately if in collision
        bool in_collision = _env->getUnderlyingState().second;
        if (in_collision) {
            std::cout << "[Navigate] Termination: Collision detected.\n";
            return;
        }

        // Check if we have arrived in the target node's initiation set
        if (_nodeCanStart(node_idx, current_state, true)) {
            std::cout << "[Navigate] Arrived in target " << _nodeLabel(node_idx) << "\n";
            return;
        }

        // 1. Path Planning: Find executable path via Dijkstra
        auto V_s = _getV(current_state);
        std::cout << "[Navigate] V(s_t)=" << labelsToStringLocal(V_s) << "\n";
        int best_node_idx = -1;
        int next_hop_idx = -1;
        float best_cost = std::numeric_limits<float>::infinity();
        std::vector<int> best_path;

        for (int v : V_s)
        {
            auto [cost, path] = _dijkstraPath(v, node_idx);
            if (!path.empty())
            {
                std::cout << "[Navigate] Candidate from " << _nodeLabel(v)
                          << " cost=" << cost
                          << " path=" << pathToString(path, v) << "\n";
            }
            else
            {
                std::cout << "[Navigate] Candidate from " << _nodeLabel(v)
                          << " has no path to " << _nodeLabel(node_idx) << "\n";
            }
            if (!path.empty() && cost < best_cost)
            {
                best_cost = cost;
                best_node_idx = v;
                best_path = path;
            }
        }
        if (best_node_idx != -1)
        {
            std::cout << "[Navigate] Best candidate: " << _nodeLabel(best_node_idx)
                      << " cost=" << best_cost
                      << " path=" << pathToString(best_path, best_node_idx) << "\n";
        }
        else
        {
            std::cout << "[Navigate] No graph candidate found from current V(s_t).\n";
        }

        // 2. Execution Logic
        int exec_node = -1;
        AbstractedState current_subgoal;

        if (best_node_idx != -1)
        {
            // Peeking: If current node is a Goal Region, skip to the first Skill in path
            exec_node = best_node_idx;
            std::vector<int> path_copy = best_path;

            while (exec_node != -1 && _nodes[exec_node].is_goal_region)
            {
                if (!path_copy.empty())
                {
                    std::cout << "[Navigate] Peeking through goal node "
                              << _nodeLabel(exec_node)
                              << " -> " << _nodeLabel(path_copy.front()) << "\n";
                    exec_node = path_copy.front();
                    path_copy.erase(path_copy.begin());
                }
                else
                {
                    // Path only contained the GR we are standing in
                    std::cout << "[Navigate] Path ended at goal node "
                              << _nodeLabel(best_node_idx)
                              << " with no executable option.\n";
                    exec_node = -1;
                    break;
                }
            }

            if (exec_node != -1) {
                // Determine next_hop for weight updates and set local goal
                next_hop_idx = (exec_node == best_node_idx && !best_path.empty()) 
                               ? best_path.front() : exec_node;
                current_subgoal = _nodeRepresentativeState(next_hop_idx);
                std::cout << "[Navigate] Selected exec_node=" << _nodeLabel(exec_node)
                          << " next_hop=" << _nodeLabel(next_hop_idx) << "\n";
            }
        }

        // 3. Rollout: Execute found Skill or Fallback to Global Option
        if (exec_node == -1 || _nodes[exec_node].is_goal_region)
        {
            std::cout << "[Navigate] Fallback to Global Option ("
                      << ((exec_node == -1) ? "no executable node" : "exec node is goal region")
                      << ").\n";
            exec_node = _global_option_idx;
            current_subgoal = _nodeRepresentativeState(node_idx);
        }

        std::cout << "[Navigate] Step " << step << " | Executing: " << _nodeLabel(exec_node) << "\n";

        auto [steps_taken, cum_reward, done, first_poo, last_poo] = (exec_node == _global_option_idx)
            ? _skills[_global_option_idx]->rollout(current_subgoal)
            : _nodes[exec_node].skill->rollout(current_subgoal);

        // Safety break for zero-step rollouts to prevent infinite loop
        if (steps_taken == 0) {
            std::cout << "[Navigate] Warning: Skill rollout took 0 steps. Breaking.\n";
            return;
        }

        step += steps_taken;

        // 4. Edge Maintenance: Update weight of the edge we intended to traverse
        if (next_hop_idx != -1 && exec_node != _global_option_idx)
        {
            bool success = _nodeCanStart(next_hop_idx, _env->getAbstractedState(), false);
            _updateEdgeWeight(best_node_idx, next_hop_idx, success);
        }
    }

    std::cout << "[Navigate] Navigation ended after " << step << " steps.\n";
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
    auto eng = _robot_bridge->getEngine();

    // s_0: state at the beginning of this expansion episode.
    const AbstractedState s_0 = _env->getAbstractedState();

    // 1. Sample random reachable state (exploration target)
    AbstractedState s_rand = _env->getRandomValidAbstractedState();

    // Debug markers for expansion:
    //   cyan  = s_0
    //   magenta = s_rand
    if (eng && eng->render_m)
    {
        eng->setDebugSpheres({
            {s_0.position[0], s_0.position[1], 0.20f, 0.10f, {0.0f, 1.0f, 1.0f, 0.95f}},
            {s_rand.position[0], s_rand.position[1], 0.20f, 0.10f, {1.0f, 0.0f, 1.0f, 0.95f}},
        });
    }

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

    // 4.5 Safety gate: require the full goal-region epsilon-ball around s_mpc
    // to be clear of obstacles.
    {
        const float required_clearance = _dsg_cfg.goal_region_epsilon;
        const float obs_dist = _env->distanceToNearestObstacle(s_mpc.position, s_mpc.orientation);
        if (obs_dist < required_clearance)
        {
            if (_cfg.verbose)
            {
                std::cout << "[DSG Expansion] Rejected — s_mpc too close to obstacle. "
                          << "obs_dist=" << obs_dist
                          << " required>=" << required_clearance
                          << " (eps=" << _dsg_cfg.goal_region_epsilon << ")\n";
            }
            return false;
        }
    }

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
        std::cout << "Option: " << _optionLabel(option)
                  << " Goal: (" << goal.position[0] << ", " << goal.position[1] << ")\n";

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
                      << " (" << _optionLabel(option) << ")"
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
    // Ensure the visualization reads the exact same saved records as
    // visualize_initiation_set.py (skill_*_classifier.svm_positives.txt).
    save(_dsg_cfg.save_path);

    std::string temp_file = "/tmp/init_sets.txt";
    std::ofstream out(temp_file);

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

    std::string cmd = "python ../../visualize.py \"" + _scene_file_path + "\" \"" + temp_file +
                      "\" --models-dir \"" + _dsg_cfg.save_path + "\"";
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
    auto eng = _robot_bridge->getEngine();

    auto setConsolidationMarkers = [&](int v_d, int v_a)
    {
        if (!(eng && eng->render_m))
            return;
        const auto s_d = _nodeRepresentativeState(v_d);
        const auto s_a = _nodeRepresentativeState(v_a);

        // Debug markers for consolidation:
        //   yellow = v_d
        //   green  = v_a
        eng->setDebugSpheres({
            {s_d.position[0], s_d.position[1], 0.20f, 0.10f, {1.0f, 1.0f, 0.0f, 0.95f}},
            {s_a.position[0], s_a.position[1], 0.20f, 0.10f, {0.0f, 1.0f, 0.0f, 0.95f}},
        });
    };
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
        setConsolidationMarkers(_current_dsc_problem->v_d, _current_dsc_problem->v_a);
        solved_current_problem = _containsStart(_current_dsc_problem->v_d, _current_dsc_problem->dsc_chain);
        if (solved_current_problem)
        {
            int v_d = _current_dsc_problem->v_d;

            int bridge_skill_idx = -1;
            for (int o : _current_dsc_problem->dsc_chain)
            {
                if (_skillCoversNodeLoose(o, v_d, /*pessimistic=*/false))
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
        setConsolidationMarkers(v_d, v_a);

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
        std::cout << _optionLabel(_unfinished_option_idx)
                  << " is the new option being trained to bridge "
                  << _nodeLabel(v_d) << " to " << _nodeLabel(v_g) << "\n";
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
    _validateOption(); // this MUST run, it creates the structural edge to parent and also adds to graph if successful
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

#define SCENE_FILE "../config/scene/umaze_scene.xml"
#define OG_ACTOR "../models/UMaze/pretrain_actor_umaze.pt"
#define OG_CRITIC1 "../models/UMaze/pretrain_critic_1_umaze.pt" 
#define OG_CRITIC2 "../models/UMaze/pretrain_critic_2_umaze.pt"
#define DSG_SAVE_PATH "../models/UMaze/dsg_models/run3"
#define DSG_LOAD_PATH "../models/UMaze/dsg_models/run3"
#define CONTINUE_RUN true
#define TM_CHECKPOINT "../checkpoints/improved/transition_transformer_delta_latest.pt"
#define TM_NORMALISER "../checkpoints/improved/normaliser.txt"
#define TEST false

#define RENDER_TRAINING false
#define RENDER_REALTIME false // if true, will
#define RENDER_EVAL false
#define ADD_EXPLICIT_SMPC_GOAL_REGIONS true

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
        // Some environments report MPS available, but runtime tensor creation/load
        // can still fail (e.g., unsupported macOS runtime). Probe once and fallback.
        try
        {
            auto probe = torch::ones({1}, torch::TensorOptions().device(torch::kMPS));
            (void)probe;
            std::cout << "MPS is available! Training on Apple GPU.\n";
            device = torch::Device(torch::kMPS);
        }
        catch (const c10::Error &e)
        {
            std::cout << "[DeviceSelect] MPS reported available but is not usable: "
                      << e.what_without_backtrace()
                      << "\n[DeviceSelect] Falling back to CPU.\n";
            device = torch::Device(torch::kCPU);
        }
    }

    auto robot_bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, /*render=*/RENDER_TRAINING);
    robot_bridge->setRenderRealtime(RENDER_REALTIME);

    DeepSkillGraph::Config cfg;
    // -------------------------------------------------------------------------
    // Episode / runtime control
    // -------------------------------------------------------------------------
    cfg.training_episodes = 20000;
    cfg.steps_per_episode = 1000;
    cfg.warmup_episodes = 0; // warm up policy-over-options before DSG expansion/consolidation
    cfg.save_path = DSG_SAVE_PATH;
    cfg.save_interval = 200;

    // -------------------------------------------------------------------------
    // Environment + geometric termination / region thresholds
    // -------------------------------------------------------------------------
    cfg.success_radius = 0.5f;      // env local-goal success radius
    cfg.goal_region_epsilon = 0.5f; // GR epsilon-ball radius

    // -------------------------------------------------------------------------
    // Skill lifecycle (DSC option learning / validation)
    // -------------------------------------------------------------------------
    cfg.gestation_train_steps = 5000;
    cfg.gestation_n = 60;
    cfg.last_k = 30;
    cfg.refinement_eps = 30;
    cfg.max_option_steps = 50;
    cfg.max_skills = 10;
    cfg.val_accuracy_threshold = 0.0f;
    cfg.exploration_noise_gestation = 0.2;
    cfg.exploration_noise_mature = 0.1;

    // -------------------------------------------------------------------------
    // Classifier / initiation set behaviour
    // -------------------------------------------------------------------------
    cfg.nu = 0.05;                                       // one-class SVM outlier fraction (pessimistic tightness, lower = tighter)
    cfg.pessimistic_ocsvm_gamma = 0.5;                  // one-class RBF gamma for pessimistic classifier
    cfg.optimistic_ocsvm_gamma = 0.1;                   // one-class RBF gamma for optimistic classifier in phase-1
    cfg.optimistic_ocsvm_nu_divisor = 10.0;             // optimistic phase-1 nu = nu / divisor
    cfg.optimistic_svc_c = 1.0;                         // phase-2 optimistic SVC C (Higher C: penalizes training errors more, tries harder to classify training points correctly (especially positives), less regularization. Lower C: allows more misclassification, smoother/more regularized boundary.)
    cfg.optimistic_svc_gamma = 0.5;                     // phase-2 optimistic SVC gamma (kernel coefficient, lower = wider)
    cfg.optimistic_svc_balance_classes = false;         // phase-2 class balancing for optimistic SVC
    cfg.optimistic_svc_positive_margin_tolerance = 0.0; // include SVC decision margin band: keep points with decisionValue >= -tol
    cfg.subgoal_robustness_tolerance = 0.25f;           // neighborhood check around sampled subgoal
    cfg.negative_samples_per_failure = 1;               // failure negatives added per failed rollout
    cfg.option_node_cover_threshold = 0.8f;             // chain-closure criterion for option-node v_d coverage

    // -------------------------------------------------------------------------
    // Policy / critic architectures
    // -------------------------------------------------------------------------
    cfg.actor_layers = {256, 256, 256};
    cfg.critic_layers = {256, 256, 256};
    cfg.poo_layers = {256, 256, 256};

    // -------------------------------------------------------------------------
    // Learning dynamics (TD3 + policy-over-options)
    // -------------------------------------------------------------------------
    cfg.lr_actor = 5e-5f;
    cfg.lr_critic = 1e-4f;
    cfg.lr_actor_global = 0.0f;
    cfg.lr_critic_global = 0.0f;
    cfg.lr_poo = 1e-4f;
    cfg.actor_warmup_steps = 0; // TBD
    cfg.tau = 0.005f;
    cfg.gamma = 0.99f;
    cfg.batch_size = 256;
    cfg.poo_batch_size = 16;
    cfg.actor_update_freq = 2;
    cfg.updates_per_step = 2;

    // -------------------------------------------------------------------------
    // DSG graph structure + edge maintenance
    // -------------------------------------------------------------------------
    cfg.graph_update_freq = 10;
    cfg.max_children_per_node = 5;
    cfg.expansion_freq = 100; // every N episodes run expansion, else consolidation
    cfg.max_expansion_tries = 50;
    cfg.edge_weight_kappa = 0.95f; // learning rate for edge weight updates

    // -------------------------------------------------------------------------
    // Expansion controller (global-option proxy / transition-model MPC)
    // -------------------------------------------------------------------------
    cfg.mpc_steps = 200; // real env steps while extending toward s_rand
    cfg.mpc_horizon = 7;
    cfg.mpc_candidates = 256;
    cfg.mpc_cem_rounds = 3;
    cfg.mpc_cem_elites = 32;
    cfg.mpc_w_pos = 1.0;
    cfg.mpc_w_heading = 0.5;
    cfg.mpc_w_terminal = 3.0;
    cfg.mpc_w_smooth = 0.1;
    cfg.mpc_w_backward = 0.3;
    cfg.mpc_w_collision = 1000.0;
    cfg.mpc_base_radius = 0.35;
    cfg.mpc_clearance = 0.05;

    // -------------------------------------------------------------------------
    // Logging / debug visibility
    // -------------------------------------------------------------------------
    cfg.render_training = RENDER_TRAINING;
    cfg.verbose = true;
    cfg.log_interval = 25;
    cfg.visualize_initiation_sets = true;
    std::cout << "[RenderConfig] render_training=" << (RENDER_TRAINING ? "true" : "false")
              << ", render_eval=" << (RENDER_EVAL ? "true" : "false") << "\n";

    AbstractedState global_start = {{-5.3, -4.5, 0.}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    // AbstractedState global_start = {{0.0, 0.0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    // DSG has no fixed global goal — the graph grows outward from global_start
    DeepSkillGraph dsg(robot_bridge, device, global_start,
                       OG_ACTOR, OG_CRITIC1, OG_CRITIC2, SCENE_FILE, cfg);

    const std::string load_path = CONTINUE_RUN ? std::string(DSG_LOAD_PATH) : std::string();
    std::cout << "[CheckpointConfig] continue_run=" << (CONTINUE_RUN ? "true" : "false")
              << ", load_path=" << (CONTINUE_RUN ? load_path : "<none>")
              << ", save_path=" << cfg.save_path << "\n";

    if (CONTINUE_RUN)
    {
        const std::filesystem::path load_dir(load_path);
        const std::vector<std::string> required_files = {
            "graph_structure.txt",
            "skill_count.txt",
            "scene_file.txt"};

        for (const auto &fname : required_files)
        {
            const auto fp = load_dir / fname;
            if (!std::filesystem::exists(fp))
            {
                throw std::runtime_error(
                    "CONTINUE_RUN=true but missing required artifact: " + fp.string() +
                    " (load dir: " + load_dir.string() + ")");
            }
        }

        try
        {
            dsg.load(load_path, SCENE_FILE);
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("Failed to load DSG checkpoint from " + load_path + ": " + e.what());
        }
    }

    if (ADD_EXPLICIT_SMPC_GOAL_REGIONS)
    {
        // Add hand-picked s_mpc-like goals if they are not already represented by a GR.
        const std::array<std::array<float, 2>, 5> explicit_xy = {
            std::array<float, 2>{0.0f, -2.0f},
            std::array<float, 2>{2.0f, -1.0f},
            std::array<float, 2>{2.0f, 3.0f},
            std::array<float, 2>{2.0f, 6.0f},
            std::array<float, 2>{-4.0f, 6.0f},
        };

        std::vector<AbstractedState> explicit_gr_centers;
        explicit_gr_centers.reserve(explicit_xy.size());
        for (const auto &xy : explicit_xy)
        {
            AbstractedState s = global_start;
            s.position[0] = xy[0];
            s.position[1] = xy[1];
            explicit_gr_centers.push_back(s);
        }

        dsg.addExplicitGoalRegionsIfMissing(explicit_gr_centers, cfg.goal_region_epsilon);
        std::cout << "[DSG] Explicit s_mpc goal-region injection enabled (N="
                  << explicit_gr_centers.size() << ", eps=" << cfg.goal_region_epsilon << ")\n";
        if (!cfg.save_path.empty())
        {
            std::filesystem::create_directories(cfg.save_path);
            dsg.save(cfg.save_path);
            std::cout << "[DSG] Saved checkpoint after explicit goal-region injection to "
                      << cfg.save_path << "\n";
        }
    }

    // Load transition model if checkpoints exist (graceful skip if not yet trained)
    if (std::filesystem::exists(TM_CHECKPOINT) && std::filesystem::exists(TM_NORMALISER))
        dsg.loadTransitionModel(TM_CHECKPOINT, TM_NORMALISER);
    else
        std::cout << "[DSG] No transition model found at " << TM_CHECKPOINT
                  << " — using global-option proxy for graph expansion.\n";

    if (!TEST)
    {
        dsg.save(cfg.save_path); // save initial graph structure before training so we can visualize initiation sets even if training is interrupted
        int n = dsg.train(cfg.training_episodes); // cfg.training_episodes default = 20000
        std::cout << "\nTraining complete: " << n << " skill(s) in graph.\n";
    }
    else if (!CONTINUE_RUN)
    {
        dsg.load(DSG_SAVE_PATH, SCENE_FILE);
    }

    // EVALUATION (uncomment to run after training, adjust episode count as desired)
    // float total = 0.0f;
    // if (RENDER_EVAL)
    //     robot_bridge->startRender();
    // for (int i = 0; i < 40; ++i)
    // {
    //     float r = dsg.execute();
    //     total += r;
    //     std::cout << "  Episode " << i + 1 << ": reward = " << r << "\n";
    // }
    // std::cout << "Mean reward: " << (total / 40.0f) << "\n";

    return 0;
}
