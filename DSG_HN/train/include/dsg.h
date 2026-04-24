#pragma once

#include "dsc.h"
#include "../sandbox_mpc_torch.h"
#include <unordered_map>

// inherits DSC's skill learning and execution machinery, but overrides option selection to implement graph-based planning and expansion, and adds graph management methods to maintain the structure of the skill graph and its edges
class DeepSkillGraph : public DeepSkillChaining
{
public:
    struct Config : public DeepSkillChaining::Config
    {
        // ---- DSG-specific params ----
        int graph_update_freq = 10;              // update edges every N episodes
        int max_children_per_node = 3;           // max sibling skills under one parent node
        int expansion_freq = 5;                  // run expansion phase every N episodes; consolidation otherwise
        int mpc_steps = 100;                      // steps global option runs as MPC proxy toward s_rand
        float goal_region_epsilon = 1.0f;        // epsilon-ball radius (metres) defining a goal region node
        int max_expansion_tries = 10;            // max attempts per expansion phase before falling back to consolidation
        float edge_weight_kappa = 0.95f;         // multiplicative factor for edge weight updates: w *= κ^f(s), f∈{1,-1}
        int save_interval = 100;                 // checkpoint every N episodes (0 = no periodic saves)
        std::string save_path = "../dsg_models"; // directory for all checkpoints
        int training_episodes = 20000;           // total episodes for DeepSkillGraph::train()

        // DSG only: MPC look-ahead horizon (K in paper Appendix G, Table 5)
        int mpc_horizon = 7;

        // DSG only: number of CEM candidate action sequences per solve
        int mpc_candidates = 256;

        // DSG only: number of CEM refinement rounds
        int mpc_cem_rounds = 3;

        // DSG only: number of elite samples kept per CEM round
        int mpc_cem_elites = 32;

        // DSG only: CEM cost weights (matched to MpcConfig in sandbox_mpc_torch.h)
        double mpc_w_pos = 1.0;          // squared distance-to-goal at each step
        double mpc_w_heading = 0.5;      // squared heading error toward goal
        double mpc_w_terminal = 3.0;     // terminal squared distance-to-goal
        double mpc_w_smooth = 0.1;       // action smoothness (delta penalty)
        double mpc_w_backward = 0.3;     // penalty for negative vx
        double mpc_w_collision = 1000.0; // cost per imagined step in collision
        double mpc_base_radius = 0.35;   // robot footprint radius (metres)
        double mpc_clearance = 0.05;     // safety margin around obstacles
    };

    DeepSkillGraph(std::shared_ptr<RobotBridgeTrain> robot_bridge,
                   torch::Device device,
                   AbstractedState global_start,
                   const std::string &pretrain_actor_path,
                   const std::string &pretrain_critic1_path,
                   const std::string &pretrain_critic2_path,
                   const std::string &scene_file,
                   Config cfg)
        : DeepSkillChaining(robot_bridge, device, AbstractedState{}, global_start,
                            pretrain_actor_path, pretrain_critic1_path, pretrain_critic2_path,
                            scene_file, cfg, false, false), _dsg_cfg(cfg) {}

    DeepSkillGraph(std::shared_ptr<RobotBridgeTrain> robot_bridge,
                   torch::Device device,
                   AbstractedState global_start,
                   const std::string &scene_file,
                   Config cfg)
        : DeepSkillChaining(robot_bridge, device, AbstractedState{}, global_start, scene_file, cfg, false, false),
          _dsg_cfg(cfg) {}


    int train(int max_episodes) override;
    float execute() override; // graph-based: navigate from _global_start toward frontier
    void visualizeInitiationSets();
    void save(const std::string &dir) const override;
    void load(const std::string &dir, const std::string &scene_file) override;

    // Load a pre-trained Transformer transition model for CEM-based graph expansion.
    // model_path      : path to the .pt checkpoint (farnaz/transition training script)
    // normaliser_path : path to the matching normaliser.txt
    // Once loaded, _runMPC() will use the Transformer+CEM planner instead of the global-option proxy.
    void loadTransitionModel(const std::string &model_path, const std::string &normaliser_path);

protected:
    // Lightweight graph node representing a reached state in unexplored space.
    // Has no policy or classifier — membership is a pure Euclidean epsilon-ball check.
    struct GoalRegion
    {
        AbstractedState center;
        float epsilon;
    };

    // Directed edge in the skill graph; weight = accumulated reward along the edge.
    struct Edge
    {
        int from;
        int to;
        float weight;
    };

    struct Node
    {
        bool is_goal_region;
        std::shared_ptr<Skill> skill; // null if is_goal_region == true
        GoalRegion goal_region;          // only valid if is_goal_region == true
        std::vector<std::pair<int, float>> children; // indices of children nodes and associated edge weights
        std::vector<std::pair<int, float>> parents;  // indices of parent nodes and associated edge weights
        int id;
    };

    std::vector<Node> _nodes;
    // std::vector<GoalRegion> _goal_regions;
    // std::vector<Edge> _edges;

    // --- DSC overrides ---
    void _makeSkill(bool is_global, std::shared_ptr<Skill> parent) override;
    bool _shouldCreateNewOption(int v_d, const std::vector<int>& dsc_chain);
    bool _containsStart(int v_d, const std::vector<int>& dsc_chain);
    void _validateOption() override;
    float _dscRollout() override;
    std::pair<int, AbstractedState> _pickOption() override;

    void _warmupRollout(); // override: rolls out global option toward a random valid state

    // --- graph edge management ---
    void _updateEdges();
    void _ensureStructuralEdge(int from, int to, const std::string &reason);
    // Returns {total_cost, path} from from_node to to_node. Path is empty if unreachable.
    std::pair<float, std::vector<int>> _dijkstraPath(int from_node, int to_node) const;
    // Multiplicative edge weight update: w *= κ^f(s), κ=0.95, f=1 success / f=-1 failure.
    void _updateEdgeWeight(int from, int to, bool success);

private:
    std::unique_ptr<DSCProblem> _current_dsc_problem = nullptr;
    // For first-chain skills created with null Skill parent (anchor is a goal region),
    // remember the intended anchor node so we can add a structural edge on maturation.
    std::unordered_map<const Skill *, int> _pending_structural_target_node;

    // --- unified node dispatch ---
    int _totalNodes() const;
    bool _nodeCanStart(int node_idx, const AbstractedState &s, bool pessimistic) const;
    std::vector<AbstractedState> _nodeCoverageSamples(int node_idx) const;
    bool _skillCoversNodeStrict(int skill_idx, int node_idx, bool pessimistic) const;
    bool _skillCoversNodeLoose(int skill_idx, int node_idx, bool pessimistic) const;
    float _nodeDistanceToState(int node_idx, const AbstractedState &s) const;
    int _currentNodeIdx() const;
    std::string _nodeLabel(int node_idx) const; // human-readable label for logging
    std::string _optionLabel(int option_idx) const; // option index + current graph-node mapping

    // --- graph queries ---
    std::vector<int> _getV(const AbstractedState &s) const;   // V(s) = O(s) ∪ B(s): all nodes whose region contains s
    std::vector<int> _getDSt(const AbstractedState &s) const; // D(s_t) = union of descendants of all v in V(s_t)
    std::vector<int> _getReachableDescendants(int node_idx) const;
    std::vector<int> _getAncestors(int node_idx) const;
    int _nearestNodeToState(const AbstractedState &s) const;
    int _closestDisconnectedNode() const;
    std::pair<int, int> _closestPair(const std::vector<int> &D,
                                     const std::vector<int> &A) const;

    // --- transition model (loaded once via loadTransitionModel()) ---
    // Opaque handle to the Transformer+CEM MPC context (null until loadTransitionModel() is called).
    MpcContextPtr _mpc_ctx;

    // --- navigation and training primitives ---
    void _navigateTo(int node_idx, int max_steps);
    // Run receding-horizon MPC from the current environment state toward `target` for
    // cfg.mpc_steps real environment steps.  Returns the state reached.
    // Falls back to running the global option as a proxy if no transition model is loaded.
    AbstractedState _runMPC(const AbstractedState &target);
    // Returns a representative AbstractedState for any node in the unified index space.
    AbstractedState _nodeRepresentativeState(int node_idx) const;

    // --- phase methods ---
    bool _graphExpansionPhase(); // returns true if a goal region was accepted (not rejected)
    void _graphConsolidationPhase();

    Config _dsg_cfg; // full DSG config (superset of _cfg, which is sliced to DSC fields)
};
