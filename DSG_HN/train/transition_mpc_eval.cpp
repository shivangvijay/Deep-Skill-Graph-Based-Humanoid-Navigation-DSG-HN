#include <torch/torch.h>

#include <boost/program_options.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace po = boost::program_options;

static constexpr int kRawStateDim = 13;
static constexpr int kJointDim = 35;
static constexpr int kFullStateDim = kRawStateDim + (2 * kJointDim);
static constexpr int kInputStateDim = 7 + (2 * kJointDim);
static constexpr int kActionDim = 3;
static constexpr int kInputDim = kInputStateDim + kActionDim;
static constexpr int kOutputDim = 6 + (2 * kJointDim);

static constexpr int IDX_X = 0;
static constexpr int IDX_Y = 1;
static constexpr int IDX_Z = 2;
static constexpr int IDX_QW = 3;
static constexpr int IDX_QX = 4;
static constexpr int IDX_QY = 5;
static constexpr int IDX_QZ = 6;
static constexpr int IDX_VX = 7;
static constexpr int IDX_VY = 8;
static constexpr int IDX_VZ = 9;
static constexpr int IDX_OX = 10;
static constexpr int IDX_OY = 11;
static constexpr int IDX_OZ = 12;

static std::string two_digit(int value)
{
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << value;
    return oss.str();
}

static std::vector<std::string> current_state_columns()
{
    std::vector<std::string> columns = {"x", "y", "z", "qw", "qx", "qy", "qz", "vx", "vy", "vz", "omega_x", "omega_y", "omega_z"};
    for (int i = 0; i < kJointDim; ++i) columns.push_back("joint_pos_" + two_digit(i));
    for (int i = 0; i < kJointDim; ++i) columns.push_back("joint_vel_" + two_digit(i));
    return columns;
}

static std::vector<std::string> action_columns()
{
    return {"cmd_vx", "cmd_vy", "cmd_yaw"};
}

static std::vector<std::string> split_csv_line(const std::string &line)
{
    std::vector<std::string> cells;
    std::string cell;
    std::stringstream ss(line);
    while (std::getline(ss, cell, ',')) cells.push_back(cell);
    return cells;
}

static std::vector<int> lookup_indices(const std::unordered_map<std::string, int> &header_index,
                                       const std::vector<std::string> &columns)
{
    std::vector<int> indices;
    indices.reserve(columns.size());
    for (const auto &name : columns)
    {
        auto it = header_index.find(name);
        if (it == header_index.end()) throw std::runtime_error("Missing CSV column: " + name);
        indices.push_back(it->second);
    }
    return indices;
}

static torch::Tensor make_tensor_from_flat(const std::vector<float> &data, int64_t rows, int64_t cols)
{
    auto tensor = torch::empty({rows, cols}, torch::TensorOptions().dtype(torch::kFloat32));
    std::memcpy(tensor.data_ptr<float>(), data.data(), data.size() * sizeof(float));
    return tensor;
}

static torch::Tensor make_double_tensor_from_flat(const std::vector<double> &data, int64_t rows, int64_t cols)
{
    auto tensor = torch::empty({rows, cols}, torch::TensorOptions().dtype(torch::kFloat64));
    std::memcpy(tensor.data_ptr<double>(), data.data(), data.size() * sizeof(double));
    return tensor;
}

struct LoadedCsv
{
    torch::Tensor states;
    torch::Tensor actions;
    torch::Tensor timestamps;
};

static LoadedCsv load_csv(const std::string &csv_path)
{
    std::ifstream file(csv_path);
    if (!file.is_open()) throw std::runtime_error("Failed to open CSV file: " + csv_path);

    std::string header_line;
    if (!std::getline(file, header_line)) throw std::runtime_error("CSV file is empty: " + csv_path);

    auto headers = split_csv_line(header_line);
    std::unordered_map<std::string, int> header_index;
    for (int i = 0; i < static_cast<int>(headers.size()); ++i) header_index.emplace(headers[i], i);

    auto state_indices = lookup_indices(header_index, current_state_columns());
    auto action_indices = lookup_indices(header_index, action_columns());
    auto timestamp_it = header_index.find("timestamp_s");
    if (timestamp_it == header_index.end()) throw std::runtime_error("Missing CSV column: timestamp_s");
    const int timestamp_index = timestamp_it->second;

    std::vector<float> states_flat, actions_flat;
    std::vector<double> timestamps_flat;
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty()) continue;
        auto cells = split_csv_line(line);
        if (cells.size() != headers.size()) continue;

        try
        {
            timestamps_flat.push_back(std::stod(cells[timestamp_index]));
            for (int index : state_indices) states_flat.push_back(std::stof(cells[index]));
            for (int index : action_indices) actions_flat.push_back(std::stof(cells[index]));
        }
        catch (const std::exception &)
        {
            continue;
        }
    }

    const int64_t row_count = static_cast<int64_t>(timestamps_flat.size());
    if (row_count == 0) throw std::runtime_error("No valid rows were loaded from: " + csv_path);

    return {make_tensor_from_flat(states_flat, row_count, kFullStateDim),
            make_tensor_from_flat(actions_flat, row_count, kActionDim),
            make_double_tensor_from_flat(timestamps_flat, row_count, 1)};
}

static double wrap_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

static double yaw_from_row(const torch::Tensor &row)
{
    const double qw = row[IDX_QW].item<double>();
    const double qx = row[IDX_QX].item<double>();
    const double qy = row[IDX_QY].item<double>();
    const double qz = row[IDX_QZ].item<double>();
    return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

struct Normaliser
{
    torch::Tensor input_mean, input_std, action_mean, action_std, target_mean, target_std;

    torch::Tensor norm_input(const torch::Tensor &x) const { return (x - input_mean) / input_std; }
    torch::Tensor norm_action(const torch::Tensor &x) const { return (x - action_mean) / action_std; }
    torch::Tensor unnorm_target(const torch::Tensor &x) const { return x * target_std + target_mean; }

    static Normaliser load(const std::string &path)
    {
        std::ifstream in(path);
        if (!in.is_open()) throw std::runtime_error("Failed to open normaliser file: " + path);

        Normaliser normaliser;
        std::unordered_map<std::string, torch::Tensor> tensors;
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string name;
            ss >> name;
            std::vector<float> values;
            float value;
            while (ss >> value) values.push_back(value);
            auto tensor = torch::empty({static_cast<int64_t>(values.size())}, torch::TensorOptions().dtype(torch::kFloat32));
            std::memcpy(tensor.data_ptr<float>(), values.data(), values.size() * sizeof(float));
            tensors.emplace(std::move(name), std::move(tensor));
        }

        normaliser.input_mean = tensors.at("input_mean");
        normaliser.input_std = tensors.at("input_std");
        normaliser.action_mean = tensors.at("action_mean");
        normaliser.action_std = tensors.at("action_std");
        normaliser.target_mean = tensors.at("target_mean");
        normaliser.target_std = tensors.at("target_std");
        return normaliser;
    }
};

static void init_linear_weights(torch::nn::Module &module)
{
    if (auto *linear = module.as<torch::nn::Linear>())
    {
        torch::NoGradGuard no_grad;
        torch::nn::init::xavier_uniform_(linear->weight);
        torch::nn::init::constant_(linear->bias, 0.0);
    }
}

struct GaussianMLPImpl : torch::nn::Module
{
    GaussianMLPImpl(int64_t input_dim, int64_t output_dim, double dropout)
    {
        trunk = register_module("trunk", torch::nn::Sequential());
        trunk->push_back(torch::nn::Linear(input_dim, 256));
        trunk->push_back(torch::nn::LayerNorm(torch::nn::LayerNormOptions({256})));
        trunk->push_back(torch::nn::SiLU());
        trunk->push_back(torch::nn::Dropout(torch::nn::DropoutOptions(dropout)));
        trunk->push_back(torch::nn::Linear(256, 256));
        trunk->push_back(torch::nn::LayerNorm(torch::nn::LayerNormOptions({256})));
        trunk->push_back(torch::nn::SiLU());
        trunk->push_back(torch::nn::Dropout(torch::nn::DropoutOptions(dropout)));
        trunk->push_back(torch::nn::Linear(256, 128));
        trunk->push_back(torch::nn::LayerNorm(torch::nn::LayerNormOptions({128})));
        trunk->push_back(torch::nn::SiLU());
        trunk->push_back(torch::nn::Dropout(torch::nn::DropoutOptions(dropout)));
        mu_head = register_module("mu_head", torch::nn::Linear(128, output_dim));
        log_var_head = register_module("log_var_head", torch::nn::Linear(128, output_dim));
        apply(init_linear_weights);
    }

    std::pair<torch::Tensor, torch::Tensor> forward(torch::Tensor x)
    {
        auto h = trunk->forward(x);
        return {mu_head->forward(h), log_var_head->forward(h)};
    }

    torch::nn::Sequential trunk{nullptr};
    torch::nn::Linear mu_head{nullptr};
    torch::nn::Linear log_var_head{nullptr};
};

TORCH_MODULE(GaussianMLP);

struct RolloutState;
static torch::Tensor state_to_input_features(const RolloutState &state);

static torch::Tensor predict_delta(GaussianMLP &model,
                                   const Normaliser &normaliser,
                                   const RolloutState &state,
                                   const torch::Tensor &action_row,
                                   torch::Device device)
{
    auto input = state_to_input_features(state).unsqueeze(0);
    auto model_input = torch::cat(std::vector<torch::Tensor>{normaliser.norm_input(input), normaliser.norm_action(action_row.unsqueeze(0))}, 1).to(device);

    torch::NoGradGuard no_grad;
    auto [mu_norm, log_var_norm] = model->forward(model_input);
    (void)log_var_norm;
    return normaliser.unnorm_target(mu_norm.to(torch::kCPU)).squeeze(0);
}

struct RolloutState
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double oz = 0.0;
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
};

static torch::Tensor state_to_input_features(const RolloutState &state)
{
    const float qw = static_cast<float>(std::cos(state.yaw * 0.5));
    const float qz = static_cast<float>(std::sin(state.yaw * 0.5));
    std::vector<float> row = {
        qw, 0.0f, 0.0f, qz,
        static_cast<float>(state.vx),
        static_cast<float>(state.vy),
        static_cast<float>(state.oz)
    };
    for (double value : state.joint_pos) row.push_back(static_cast<float>(value));
    for (double value : state.joint_vel) row.push_back(static_cast<float>(value));
    return torch::tensor(row, torch::TensorOptions().dtype(torch::kFloat32));
}

static RolloutState state_from_row(const torch::Tensor &row)
{
    RolloutState state;
    state.x = row[IDX_X].item<double>();
    state.y = row[IDX_Y].item<double>();
    state.z = row[IDX_Z].item<double>();
    state.yaw = yaw_from_row(row);
    state.vx = row[IDX_VX].item<double>();
    state.vy = row[IDX_VY].item<double>();
    state.oz = row[IDX_OZ].item<double>();
    state.joint_pos.resize(kJointDim);
    state.joint_vel.resize(kJointDim);
    for (int i = 0; i < kJointDim; ++i)
    {
        state.joint_pos[i] = row[kRawStateDim + i].item<double>();
        state.joint_vel[i] = row[kRawStateDim + kJointDim + i].item<double>();
    }
    return state;
}

static void apply_delta(RolloutState &state, const torch::Tensor &delta)
{
    state.x += delta[0].item<double>();
    state.y += delta[1].item<double>();
    state.yaw = wrap_angle(state.yaw + delta[2].item<double>());
    state.vx += delta[3].item<double>();
    state.vy += delta[4].item<double>();
    state.oz += delta[5].item<double>();
    for (int i = 0; i < kJointDim; ++i)
        state.joint_pos[i] += delta[6 + i].item<double>();
    for (int i = 0; i < kJointDim; ++i)
        state.joint_vel[i] += delta[6 + kJointDim + i].item<double>();
}

struct Obstacle
{
    std::array<float, 3> position{};
    std::array<float, 2> size{};
    std::string type;
};

static std::vector<Obstacle> load_obstacles_from_scene(const std::string &scene_file)
{
    std::ifstream file(scene_file);
    if (!file.is_open()) throw std::runtime_error("Failed to open scene file: " + scene_file);

    std::vector<Obstacle> obstacles;
    std::string xml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    std::regex geom_pattern(R"(<geom[^>]*layout_[^>]*>)");
    auto begin = std::sregex_iterator(xml.begin(), xml.end(), geom_pattern);
    auto end = std::sregex_iterator();

    for (auto it = begin; it != end; ++it)
    {
        const std::string geom_block = it->str();

        bool found_type = false;
        bool found_pos = false;
        bool found_size = false;
        Obstacle obs;

        size_t type_pos = geom_block.find("type=\"");
        if (type_pos != std::string::npos)
        {
            size_t start = type_pos + 6;
            size_t end = geom_block.find('"', start);
            obs.type = geom_block.substr(start, end - start);
            found_type = true;
        }

        size_t pos_attr = geom_block.find("pos=\"");
        if (pos_attr != std::string::npos)
        {
            size_t start = pos_attr + 5;
            size_t end = geom_block.find('"', start);
            std::string pos_str = geom_block.substr(start, end - start);
            std::stringstream ss(pos_str);
            ss >> obs.position[0] >> obs.position[1] >> obs.position[2];
            found_pos = true;
        }

        size_t size_attr = geom_block.find("size=\"");
        if (size_attr != std::string::npos)
        {
            size_t start = size_attr + 6;
            size_t end = geom_block.find('"', start);
            std::string size_str = geom_block.substr(start, end - start);
            std::stringstream ss(size_str);
            ss >> obs.size[0] >> obs.size[1];
            found_size = true;
        }

        if (found_type && found_pos && found_size)
            obstacles.push_back(obs);
    }

    return obstacles;
}

static double obstacle_radius(const Obstacle &obs)
{
    if (obs.type.find("box") != std::string::npos)
        return std::sqrt(static_cast<double>(obs.size[0]) * static_cast<double>(obs.size[0]) +
                         static_cast<double>(obs.size[1]) * static_cast<double>(obs.size[1]));
    return static_cast<double>(obs.size[0]);
}

struct CollisionSphere
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double radius = 0.0;
};

static std::vector<CollisionSphere> collision_spheres_for_state(const RolloutState &state, double base_radius)
{
    return {CollisionSphere{state.x, state.y, state.z, base_radius}};
}

static bool sphere_collides_with_obstacle(const CollisionSphere &sphere, const Obstacle &obstacle, double clearance)
{
    const double dx = sphere.x - static_cast<double>(obstacle.position[0]);
    const double dy = sphere.y - static_cast<double>(obstacle.position[1]);
    const double distance_xy = std::sqrt(dx * dx + dy * dy);
    return distance_xy <= (sphere.radius + obstacle_radius(obstacle) + clearance);
}

static bool state_in_collision(const RolloutState &state,
                               const std::vector<Obstacle> &obstacles,
                               double base_radius,
                               double clearance)
{
    for (const auto &sphere : collision_spheres_for_state(state, base_radius))
    {
        for (const auto &obstacle : obstacles)
        {
            if (sphere_collides_with_obstacle(sphere, obstacle, clearance))
                return true;
        }
    }
    return false;
}

struct ActionSample
{
    double vx = 0.0;
    double vy = 0.0;
    double yaw = 0.0;
};

static torch::Tensor action_to_tensor(const ActionSample &action)
{
    return torch::tensor({static_cast<float>(action.vx), static_cast<float>(action.vy), static_cast<float>(action.yaw)},
                         torch::TensorOptions().dtype(torch::kFloat32));
}

static ActionSample sample_action(std::mt19937 &rng, const std::array<double, 3> &limits)
{
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    return ActionSample{unit(rng) * limits[0], unit(rng) * limits[1], unit(rng) * limits[2]};
}

struct PlanResult
{
    double cost = std::numeric_limits<double>::infinity();
    bool collision = false;
    std::vector<ActionSample> actions;
    std::vector<RolloutState> states;
};

static PlanResult evaluate_sequence(GaussianMLP &model,
                                    const Normaliser &normaliser,
                                    const RolloutState &initial_state,
                                    const std::vector<ActionSample> &actions,
                                    const std::vector<Obstacle> &obstacles,
                                    torch::Device device,
                                    double goal_x,
                                    double goal_y,
                                    double base_radius,
                                    double clearance,
                                    double collision_penalty,
                                    double action_penalty,
                                    double smoothness_penalty,
                                    double goal_weight,
                                    bool sample_dynamics)
{
    (void)sample_dynamics;
    PlanResult result;
    result.cost = 0.0;
    result.actions = actions;
    result.states.reserve(actions.size() + 1);
    result.states.push_back(initial_state);

    RolloutState current = initial_state;
    ActionSample previous_action{};

    for (std::size_t step = 0; step < actions.size(); ++step)
    {
        const auto &action = actions[step];
        auto delta = predict_delta(model, normaliser, current, action_to_tensor(action), device);
        apply_delta(current, delta);
        result.states.push_back(current);

        const double dx_goal = current.x - goal_x;
        const double dy_goal = current.y - goal_y;
        result.cost += goal_weight * (dx_goal * dx_goal + dy_goal * dy_goal);

        const double action_norm = action.vx * action.vx + action.vy * action.vy + action.yaw * action.yaw;
        result.cost += action_penalty * action_norm;

        if (step > 0)
        {
            const double dvx = action.vx - previous_action.vx;
            const double dvy = action.vy - previous_action.vy;
            const double dyaw = action.yaw - previous_action.yaw;
            result.cost += smoothness_penalty * (dvx * dvx + dvy * dvy + dyaw * dyaw);
        }

        if (state_in_collision(current, obstacles, base_radius, clearance))
        {
            result.cost += collision_penalty;
            result.collision = true;
            break;
        }

        previous_action = action;
    }

    return result;
}

static PlanResult random_shoot_mpc(GaussianMLP &model,
                                   const Normaliser &normaliser,
                                   const RolloutState &initial_state,
                                   const std::vector<Obstacle> &obstacles,
                                   torch::Device device,
                                   double goal_x,
                                   double goal_y,
                                   int horizon,
                                   int num_candidates,
                                   const std::array<double, 3> &action_limits,
                                   double base_radius,
                                   double clearance,
                                   double collision_penalty,
                                   double action_penalty,
                                   double smoothness_penalty,
                                   double goal_weight,
                                   bool sample_dynamics,
                                   unsigned int seed)
{
    std::mt19937 rng(seed);
    PlanResult best;

    for (int candidate = 0; candidate < num_candidates; ++candidate)
    {
        std::vector<ActionSample> actions;
        actions.reserve(horizon);
        for (int step = 0; step < horizon; ++step)
            actions.push_back(sample_action(rng, action_limits));

        auto result = evaluate_sequence(model, normaliser, initial_state, actions, obstacles, device,
                                        goal_x, goal_y, base_radius, clearance,
                                        collision_penalty, action_penalty, smoothness_penalty,
                                        goal_weight, sample_dynamics);
        if (result.cost < best.cost)
            best = std::move(result);
    }

    return best;
}

int main(int argc, char **argv)
{
    po::options_description desc("MPC planner over the learned transition model");
    desc.add_options()
        ("help,h", "show help")
        ("csv", po::value<std::string>()->required(), "path to transitions.csv")
        ("checkpoint", po::value<std::string>()->required(), "path to a Gaussian transition checkpoint")
        ("normaliser", po::value<std::string>()->default_value(""), "normaliser.txt path (defaults to checkpoint directory)")
        ("scene", po::value<std::string>()->default_value("../config/scene/scene.xml"), "scene xml used for collision checking")
        ("start-row", po::value<int>()->default_value(0), "CSV row used as the initial state")
        ("goal-row", po::value<int>()->default_value(-1), "CSV row used as the goal state when goal-x/goal-y are not provided")
        ("goal-x", po::value<std::string>()->default_value(""), "override goal x coordinate")
        ("goal-y", po::value<std::string>()->default_value(""), "override goal y coordinate")
        ("horizon", po::value<int>()->default_value(10), "MPC horizon length")
        ("candidates", po::value<int>()->default_value(256), "number of sampled action sequences")
        ("base-radius", po::value<double>()->default_value(0.35), "base footprint radius used for collision checking")
        ("clearance", po::value<double>()->default_value(0.05), "extra safety clearance around obstacles")
        ("collision-penalty", po::value<double>()->default_value(1000.0), "penalty applied to colliding rollouts")
        ("action-penalty", po::value<double>()->default_value(0.01), "quadratic action penalty")
        ("smoothness-penalty", po::value<double>()->default_value(0.02), "quadratic action change penalty")
        ("goal-weight", po::value<double>()->default_value(1.0), "quadratic goal distance weight")
        ("seed", po::value<int>()->default_value(42), "random seed")
        ("sample-dynamics", po::bool_switch()->default_value(false), "reserved for future stochastic dynamics sampling");

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) { std::cout << desc << std::endl; return 0; }
        po::notify(vm);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n' << desc << std::endl;
        return 1;
    }

    const std::string csv_path = vm["csv"].as<std::string>();
    const std::string checkpoint_path = vm["checkpoint"].as<std::string>();
    const int start_row = vm["start-row"].as<int>();
    const int goal_row_option = vm["goal-row"].as<int>();
    const int horizon = vm["horizon"].as<int>();
    const int candidates = vm["candidates"].as<int>();
    const double base_radius = vm["base-radius"].as<double>();
    const double clearance = vm["clearance"].as<double>();
    const double collision_penalty = vm["collision-penalty"].as<double>();
    const double action_penalty = vm["action-penalty"].as<double>();
    const double smoothness_penalty = vm["smoothness-penalty"].as<double>();
    const double goal_weight = vm["goal-weight"].as<double>();
    const bool sample_dynamics = vm["sample-dynamics"].as<bool>();
    const unsigned int seed = static_cast<unsigned int>(vm["seed"].as<int>());

    auto data = load_csv(csv_path);
    const int64_t row_count = data.states.size(0);
    if (start_row < 0 || start_row >= row_count)
    {
        std::cerr << "start-row is out of range\n";
        return 1;
    }

    double goal_x = 0.0;
    double goal_y = 0.0;
    const std::string goal_x_override = vm["goal-x"].as<std::string>();
    const std::string goal_y_override = vm["goal-y"].as<std::string>();
    if (!goal_x_override.empty() && !goal_y_override.empty())
    {
        goal_x = std::stod(goal_x_override);
        goal_y = std::stod(goal_y_override);
    }
    else
    {
        int goal_row = goal_row_option;
        if (goal_row < 0) goal_row = static_cast<int>(row_count) - 1;
        if (goal_row < 0 || goal_row >= row_count)
        {
            std::cerr << "goal-row is out of range\n";
            return 1;
        }
        goal_x = data.states[goal_row][IDX_X].item<double>();
        goal_y = data.states[goal_row][IDX_Y].item<double>();
    }

    std::string normaliser_path = vm["normaliser"].as<std::string>();
    if (normaliser_path.empty())
    {
        const auto slash = checkpoint_path.find_last_of('/');
        normaliser_path = (slash == std::string::npos ? std::string(".") : checkpoint_path.substr(0, slash)) + "/normaliser.txt";
    }

    std::vector<Obstacle> obstacles;
    try
    {
        obstacles = load_obstacles_from_scene(vm["scene"].as<std::string>());
    }
    catch (const std::exception &e)
    {
        std::cerr << "Scene load warning: " << e.what() << '\n';
    }

    const torch::Device device(torch::kCPU);
    std::cout << "Using device: " << device << '\n';
    std::cout << "Loaded " << row_count << " rows from " << csv_path << '\n';
    std::cout << "Loaded " << obstacles.size() << " obstacles from " << vm["scene"].as<std::string>() << '\n';

    Normaliser normaliser = Normaliser::load(normaliser_path);
    GaussianMLP model(kInputDim, kOutputDim, 0.2);
    model->to(device);
    torch::load(model, checkpoint_path, torch::kCPU);
    model->eval();

    const RolloutState initial_state = state_from_row(data.states[start_row]);
    const std::array<double, 3> action_limits = {0.5, 0.3, 0.2};
    auto best = random_shoot_mpc(model, normaliser, initial_state, obstacles, device,
                                 goal_x, goal_y, horizon, candidates, action_limits,
                                 base_radius, clearance, collision_penalty,
                                 action_penalty, smoothness_penalty, goal_weight,
                                 sample_dynamics, seed);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Start row: " << start_row << "\n";
    std::cout << "Goal: (" << goal_x << ", " << goal_y << ")\n";
    std::cout << "Best cost: " << best.cost << "\n";
    std::cout << "Collision: " << (best.collision ? "yes" : "no") << "\n";
    if (!best.actions.empty())
    {
        const auto &first = best.actions.front();
        std::cout << "First action: vx=" << first.vx << " vy=" << first.vy << " yaw=" << first.yaw << "\n";
    }

    for (std::size_t i = 0; i < best.states.size(); ++i)
    {
        const auto &state = best.states[i];
        std::cout << "step " << i << ": x=" << state.x << " y=" << state.y << " yaw=" << state.yaw << '\n';
    }

    return 0;
}