#include <torch/torch.h>

#include <boost/program_options.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
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
static constexpr int kInputStateDim = 7;
static constexpr int kActionDim = 3;
static constexpr int kInputDim = kInputStateDim + kActionDim;
static constexpr int kOutputDim = 6;
static constexpr double kBoundaryGapSeconds = 0.15;

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

static std::vector<std::string> next_state_columns()
{
    std::vector<std::string> columns = {"next_x", "next_y", "next_z", "next_qw", "next_qx", "next_qy", "next_qz", "next_vx", "next_vy", "next_vz", "next_omega_x", "next_omega_y", "next_omega_z"};
    for (int i = 0; i < kJointDim; ++i) columns.push_back("next_joint_pos_" + two_digit(i));
    for (int i = 0; i < kJointDim; ++i) columns.push_back("next_joint_vel_" + two_digit(i));
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
    torch::Tensor next_states;
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
    auto next_state_indices = lookup_indices(header_index, next_state_columns());
    auto timestamp_it = header_index.find("timestamp_s");
    if (timestamp_it == header_index.end()) throw std::runtime_error("Missing CSV column: timestamp_s");
    const int timestamp_index = timestamp_it->second;

    std::vector<float> states_flat, actions_flat, next_states_flat;
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
            for (int index : next_state_indices) next_states_flat.push_back(std::stof(cells[index]));
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
            make_tensor_from_flat(next_states_flat, row_count, kFullStateDim),
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

struct RolloutState;
static torch::Tensor rollout_state_to_input(const RolloutState &state);

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

static torch::Tensor predict_delta(GaussianMLP &model,
                                   const Normaliser &normaliser,
                                   const RolloutState &state,
                                   const torch::Tensor &action_row,
                                   torch::Device device,
                                   bool sample)
{
    auto input = rollout_state_to_input(state);
    auto model_input = torch::cat(std::vector<torch::Tensor>{
        normaliser.norm_input(input),
        normaliser.norm_action(action_row.unsqueeze(0))
    }, 1).to(device);

    torch::NoGradGuard no_grad;
    auto [mu_norm, log_var_norm] = model->forward(model_input);
    if (sample)
    {
        auto clamped = torch::clamp(log_var_norm, -4.0, 4.0);
        auto noise_std = torch::exp(0.5 * clamped);
        auto sampled = mu_norm + noise_std * torch::randn_like(mu_norm);
        return normaliser.unnorm_target(sampled.to(torch::kCPU)).squeeze(0);
    }
    return normaliser.unnorm_target(mu_norm.to(torch::kCPU)).squeeze(0);
}

struct RolloutState
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double oz = 0.0;
};

static RolloutState state_from_row(const torch::Tensor &row)
{
    RolloutState state;
    state.x = row[IDX_X].item<double>();
    state.y = row[IDX_Y].item<double>();
    state.yaw = yaw_from_row(row);
    state.vx = row[IDX_VX].item<double>();
    state.vy = row[IDX_VY].item<double>();
    state.oz = row[IDX_OZ].item<double>();
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
}

static torch::Tensor rollout_state_to_input(const RolloutState &state)
{
    const float qw = static_cast<float>(std::cos(state.yaw * 0.5));
    const float qz = static_cast<float>(std::sin(state.yaw * 0.5));
    return torch::tensor({qw, 0.0f, 0.0f, qz,
                          static_cast<float>(state.vx),
                          static_cast<float>(state.vy),
                          static_cast<float>(state.oz)}, torch::TensorOptions().dtype(torch::kFloat32)).unsqueeze(0);
}

struct Metrics
{
    double pos_xy = 0.0;
    double vel_xy = 0.0;
    double heading = 0.0;
    double omega_z = 0.0;
    int count = 0;
};

static void accumulate_metrics(Metrics &metrics, const RolloutState &pred, const RolloutState &gt)
{
    const double dx = pred.x - gt.x;
    const double dy = pred.y - gt.y;
    const double dvx = pred.vx - gt.vx;
    const double dvy = pred.vy - gt.vy;

    metrics.pos_xy += std::sqrt(dx * dx + dy * dy);
    metrics.vel_xy += std::sqrt(dvx * dvx + dvy * dvy);
    metrics.heading += std::abs(wrap_angle(pred.yaw - gt.yaw));
    metrics.omega_z += std::abs(pred.oz - gt.oz);
    ++metrics.count;
}

static std::vector<double> parse_horizons(const std::string &value)
{
    std::vector<double> horizons;
    std::stringstream ss(value);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        if (!token.empty()) horizons.push_back(std::stod(token));
    }
    if (horizons.empty()) throw std::runtime_error("No valid horizons were provided");
    return horizons;
}

static Metrics evaluate_horizon(GaussianMLP &model,
                                const LoadedCsv &data,
                                const Normaliser &normaliser,
                                double horizon_s,
                                torch::Device device,
                                bool sample)
{
    Metrics metrics;
    const int64_t n = data.states.size(0);

    std::vector<bool> boundary(n, false);
    boundary[0] = true;
    for (int64_t i = 1; i < n; ++i)
        boundary[i] = (data.timestamps[i].item<double>() - data.timestamps[i - 1].item<double>()) > kBoundaryGapSeconds;

    for (int64_t start = 0; start < n; ++start)
    {
        const double start_time = data.timestamps[start].item<double>();
        int64_t end = start + 1;
        while (end < n && (data.timestamps[end].item<double>() - start_time) < horizon_s)
            ++end;

        if (end >= n)
            break;

        bool crosses_boundary = false;
        for (int64_t i = start + 1; i <= end; ++i)
        {
            if (boundary[i])
            {
                crosses_boundary = true;
                break;
            }
        }
        if (crosses_boundary)
            continue;

        RolloutState current = state_from_row(data.states[start]);
        for (int64_t i = start; i < end; ++i)
        {
            auto delta = predict_delta(model, normaliser, current, data.actions[i], device, sample);
            apply_delta(current, delta);
        }

        accumulate_metrics(metrics, current, state_from_row(data.states[end]));
    }

    return metrics;
}

int main(int argc, char **argv)
{
    po::options_description desc("Gaussian delta transition rollout evaluation");
    desc.add_options()
        ("help,h", "show help")
        ("csv", po::value<std::string>()->required(), "path to transitions.csv")
        ("checkpoint", po::value<std::string>()->required(), "path to a checkpoint file")
        ("normaliser", po::value<std::string>()->default_value(""), "normaliser.txt path (defaults to checkpoint directory)")
        ("horizons", po::value<std::string>()->default_value("1,5"), "comma-separated rollout horizons in seconds")
        ("sample", po::bool_switch()->default_value(false), "sample from Gaussian predictions instead of using the mean")
        ("seed", po::value<int>()->default_value(42), "random seed");

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
    const std::vector<double> horizons = parse_horizons(vm["horizons"].as<std::string>());
    const bool sample = vm["sample"].as<bool>();
    torch::manual_seed(vm["seed"].as<int>());

    auto data = load_csv(csv_path);
    const int64_t row_count = data.states.size(0);
    const int64_t train_count = static_cast<int64_t>(row_count * 0.70);
    const int64_t val_count = static_cast<int64_t>(row_count * 0.20);
    const int64_t test_start = train_count + val_count;
    const int64_t test_count = row_count - test_start;
    if (test_count <= 0)
    {
        std::cerr << "Not enough data for a sequential test split\n";
        return 1;
    }

    LoadedCsv test_data{
        data.states.narrow(0, test_start, test_count).clone(),
        data.actions.narrow(0, test_start, test_count).clone(),
        data.next_states.narrow(0, test_start, test_count).clone(),
        data.timestamps.narrow(0, test_start, test_count).clone(),
    };

    std::string normaliser_path = vm["normaliser"].as<std::string>();
    if (normaliser_path.empty())
    {
        const auto slash = checkpoint_path.find_last_of('/');
        normaliser_path = (slash == std::string::npos ? std::string(".") : checkpoint_path.substr(0, slash)) + "/normaliser.txt";
    }
    Normaliser normaliser = Normaliser::load(normaliser_path);

    const torch::Device device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);
    std::cout << "Using device: " << device << '\n';
    std::cout << "Evaluating on sequential test split of " << test_count << " rows\n";

    GaussianMLP model(kInputDim, kOutputDim, 0.2);
    model->to(device);
    torch::load(model, checkpoint_path);
    model->eval();

    for (double horizon : horizons)
    {
        auto metrics = evaluate_horizon(model, test_data, normaliser, horizon, device, sample);
        if (metrics.count == 0)
        {
            std::cout << "Rollout @ " << horizon << "s: no valid windows\n";
            continue;
        }

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Rollout @ " << horizon << "s (" << metrics.count << " windows)\n";
        std::cout << "  XY error      = " << (metrics.pos_xy / metrics.count) << " m\n";
        std::cout << "  Vel error     = " << (metrics.vel_xy / metrics.count) << " m/s\n";
        std::cout << "  Heading error = " << (metrics.heading / metrics.count) << " rad\n";
        std::cout << "  Omega_z error = " << (metrics.omega_z / metrics.count) << " rad/s\n";
    }

    return 0;
}
