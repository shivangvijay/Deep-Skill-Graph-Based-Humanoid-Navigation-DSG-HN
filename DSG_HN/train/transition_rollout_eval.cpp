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
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace po = boost::program_options;

static constexpr int kBaseStateDim = 13;
static constexpr int kJointDim = 35;
static constexpr int kStateDim = kBaseStateDim + (2 * kJointDim);
static constexpr int kActionDim = 3;
static constexpr int kInputDim = kStateDim + kActionDim;
static constexpr double kBoundaryGapSeconds = 0.15;

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

static std::vector<std::string> action_columns() { return {"cmd_vx", "cmd_vy", "cmd_yaw"}; }

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

    return {make_tensor_from_flat(states_flat, row_count, kStateDim),
            make_tensor_from_flat(actions_flat, row_count, kActionDim),
            make_double_tensor_from_flat(timestamps_flat, row_count, 1)};
}

struct Normaliser
{
    torch::Tensor state_mean;
    torch::Tensor state_std;
    torch::Tensor action_mean;
    torch::Tensor action_std;
    torch::Tensor target_mean;
    torch::Tensor target_std;

    static Normaliser load(const std::filesystem::path &path)
    {
        std::ifstream in(path);
        if (!in.is_open()) throw std::runtime_error("Failed to open normaliser file: " + path.string());

        Normaliser n;
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

        n.state_mean = tensors.at("state_mean");
        n.state_std = tensors.at("state_std");
        n.action_mean = tensors.at("action_mean");
        n.action_std = tensors.at("action_std");
        n.target_mean = tensors.at("target_mean");
        n.target_std = tensors.at("target_std");
        return n;
    }

    torch::Tensor norm_state(const torch::Tensor &x) const { return (x - state_mean) / state_std; }
    torch::Tensor norm_action(const torch::Tensor &x) const { return (x - action_mean) / action_std; }
    torch::Tensor unnorm_target(const torch::Tensor &x) const { return x * target_std + target_mean; }
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

struct DeterministicTransitionMLPImpl : torch::nn::Module
{
    DeterministicTransitionMLPImpl(int64_t input_dim, int64_t output_dim)
    {
        net = register_module("net", torch::nn::Sequential(
            torch::nn::Linear(input_dim, 256), torch::nn::LayerNorm(torch::nn::LayerNormOptions({256})), torch::nn::SiLU(), torch::nn::Dropout(0.2),
            torch::nn::Linear(256, 256), torch::nn::LayerNorm(torch::nn::LayerNormOptions({256})), torch::nn::SiLU(), torch::nn::Dropout(0.2),
            torch::nn::Linear(256, 128), torch::nn::LayerNorm(torch::nn::LayerNormOptions({128})), torch::nn::SiLU(), torch::nn::Dropout(0.2),
            torch::nn::Linear(128, output_dim)));
        apply(init_linear_weights);
    }

    torch::Tensor forward(torch::Tensor x) { return net->forward(x); }
    torch::nn::Sequential net{nullptr};
};

struct GaussianTransitionMLPImpl : torch::nn::Module
{
    GaussianTransitionMLPImpl(int64_t input_dim, int64_t output_dim)
    {
        trunk = register_module("trunk", torch::nn::Sequential(
            torch::nn::Linear(input_dim, 256), torch::nn::LayerNorm(torch::nn::LayerNormOptions({256})), torch::nn::SiLU(), torch::nn::Dropout(0.2),
            torch::nn::Linear(256, 256), torch::nn::LayerNorm(torch::nn::LayerNormOptions({256})), torch::nn::SiLU(), torch::nn::Dropout(0.2),
            torch::nn::Linear(256, 128), torch::nn::LayerNorm(torch::nn::LayerNormOptions({128})), torch::nn::SiLU(), torch::nn::Dropout(0.2)));
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

TORCH_MODULE(DeterministicTransitionMLP);
TORCH_MODULE(GaussianTransitionMLP);

static double wrap_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

static double yaw_from_quat(const torch::Tensor &row)
{
    const double qw = row[3].item<double>();
    const double qx = row[4].item<double>();
    const double qy = row[5].item<double>();
    const double qz = row[6].item<double>();
    return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

static torch::Tensor make_output_weights(torch::Device device)
{
    auto weights = torch::ones({kStateDim}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
    weights[0] = 2.0f;
    weights[1] = 2.0f;
    for (int i = kBaseStateDim; i < kStateDim; ++i) weights[i] = 0.5f;
    return weights;
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

struct Metrics
{
    double pos_xy = 0.0;
    double vel_xy = 0.0;
    double heading = 0.0;
    double omega_z = 0.0;
    double joint_pos_sq = 0.0;
    double joint_vel_sq = 0.0;
    int count = 0;
};

static void accumulate_metrics(Metrics &metrics, const torch::Tensor &pred, const torch::Tensor &gt)
{
    const double dx = pred[0].item<double>() - gt[0].item<double>();
    const double dy = pred[1].item<double>() - gt[1].item<double>();
    const double dvx = pred[7].item<double>() - gt[7].item<double>();
    const double dvy = pred[8].item<double>() - gt[8].item<double>();
    const double yaw_pred = yaw_from_quat(pred);
    const double yaw_gt = yaw_from_quat(gt);
    const double heading_error = wrap_angle(yaw_pred - yaw_gt);
    const double omega_z_error = pred[12].item<double>() - gt[12].item<double>();

    for (int i = 0; i < kJointDim; ++i)
    {
        const int pos_idx = kBaseStateDim + i;
        const int vel_idx = kBaseStateDim + kJointDim + i;
        const double pos_err = pred[pos_idx].item<double>() - gt[pos_idx].item<double>();
        const double vel_err = pred[vel_idx].item<double>() - gt[vel_idx].item<double>();
        metrics.joint_pos_sq += pos_err * pos_err;
        metrics.joint_vel_sq += vel_err * vel_err;
    }

    metrics.pos_xy += std::sqrt(dx * dx + dy * dy);
    metrics.vel_xy += std::sqrt(dvx * dvx + dvy * dvy);
    metrics.heading += std::abs(heading_error);
    metrics.omega_z += std::abs(omega_z_error);
    ++metrics.count;
}

template <typename ModelType>
static torch::Tensor predict_next_state(ModelType &model,
                                       const Normaliser &normaliser,
                                       const torch::Tensor &state,
                                       const torch::Tensor &action,
                                       torch::Device device,
                                       bool sample)
{
    auto input = torch::cat({normaliser.norm_state(state.unsqueeze(0)), normaliser.norm_action(action.unsqueeze(0))}, 1).to(device);
    torch::NoGradGuard no_grad;

    if constexpr (std::is_same_v<ModelType, DeterministicTransitionMLP>)
    {
        auto pred_norm = model->forward(input).to(torch::kCPU);
        return normaliser.unnorm_target(pred_norm).squeeze(0);
    }
    else
    {
        auto [mu_norm, log_var] = model->forward(input);
        auto mu = normaliser.unnorm_target(mu_norm.to(torch::kCPU));
        auto var = torch::exp(torch::clamp(log_var.to(torch::kCPU), -4.0, 4.0));
        if (sample)
            return (mu + torch::sqrt(var) * torch::randn_like(mu)).squeeze(0);
        return mu.squeeze(0);
    }
}

template <typename ModelType>
static Metrics evaluate_horizon(ModelType &model,
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
        boundary[i] = (data.timestamps[i].item<float>() - data.timestamps[i - 1].item<float>()) > kBoundaryGapSeconds;

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

        auto current = data.states[start];
        for (int64_t i = start; i < end; ++i)
        {
            current = predict_next_state(model, normaliser, current, data.actions[i], device, sample);
        }

        accumulate_metrics(metrics, current, data.states[end]);
    }

    return metrics;
}

int main(int argc, char **argv)
{
    po::options_description desc("Transition rollout evaluation");
    desc.add_options()
        ("help,h", "show help")
        ("csv", po::value<std::string>()->required(), "path to transitions.csv")
        ("checkpoint", po::value<std::string>()->required(), "path to a checkpoint file")
        ("normaliser", po::value<std::string>()->default_value(""), "normaliser.txt path (defaults to checkpoint directory)")
        ("model-type", po::value<std::string>()->default_value("deterministic"), "deterministic or gaussian")
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
    const std::string model_type = vm["model-type"].as<std::string>();
    const std::vector<double> horizons = parse_horizons(vm["horizons"].as<std::string>());
    const bool sample = vm["sample"].as<bool>();
    torch::manual_seed(vm["seed"].as<int>());

    if (model_type != "deterministic" && model_type != "gaussian")
    {
        std::cerr << "--model-type must be deterministic or gaussian\n";
        return 1;
    }

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
        data.timestamps.narrow(0, test_start, test_count).clone(),
    };

    std::filesystem::path normaliser_path = vm["normaliser"].as<std::string>();
    if (normaliser_path.empty())
        normaliser_path = std::filesystem::path(checkpoint_path).parent_path() / "normaliser.txt";
    Normaliser normaliser = Normaliser::load(normaliser_path);

    const torch::Device device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);
    std::cout << "Using device: " << device << '\n';
    std::cout << "Evaluating on sequential test split of " << test_count << " rows\n";

    if (model_type == "deterministic")
    {
        DeterministicTransitionMLP model(kInputDim, kStateDim);
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
            std::cout << "  Joint pos RMSE = " << std::sqrt(metrics.joint_pos_sq / (metrics.count * kJointDim)) << " rad\n";
            std::cout << "  Joint vel RMSE = " << std::sqrt(metrics.joint_vel_sq / (metrics.count * kJointDim)) << " rad/s\n";
        }
    }
    else
    {
        GaussianTransitionMLP model(kInputDim, kStateDim);
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
            std::cout << "  Joint pos RMSE = " << std::sqrt(metrics.joint_pos_sq / (metrics.count * kJointDim)) << " rad\n";
            std::cout << "  Joint vel RMSE = " << std::sqrt(metrics.joint_vel_sq / (metrics.count * kJointDim)) << " rad/s\n";
        }
    }

    return 0;
}