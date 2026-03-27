#include <torch/torch.h>

#include <boost/program_options.hpp>

#include <algorithm>
#include <cstring>
#include <filesystem>
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

static constexpr int kBaseStateDim = 13;
static constexpr int kJointDim = 35;
static constexpr int kStateDim = kBaseStateDim + (2 * kJointDim);
static constexpr int kActionDim = 3;
static constexpr int kInputDim = kStateDim + kActionDim;

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

struct LoadedCsv
{
    torch::Tensor states, actions, next_states, timestamps;
    std::vector<std::vector<std::string>> rows;
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

    std::vector<float> states_flat, actions_flat, next_states_flat, timestamps_flat;
    std::vector<std::vector<std::string>> rows;

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty()) continue;
        auto cells = split_csv_line(line);
        if (cells.size() != headers.size()) continue;

        try
        {
            timestamps_flat.push_back(std::stof(cells[timestamp_index]));
            for (int index : state_indices) states_flat.push_back(std::stof(cells[index]));
            for (int index : action_indices) actions_flat.push_back(std::stof(cells[index]));
            for (int index : next_state_indices) next_states_flat.push_back(std::stof(cells[index]));
            rows.push_back(std::move(cells));
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
            make_tensor_from_flat(next_states_flat, row_count, kStateDim),
            make_tensor_from_flat(timestamps_flat, row_count, 1),
            std::move(rows)};
}

struct Normaliser
{
    torch::Tensor state_mean, state_std, action_mean, action_std, target_mean, target_std;

    static Normaliser load(const std::filesystem::path &path)
    {
        std::ifstream in(path);
        if (!in.is_open()) throw std::runtime_error("Failed to open normaliser file: " + path.string());

        Normaliser n;
        auto parse_line = [](const std::string &line) {
            std::stringstream ss(line);
            std::string name;
            ss >> name;
            std::vector<float> values;
            float value;
            while (ss >> value) values.push_back(value);
            return std::pair<std::string, std::vector<float>>{std::move(name), std::move(values)};
        };

        std::unordered_map<std::string, torch::Tensor> tensors;
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty()) continue;
            auto [name, values] = parse_line(line);
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

static torch::Tensor make_output_weights(torch::Device device)
{
    auto weights = torch::ones({kStateDim}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
    weights[0] = 2.0f;
    weights[1] = 2.0f;
    for (int i = kBaseStateDim; i < kStateDim; ++i) weights[i] = 0.5f;
    return weights;
}

static torch::Tensor weighted_mse_loss(const torch::Tensor &pred, const torch::Tensor &target, const torch::Tensor &weights)
{
    return ((pred - target).pow(2) * weights).mean();
}

static torch::Tensor gaussian_nll_loss(const torch::Tensor &mu, const torch::Tensor &log_var, const torch::Tensor &target, const torch::Tensor &weights)
{
    auto clamped = torch::clamp(log_var, -4.0, 4.0);
    auto var = torch::exp(clamped);
    auto nll = 0.5 * (clamped + (target - mu).pow(2) / var);
    return (nll * weights).mean();
}

int main(int argc, char **argv)
{
    po::options_description desc("Transition inference");
    desc.add_options()
        ("help,h", "show help")
        ("csv", po::value<std::string>()->required(), "path to transitions.csv")
        ("checkpoint", po::value<std::string>()->required(), "path to a checkpoint file")
        ("normaliser", po::value<std::string>()->default_value(""), "normaliser.txt path (defaults to checkpoint directory)")
        ("model-type", po::value<std::string>()->default_value("deterministic"), "deterministic or gaussian")
        ("row", po::value<int>()->default_value(0), "row index in the CSV to evaluate")
        ("sample", po::bool_switch()->default_value(false), "sample from the Gaussian output instead of using the mean")
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
    const int row_index = vm["row"].as<int>();
    const bool sample = vm["sample"].as<bool>();
    torch::manual_seed(vm["seed"].as<int>());

    if (model_type != "deterministic" && model_type != "gaussian")
    {
        std::cerr << "--model-type must be deterministic or gaussian\n";
        return 1;
    }

    auto data = load_csv(csv_path);
    if (row_index < 0 || row_index >= static_cast<int>(data.states.size(0)))
    {
        std::cerr << "row index out of range\n";
        return 1;
    }

    std::filesystem::path normaliser_path = vm["normaliser"].as<std::string>();
    if (normaliser_path.empty())
        normaliser_path = std::filesystem::path(checkpoint_path).parent_path() / "normaliser.txt";

    Normaliser normaliser = Normaliser::load(normaliser_path);

    auto state = data.states[row_index].unsqueeze(0);
    auto action = data.actions[row_index].unsqueeze(0);
    auto next_state_gt = data.next_states[row_index].unsqueeze(0);
    auto input = torch::cat({normaliser.norm_state(state), normaliser.norm_action(action)}, 1);

    const torch::Device device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);

    if (model_type == "deterministic")
    {
        DeterministicTransitionMLP model(kInputDim, kStateDim);
        model->to(device);
        torch::load(model, checkpoint_path);
        model->eval();

        torch::NoGradGuard no_grad;
        auto pred_norm = model->forward(input.to(device)).to(torch::kCPU);
        auto pred = normaliser.unnorm_target(pred_norm);
        auto mse = weighted_mse_loss(pred, next_state_gt, torch::ones({kStateDim})).item<float>();

        std::cout << "Predicted next state (deterministic)\n";
        std::cout << "x y z qw qx qy qz vx vy vz omega_x omega_y omega_z ...\n";
        for (int i = 0; i < kStateDim; ++i)
        {
            std::cout << pred.squeeze(0)[i].item<float>() << (i + 1 < kStateDim ? ' ' : '\n');
        }
        std::cout << "Row MSE vs ground truth: " << mse << '\n';
    }
    else
    {
        GaussianTransitionMLP model(kInputDim, kStateDim);
        model->to(device);
        torch::load(model, checkpoint_path);
        model->eval();

        torch::NoGradGuard no_grad;
        auto [mu_norm, log_var] = model->forward(input.to(device));
        auto mu = normaliser.unnorm_target(mu_norm.to(torch::kCPU));
        auto var = torch::exp(torch::clamp(log_var.to(torch::kCPU), -4.0, 4.0));

        torch::Tensor chosen = mu;
        if (sample)
        {
            auto std = torch::sqrt(var);
            chosen = mu + std * torch::randn_like(mu);
        }

        std::cout << "Predicted next state (gaussian, " << (sample ? "sample" : "mean") << ")\n";
        for (int i = 0; i < kStateDim; ++i)
        {
            std::cout << chosen.squeeze(0)[i].item<float>() << (i + 1 < kStateDim ? ' ' : '\n');
        }
        std::cout << "Per-dimension variance (normalised space):\n";
        for (int i = 0; i < kStateDim; ++i)
        {
            std::cout << var.squeeze(0)[i].item<float>() << (i + 1 < kStateDim ? ' ' : '\n');
        }
    }

    return 0;
}