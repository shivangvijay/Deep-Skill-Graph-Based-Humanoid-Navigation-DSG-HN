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

static constexpr int kRawStateDim = 13;
static constexpr int kJointDim = 35;
static constexpr int kFullStateDim = kRawStateDim + (2 * kJointDim);
static constexpr int kInputStateDim = 7;
static constexpr int kActionDim = 3;
static constexpr int kInputDim = kInputStateDim + kActionDim;
static constexpr int kOutputDim = 6;

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

static torch::Tensor state_to_input_gaussian(const torch::Tensor &states)
{
    return torch::stack({states.select(1, IDX_QW),
                         states.select(1, IDX_QX),
                         states.select(1, IDX_QY),
                         states.select(1, IDX_QZ),
                         states.select(1, IDX_VX),
                         states.select(1, IDX_VY),
                         states.select(1, IDX_OZ)}, 1);
}

static torch::Tensor compute_deltas(const torch::Tensor &states, const torch::Tensor &next_states)
{
    auto qw = states.select(1, IDX_QW);
    auto qx = states.select(1, IDX_QX);
    auto qy = states.select(1, IDX_QY);
    auto qz = states.select(1, IDX_QZ);
    auto yaw = torch::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    auto next_qw = next_states.select(1, IDX_QW);
    auto next_qx = next_states.select(1, IDX_QX);
    auto next_qy = next_states.select(1, IDX_QY);
    auto next_qz = next_states.select(1, IDX_QZ);
    auto next_yaw = torch::atan2(2.0 * (next_qw * next_qz + next_qx * next_qy),
                                  1.0 - 2.0 * (next_qy * next_qy + next_qz * next_qz));

    auto dx = next_states.select(1, IDX_X) - states.select(1, IDX_X);
    auto dy = next_states.select(1, IDX_Y) - states.select(1, IDX_Y);
    auto dyaw = torch::atan2(torch::sin(next_yaw - yaw), torch::cos(next_yaw - yaw));
    auto dvx = next_states.select(1, IDX_VX) - states.select(1, IDX_VX);
    auto dvy = next_states.select(1, IDX_VY) - states.select(1, IDX_VY);
    auto doz = next_states.select(1, IDX_OZ) - states.select(1, IDX_OZ);

    return torch::stack({dx, dy, dyaw, dvx, dvy, doz}, 1);
}

struct Normaliser
{
    torch::Tensor input_mean, input_std, action_mean, action_std, target_mean, target_std;

    void fit(const torch::Tensor &inputs, const torch::Tensor &actions, const torch::Tensor &targets)
    {
        input_mean = inputs.mean(0);
        input_std = inputs.std(0, false).clamp_min(1e-6);
        action_mean = actions.mean(0);
        action_std = actions.std(0, false).clamp_min(1e-6);
        target_mean = targets.mean(0);
        target_std = targets.std(0, false).clamp_min(1e-6);
    }

    torch::Tensor norm_input(const torch::Tensor &x) const { return (x - input_mean) / input_std; }
    torch::Tensor norm_action(const torch::Tensor &x) const { return (x - action_mean) / action_std; }
    torch::Tensor norm_target(const torch::Tensor &x) const { return (x - target_mean) / target_std; }
    torch::Tensor unnorm_target(const torch::Tensor &x) const { return x * target_std + target_mean; }

    void save(const std::filesystem::path &path) const
    {
        std::ofstream out(path);
        if (!out.is_open()) throw std::runtime_error("Failed to write normaliser file: " + path.string());
        out << std::fixed << std::setprecision(8);
        auto write_tensor = [&out](const char *name, const torch::Tensor &tensor) {
            out << name;
            auto cpu = tensor.to(torch::kCPU).contiguous();
            for (int64_t i = 0; i < cpu.numel(); ++i) out << ' ' << cpu.data_ptr<float>()[i];
            out << '\n';
        };
        write_tensor("input_mean", input_mean);
        write_tensor("input_std", input_std);
        write_tensor("action_mean", action_mean);
        write_tensor("action_std", action_std);
        write_tensor("target_mean", target_mean);
        write_tensor("target_std", target_std);
    }

    static Normaliser load(const std::filesystem::path &path)
    {
        std::ifstream in(path);
        if (!in.is_open()) throw std::runtime_error("Failed to open normaliser file: " + path.string());

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

static torch::Tensor make_output_weights(torch::Device device)
{
    auto weights = torch::ones({kOutputDim}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
    weights[0] = 2.0f;
    weights[1] = 2.0f;
    weights[2] = 2.0f;
    return weights;
}

static torch::Tensor gaussian_nll_loss(const torch::Tensor &mu,
                                       const torch::Tensor &log_var,
                                       const torch::Tensor &target,
                                       const torch::Tensor &weights)
{
    auto clamped = torch::clamp(log_var, -4.0, 4.0);
    auto var = torch::exp(clamped);
    auto nll = 0.5 * (clamped + (target - mu).pow(2) / var);
    return (nll * weights).mean();
}

static torch::Tensor weighted_mse_loss(const torch::Tensor &pred,
                                       const torch::Tensor &target,
                                       const torch::Tensor &weights)
{
    return ((pred - target).pow(2) * weights).mean();
}

static double train_epoch(GaussianMLP &model,
                          const torch::Tensor &inputs,
                          const torch::Tensor &targets,
                          const torch::Tensor &weights,
                          double mean_loss_weight,
                          int64_t batch_size,
                          torch::Device device,
                          torch::optim::AdamW &optimiser)
{
    model->train();
    const int64_t count = inputs.size(0);
    auto permutation = torch::randperm(count, torch::TensorOptions().dtype(torch::kLong));
    double total_loss = 0.0;

    for (int64_t start = 0; start < count; start += batch_size)
    {
        const int64_t length = std::min(batch_size, count - start);
        auto batch_indices = permutation.narrow(0, start, length);
        auto batch_inputs = inputs.index_select(0, batch_indices).to(device);
        auto batch_targets = targets.index_select(0, batch_indices).to(device);

        optimiser.zero_grad();
        auto [mu, log_var] = model->forward(batch_inputs);
        auto nll_loss = gaussian_nll_loss(mu, log_var, batch_targets, weights);
        auto mean_loss = weighted_mse_loss(mu, batch_targets, weights);
        auto loss = nll_loss + (mean_loss_weight * mean_loss);
        loss.backward();
        torch::nn::utils::clip_grad_norm_(model->parameters(), 1.0);
        optimiser.step();

        total_loss += loss.item<double>() * static_cast<double>(length);
    }

    return total_loss / static_cast<double>(count);
}

static double evaluate_loss(GaussianMLP &model,
                            const torch::Tensor &inputs,
                            const torch::Tensor &targets,
                            const torch::Tensor &weights,
                            double mean_loss_weight,
                            int64_t batch_size,
                            torch::Device device)
{
    torch::NoGradGuard no_grad;
    model->eval();
    const int64_t count = inputs.size(0);
    double total_loss = 0.0;
    for (int64_t start = 0; start < count; start += batch_size)
    {
        const int64_t length = std::min(batch_size, count - start);
        auto batch_inputs = inputs.narrow(0, start, length).to(device);
        auto batch_targets = targets.narrow(0, start, length).to(device);
        auto [mu, log_var] = model->forward(batch_inputs);
        auto nll_loss = gaussian_nll_loss(mu, log_var, batch_targets, weights);
        auto mean_loss = weighted_mse_loss(mu, batch_targets, weights);
        total_loss += (nll_loss + (mean_loss_weight * mean_loss)).item<double>() * static_cast<double>(length);
    }
    return total_loss / static_cast<double>(count);
}

int main(int argc, char **argv)
{
    po::options_description desc("Gaussian delta transition model trainer");
    desc.add_options()
        ("help,h", "show help")
        ("csv", po::value<std::string>()->required(), "path to transitions.csv")
        ("output-dir", po::value<std::string>()->default_value("./output_gaussian_delta"), "directory for checkpoints and normaliser")
        ("epochs", po::value<int>()->default_value(1000), "maximum training epochs")
        ("batch-size", po::value<int>()->default_value(256), "mini-batch size")
        ("lr", po::value<double>()->default_value(1e-3), "learning rate")
        ("weight-decay", po::value<double>()->default_value(5e-4), "AdamW weight decay")
        ("patience", po::value<int>()->default_value(100), "early stopping patience")
        ("dropout", po::value<double>()->default_value(0.2), "dropout rate")
        ("mean-loss-weight", po::value<double>()->default_value(1.0), "auxiliary weighted MSE on the Gaussian mean")
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

    const auto csv_path = vm["csv"].as<std::string>();
    const auto output_dir = std::filesystem::path(vm["output-dir"].as<std::string>());
    const int epochs = vm["epochs"].as<int>();
    const int batch_size = vm["batch-size"].as<int>();
    const double lr = vm["lr"].as<double>();
    const double weight_decay = vm["weight-decay"].as<double>();
    const int patience = vm["patience"].as<int>();
    const double dropout = vm["dropout"].as<double>();
    const double mean_loss_weight = vm["mean-loss-weight"].as<double>();
    const int seed = vm["seed"].as<int>();

    if (batch_size <= 0) { std::cerr << "batch-size must be positive\n"; return 1; }
    std::filesystem::create_directories(output_dir);
    torch::manual_seed(seed);
    std::srand(seed);

    const torch::Device device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);
    std::cout << "Using device: " << device << '\n';

    LoadedCsv data = load_csv(csv_path);
    const int64_t row_count = data.states.size(0);
    const int64_t train_count = static_cast<int64_t>(row_count * 0.70);
    const int64_t val_count = static_cast<int64_t>(row_count * 0.20);
    const int64_t test_count = row_count - train_count - val_count;
    if (train_count <= 0 || val_count <= 0 || test_count <= 0) { std::cerr << "Not enough data for a 70/20/10 split\n"; return 1; }

    auto train_states = data.states.narrow(0, 0, train_count).clone();
    auto val_states = data.states.narrow(0, train_count, val_count).clone();
    auto test_states = data.states.narrow(0, train_count + val_count, test_count).clone();
    auto train_actions = data.actions.narrow(0, 0, train_count).clone();
    auto val_actions = data.actions.narrow(0, train_count, val_count).clone();
    auto test_actions = data.actions.narrow(0, train_count + val_count, test_count).clone();
    auto train_targets = compute_deltas(train_states, data.next_states.narrow(0, 0, train_count).clone());
    auto val_targets = compute_deltas(val_states, data.next_states.narrow(0, train_count, val_count).clone());
    auto test_targets = compute_deltas(test_states, data.next_states.narrow(0, train_count + val_count, test_count).clone());

    auto train_inputs = state_to_input_gaussian(train_states);
    auto val_inputs = state_to_input_gaussian(val_states);
    auto test_inputs = state_to_input_gaussian(test_states);

    Normaliser normaliser;
    normaliser.fit(train_inputs, train_actions, train_targets);
    normaliser.save(output_dir / "normaliser.txt");

    auto train_model_inputs = torch::cat({normaliser.norm_input(train_inputs), normaliser.norm_action(train_actions)}, 1);
    auto val_model_inputs = torch::cat({normaliser.norm_input(val_inputs), normaliser.norm_action(val_actions)}, 1);
    auto test_model_inputs = torch::cat({normaliser.norm_input(test_inputs), normaliser.norm_action(test_actions)}, 1);

    auto train_targets_norm = normaliser.norm_target(train_targets);
    auto val_targets_norm = normaliser.norm_target(val_targets);
    auto test_targets_norm = normaliser.norm_target(test_targets);

    GaussianMLP model(kInputDim, kOutputDim, dropout);
    model->to(device);
    auto weights = make_output_weights(device);
    torch::optim::AdamW optimiser(model->parameters(), torch::optim::AdamWOptions(lr).weight_decay(weight_decay));

    const auto best_ckpt_path = output_dir / "transition_gaussian_delta_model_best.pt";
    const auto latest_ckpt_path = output_dir / "transition_gaussian_delta_model_latest.pt";

    std::cout << "Loaded " << row_count << " transitions from " << csv_path << '\n';
    std::cout << "Sequential split: train=" << train_count << " val=" << val_count << " test=" << test_count << '\n';
    std::cout << "Model: Gaussian delta MLP  (" << kInputDim << "-D input -> mu/logvar of " << kOutputDim << "-D delta output)\n";

    double best_val = std::numeric_limits<double>::infinity();
    int no_improve = 0;
    for (int epoch = 1; epoch <= epochs; ++epoch)
    {
        const double train_loss = train_epoch(model, train_model_inputs, train_targets_norm, weights, mean_loss_weight, batch_size, device, optimiser);
        const double val_loss = evaluate_loss(model, val_model_inputs, val_targets_norm, weights, mean_loss_weight, batch_size, device);
        std::cout << "Epoch " << std::setw(4) << epoch << '/' << epochs
                  << "  train=" << std::fixed << std::setprecision(6) << train_loss
                  << "  val=" << val_loss << '\n';

        torch::save(model, latest_ckpt_path.string());
        if (val_loss < best_val)
        {
            best_val = val_loss;
            no_improve = 0;
            torch::save(model, best_ckpt_path.string());
        }
        else if (++no_improve >= patience)
        {
            std::cout << "Early stopping at epoch " << epoch << " (best val=" << best_val << ")\n";
            break;
        }
    }

    torch::load(model, best_ckpt_path.string());
    const double test_loss = evaluate_loss(model, test_model_inputs, test_targets_norm, weights, mean_loss_weight, batch_size, device);
    std::cout << "Best val loss: " << best_val << " -> " << best_ckpt_path.string() << '\n';
    std::cout << "Latest checkpoint -> " << latest_ckpt_path.string() << '\n';
    std::cout << "Test loss: " << test_loss << '\n';
    return 0;
}
