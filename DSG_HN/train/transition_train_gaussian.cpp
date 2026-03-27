#include <torch/torch.h>

#include <boost/program_options.hpp>

#include <algorithm>
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
            make_tensor_from_flat(timestamps_flat, row_count, 1)};
}

struct Normaliser
{
    torch::Tensor state_mean, state_std, action_mean, action_std, target_mean, target_std;

    void fit(const torch::Tensor &states, const torch::Tensor &actions, const torch::Tensor &targets)
    {
        state_mean = states.mean(0);
        state_std = states.std(0, false).clamp_min(1e-6);
        action_mean = actions.mean(0);
        action_std = actions.std(0, false).clamp_min(1e-6);
        target_mean = targets.mean(0);
        target_std = targets.std(0, false).clamp_min(1e-6);
    }

    torch::Tensor norm_state(const torch::Tensor &x) const { return (x - state_mean) / state_std; }
    torch::Tensor norm_action(const torch::Tensor &x) const { return (x - action_mean) / action_std; }
    torch::Tensor norm_target(const torch::Tensor &x) const { return (x - target_mean) / target_std; }

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
        write_tensor("state_mean", state_mean);
        write_tensor("state_std", state_std);
        write_tensor("action_mean", action_mean);
        write_tensor("action_std", action_std);
        write_tensor("target_mean", target_mean);
        write_tensor("target_std", target_std);
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

struct GaussianTransitionMLPImpl : torch::nn::Module
{
    GaussianTransitionMLPImpl(int64_t input_dim, int64_t output_dim, double dropout)
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

TORCH_MODULE(GaussianTransitionMLP);

static torch::Tensor make_output_weights(torch::Device device)
{
    auto weights = torch::ones({kStateDim}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
    weights[0] = 12.0f;
    weights[1] = 12.0f;
    weights[7] = 5.0f;
    weights[8] = 5.0f;
    weights[12] = 1.5f;
    for (int i = kBaseStateDim; i < kStateDim; ++i) weights[i] = 0.10f;
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

static double train_epoch(GaussianTransitionMLP &model,
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

static double evaluate_loss(GaussianTransitionMLP &model,
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
    po::options_description desc("Gaussian transition model trainer");
    desc.add_options()
        ("help,h", "show help")
        ("csv", po::value<std::string>()->required(), "path to transitions.csv")
        ("output-dir", po::value<std::string>()->default_value("./output_gaussian"), "directory for checkpoints and normaliser")
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
    auto train_targets = data.next_states.narrow(0, 0, train_count).clone();
    auto val_targets = data.next_states.narrow(0, train_count, val_count).clone();
    auto test_targets = data.next_states.narrow(0, train_count + val_count, test_count).clone();

    Normaliser normaliser;
    normaliser.fit(train_states, train_actions, train_targets);
    normaliser.save(output_dir / "normaliser.txt");

    auto train_inputs = torch::cat({normaliser.norm_state(train_states), normaliser.norm_action(train_actions)}, 1);
    auto val_inputs = torch::cat({normaliser.norm_state(val_states), normaliser.norm_action(val_actions)}, 1);
    auto test_inputs = torch::cat({normaliser.norm_state(test_states), normaliser.norm_action(test_actions)}, 1);

    auto train_targets_norm = normaliser.norm_target(train_targets);
    auto val_targets_norm = normaliser.norm_target(val_targets);
    auto test_targets_norm = normaliser.norm_target(test_targets);

    GaussianTransitionMLP model(kInputDim, kStateDim, dropout);
    model->to(device);
    auto weights = make_output_weights(device);
    torch::optim::AdamW optimiser(model->parameters(), torch::optim::AdamWOptions(lr).weight_decay(weight_decay));

    const auto best_ckpt_path = output_dir / "transition_gaussian_model_best.pt";
    const auto latest_ckpt_path = output_dir / "transition_gaussian_model_latest.pt";

    std::cout << "Loaded " << row_count << " transitions from " << csv_path << '\n';
    std::cout << "Sequential split: train=" << train_count << " val=" << val_count << " test=" << test_count << '\n';
    std::cout << "Model: GaussianTransitionMLP  (" << kInputDim << "-D input -> mean/logvar of " << kStateDim << "-D output)\n";

    double best_val = std::numeric_limits<double>::infinity();
    int no_improve = 0;
    for (int epoch = 1; epoch <= epochs; ++epoch)
    {
        const double train_loss = train_epoch(model, train_inputs, train_targets_norm, weights, mean_loss_weight, batch_size, device, optimiser);
        const double val_loss = evaluate_loss(model, val_inputs, val_targets_norm, weights, mean_loss_weight, batch_size, device);
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
    const double test_loss = evaluate_loss(model, test_inputs, test_targets_norm, weights, mean_loss_weight, batch_size, device);
    std::cout << "Best val loss: " << best_val << " -> " << best_ckpt_path.string() << '\n';
    std::cout << "Latest checkpoint -> " << latest_ckpt_path.string() << '\n';
    std::cout << "Test loss: " << test_loss << '\n';
    return 0;
}