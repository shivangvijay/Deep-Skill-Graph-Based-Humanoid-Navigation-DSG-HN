#define _USE_MATH_DEFINES
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

// =====================================================================
//  Dimension constants (same as gaussian_delta trainer)
// =====================================================================
static constexpr int kRawStateDim   = 13;
static constexpr int kJointDim      = 35;
static constexpr int kFullStateDim  = kRawStateDim + (2 * kJointDim);  // 83
static constexpr int kInputStateDim = 7 + (2 * kJointDim);             // 77
static constexpr int kActionDim     = 3;
static constexpr int kTokenDim      = kInputStateDim + kActionDim;     // 80
static constexpr int kOutputDim     = 6 + (2 * kJointDim);            // 76

static constexpr int IDX_X  = 0;
static constexpr int IDX_Y  = 1;
static constexpr int IDX_Z  = 2;
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

// =====================================================================
//  CSV helpers (reused from gaussian_delta)
// =====================================================================
static std::string two_digit(int value)
{
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << value;
    return oss.str();
}

static std::vector<std::string> current_state_columns()
{
    std::vector<std::string> columns = {"x","y","z","qw","qx","qy","qz","vx","vy","vz","omega_x","omega_y","omega_z"};
    for (int i = 0; i < kJointDim; ++i) columns.push_back("joint_pos_" + two_digit(i));
    for (int i = 0; i < kJointDim; ++i) columns.push_back("joint_vel_" + two_digit(i));
    return columns;
}

static std::vector<std::string> next_state_columns()
{
    std::vector<std::string> columns = {"next_x","next_y","next_z","next_qw","next_qx","next_qy","next_qz","next_vx","next_vy","next_vz","next_omega_x","next_omega_y","next_omega_z"};
    for (int i = 0; i < kJointDim; ++i) columns.push_back("next_joint_pos_" + two_digit(i));
    for (int i = 0; i < kJointDim; ++i) columns.push_back("next_joint_vel_" + two_digit(i));
    return columns;
}

static std::vector<std::string> action_columns() { return {"cmd_vx","cmd_vy","cmd_yaw"}; }

static std::vector<std::string> split_csv_line(const std::string &line)
{
    std::vector<std::string> cells;
    std::string cell;
    std::stringstream ss(line);
    while (std::getline(ss, cell, ',')) cells.push_back(cell);
    return cells;
}

static std::vector<int> lookup_indices(const std::unordered_map<std::string,int> &hdr,
                                       const std::vector<std::string> &cols)
{
    std::vector<int> idx;
    idx.reserve(cols.size());
    for (auto &c : cols)
    {
        auto it = hdr.find(c);
        if (it == hdr.end()) throw std::runtime_error("Missing CSV column: " + c);
        idx.push_back(it->second);
    }
    return idx;
}

static torch::Tensor make_tensor(const std::vector<float> &d, int64_t r, int64_t c)
{
    auto t = torch::empty({r, c}, torch::kFloat32);
    std::memcpy(t.data_ptr<float>(), d.data(), d.size() * sizeof(float));
    return t;
}

static torch::Tensor make_double_tensor(const std::vector<double> &d, int64_t r, int64_t c)
{
    auto t = torch::empty({r, c}, torch::kFloat64);
    std::memcpy(t.data_ptr<double>(), d.data(), d.size() * sizeof(double));
    return t;
}

struct LoadedCsv { torch::Tensor states, actions, next_states, timestamps; };

static LoadedCsv load_csv(const std::string &csv_path)
{
    std::ifstream file(csv_path);
    if (!file.is_open()) throw std::runtime_error("Failed to open CSV: " + csv_path);

    std::string header_line;
    if (!std::getline(file, header_line)) throw std::runtime_error("CSV empty: " + csv_path);

    auto headers = split_csv_line(header_line);
    std::unordered_map<std::string,int> hdr;
    for (int i = 0; i < (int)headers.size(); ++i) hdr.emplace(headers[i], i);

    auto si  = lookup_indices(hdr, current_state_columns());
    auto ai  = lookup_indices(hdr, action_columns());
    auto nsi = lookup_indices(hdr, next_state_columns());

    auto ts_it = hdr.find("timestamp_s");
    if (ts_it == hdr.end()) throw std::runtime_error("Missing CSV column: timestamp_s");
    const int ts_idx = ts_it->second;

    std::vector<float> sf, af, nsf;
    std::vector<double> tf;
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty()) continue;
        auto cells = split_csv_line(line);
        if (cells.size() != headers.size()) continue;
        try {
            tf.push_back(std::stod(cells[ts_idx]));
            for (int i : si)  sf.push_back(std::stof(cells[i]));
            for (int i : ai)  af.push_back(std::stof(cells[i]));
            for (int i : nsi) nsf.push_back(std::stof(cells[i]));
        } catch (...) { continue; }
    }

    int64_t n = (int64_t)tf.size();
    if (n == 0) throw std::runtime_error("No valid rows from: " + csv_path);

    return {make_tensor(sf,n,kFullStateDim), make_tensor(af,n,kActionDim),
            make_tensor(nsf,n,kFullStateDim), make_double_tensor(tf,n,1)};
}

// =====================================================================
//  Feature extraction & delta computation (same as gaussian_delta)
// =====================================================================
static torch::Tensor state_to_input(const torch::Tensor &states)
{
    std::vector<torch::Tensor> f = {
        states.select(1,IDX_QW), states.select(1,IDX_QX),
        states.select(1,IDX_QY), states.select(1,IDX_QZ),
        states.select(1,IDX_VX), states.select(1,IDX_VY),
        states.select(1,IDX_OZ)
    };
    for (int i = 0; i < kJointDim; ++i) f.push_back(states.select(1, kRawStateDim + i));
    for (int i = 0; i < kJointDim; ++i) f.push_back(states.select(1, kRawStateDim + kJointDim + i));
    return torch::stack(f, 1);
}

static torch::Tensor compute_deltas(const torch::Tensor &s, const torch::Tensor &ns)
{
    auto yaw = torch::atan2(2.0*(s.select(1,IDX_QW)*s.select(1,IDX_QZ) + s.select(1,IDX_QX)*s.select(1,IDX_QY)),
                            1.0-2.0*(s.select(1,IDX_QY).pow(2) + s.select(1,IDX_QZ).pow(2)));
    auto nyaw = torch::atan2(2.0*(ns.select(1,IDX_QW)*ns.select(1,IDX_QZ) + ns.select(1,IDX_QX)*ns.select(1,IDX_QY)),
                             1.0-2.0*(ns.select(1,IDX_QY).pow(2) + ns.select(1,IDX_QZ).pow(2)));

    std::vector<torch::Tensor> d = {
        ns.select(1,IDX_X)  - s.select(1,IDX_X),
        ns.select(1,IDX_Y)  - s.select(1,IDX_Y),
        torch::atan2(torch::sin(nyaw-yaw), torch::cos(nyaw-yaw)),
        ns.select(1,IDX_VX) - s.select(1,IDX_VX),
        ns.select(1,IDX_VY) - s.select(1,IDX_VY),
        ns.select(1,IDX_OZ) - s.select(1,IDX_OZ)
    };
    for (int i = 0; i < kJointDim; ++i)
        d.push_back(ns.select(1, kRawStateDim+i) - s.select(1, kRawStateDim+i));
    for (int i = 0; i < kJointDim; ++i)
        d.push_back(ns.select(1, kRawStateDim+kJointDim+i) - s.select(1, kRawStateDim+kJointDim+i));
    return torch::stack(d, 1);
}

// =====================================================================
//  Normaliser (same as gaussian_delta)
// =====================================================================
struct Normaliser
{
    torch::Tensor input_mean, input_std, action_mean, action_std, target_mean, target_std;

    void fit(const torch::Tensor &inp, const torch::Tensor &act, const torch::Tensor &tgt)
    {
        input_mean  = inp.mean(0); input_std  = inp.std(0,false).clamp_min(1e-6);
        action_mean = act.mean(0); action_std = act.std(0,false).clamp_min(1e-6);
        target_mean = tgt.mean(0); target_std = tgt.std(0,false).clamp_min(1e-6);
    }

    torch::Tensor norm_input (const torch::Tensor &x) const { return (x - input_mean)  / input_std;  }
    torch::Tensor norm_action(const torch::Tensor &x) const { return (x - action_mean) / action_std; }
    torch::Tensor norm_target(const torch::Tensor &x) const { return (x - target_mean) / target_std; }

    void save(const std::filesystem::path &p) const
    {
        std::ofstream out(p);
        if (!out.is_open()) throw std::runtime_error("Cannot write: " + p.string());
        out << std::fixed << std::setprecision(8);
        auto w = [&](const char *n, const torch::Tensor &t) {
            out << n;
            auto c = t.to(torch::kCPU).contiguous();
            for (int64_t i = 0; i < c.numel(); ++i) out << ' ' << c.data_ptr<float>()[i];
            out << '\n';
        };
        w("input_mean",  input_mean);  w("input_std",  input_std);
        w("action_mean", action_mean); w("action_std", action_std);
        w("target_mean", target_mean); w("target_std", target_std);
    }
};

// =====================================================================
//  Data windowing — group consecutive rows into sequences of length N
// =====================================================================
struct WindowedData
{
    torch::Tensor sequences;     // [num_windows, N, kTokenDim]
    torch::Tensor targets;       // [num_windows, kOutputDim]
    torch::Tensor future_targets;// [num_windows, K, kOutputDim]  (multi-step GT)
    torch::Tensor future_actions;// [num_windows, K, kActionDim]  (actions for multi-step rollout)
    int future_steps;            // K
};

static WindowedData create_windows(const torch::Tensor &inputs,    // [rows, kInputStateDim]
                                   const torch::Tensor &actions,   // [rows, kActionDim]
                                   const torch::Tensor &targets,   // [rows, kOutputDim]
                                   const torch::Tensor &timestamps,// [rows, 1]
                                   int window_size,
                                   int future_steps = 3,
                                   double max_time_gap = 0.5)
{
    const int64_t rows = inputs.size(0);
    auto ts_acc = timestamps.accessor<double, 2>();

    auto tokens = torch::cat({inputs, actions}, 1);

    std::vector<torch::Tensor> win_list;
    std::vector<torch::Tensor> tgt_list;
    std::vector<torch::Tensor> ftgt_list;
    std::vector<torch::Tensor> fact_list;

    for (int64_t end = window_size - 1; end < rows; ++end)
    {
        int64_t start = end - window_size + 1;

        bool valid = true;
        for (int64_t i = start + 1; i <= end; ++i)
        {
            double dt = ts_acc[i][0] - ts_acc[i-1][0];
            if (dt < 0 || dt > max_time_gap) { valid = false; break; }
        }
        if (!valid) continue;

        win_list.push_back(tokens.slice(0, start, end + 1));
        tgt_list.push_back(targets[end]);

        // Collect future targets/actions for multi-step loss (as many as available)
        int64_t avail = 0;
        for (int64_t k = 1; k <= future_steps && (end + k) < rows; ++k)
        {
            double dt = ts_acc[end + k][0] - ts_acc[end + k - 1][0];
            if (dt < 0 || dt > max_time_gap) break;
            avail = k;
        }

        auto ft = torch::zeros({future_steps, kOutputDim}, torch::kFloat32);
        auto fa = torch::zeros({future_steps, kActionDim}, torch::kFloat32);
        for (int64_t k = 0; k < avail; ++k)
        {
            ft[k] = targets[end + 1 + k];
            fa[k] = actions.narrow(0, end + 1 + k, 1).squeeze(0);
        }
        ftgt_list.push_back(ft);
        fact_list.push_back(fa);
    }

    if (win_list.empty()) throw std::runtime_error("No valid windows (need more data or smaller window)");

    return {torch::stack(win_list), torch::stack(tgt_list),
            torch::stack(ftgt_list), torch::stack(fact_list), future_steps};
}

// =====================================================================
//  Causal Transformer model
// =====================================================================
static void init_weights(torch::nn::Module &m)
{
    if (auto *lin = m.as<torch::nn::Linear>())
    {
        torch::NoGradGuard g;
        torch::nn::init::xavier_uniform_(lin->weight);
        if (lin->bias.defined()) torch::nn::init::constant_(lin->bias, 0.0);
    }
}

struct TransformerBlockImpl : torch::nn::Module
{
    TransformerBlockImpl(int64_t d_model, int64_t n_heads, double dropout)
    {
        attn = register_module("attn",
            torch::nn::MultiheadAttention(
                torch::nn::MultiheadAttentionOptions(d_model, n_heads)
                    .dropout(dropout)));
        ln1  = register_module("ln1", torch::nn::LayerNorm(torch::nn::LayerNormOptions({d_model})));
        ff   = register_module("ff", torch::nn::Sequential(
            torch::nn::Linear(d_model, 4 * d_model),
            torch::nn::SiLU(),
            torch::nn::Dropout(torch::nn::DropoutOptions(dropout)),
            torch::nn::Linear(4 * d_model, d_model),
            torch::nn::Dropout(torch::nn::DropoutOptions(dropout))
        ));
        ln2  = register_module("ln2", torch::nn::LayerNorm(torch::nn::LayerNormOptions({d_model})));
    }

    // x: [seq, batch, d_model],  mask: [seq, seq]
    torch::Tensor forward(torch::Tensor x, const torch::Tensor &mask)
    {
        auto [attn_out, _] = attn->forward(x, x, x, /*key_padding_mask=*/{}, /*need_weights=*/false, /*attn_mask=*/mask);
        x = ln1->forward(x + attn_out);
        x = ln2->forward(x + ff->forward(x));
        return x;
    }

    torch::nn::MultiheadAttention attn{nullptr};
    torch::nn::LayerNorm ln1{nullptr}, ln2{nullptr};
    torch::nn::Sequential ff{nullptr};
};
TORCH_MODULE(TransformerBlock);

struct TransformerTransitionModelImpl : torch::nn::Module
{
    TransformerTransitionModelImpl(int64_t token_dim, int64_t output_dim,
                                   int64_t seq_len, int64_t d_model,
                                   int64_t n_heads, int64_t n_layers, double dropout)
        : seq_len_(seq_len), d_model_(d_model)
    {
        token_proj = register_module("token_proj", torch::nn::Linear(token_dim, d_model));
        pos_embed  = register_module("pos_embed",
            torch::nn::Embedding(torch::nn::EmbeddingOptions(seq_len, d_model)));
        drop = register_module("drop", torch::nn::Dropout(torch::nn::DropoutOptions(dropout)));

        for (int64_t i = 0; i < n_layers; ++i)
            blocks.push_back(register_module("block_" + std::to_string(i),
                TransformerBlock(d_model, n_heads, dropout)));

        ln_final = register_module("ln_final", torch::nn::LayerNorm(torch::nn::LayerNormOptions({d_model})));
        mu_head  = register_module("mu_head", torch::nn::Linear(d_model, output_dim));
        lv_head  = register_module("lv_head", torch::nn::Linear(d_model, output_dim));

        apply(init_weights);

        // Pre-build causal mask: upper-triangular = -inf
        causal_mask_ = torch::full({seq_len, seq_len}, -std::numeric_limits<float>::infinity());
        causal_mask_.tril_();                     // lower triangle = 0
        causal_mask_ = causal_mask_.masked_fill(causal_mask_ == 0, 0.0f);
        // tril_ sets lower triangle to original values (which are -inf), so redo:
        causal_mask_ = torch::zeros({seq_len, seq_len});
        causal_mask_.masked_fill_(torch::ones({seq_len, seq_len}).triu(1).to(torch::kBool),
                                  -std::numeric_limits<float>::infinity());
    }

    std::pair<torch::Tensor, torch::Tensor> forward(torch::Tensor x)
    {
        // x: [batch, seq_len, token_dim]
        auto B = x.size(0);
        auto S = x.size(1);

        x = token_proj->forward(x);  // [batch, seq, d_model]

        auto positions = torch::arange(S, torch::TensorOptions().dtype(torch::kLong).device(x.device()));
        x = x + pos_embed->forward(positions).unsqueeze(0);
        x = drop->forward(x);

        // MHA expects [seq, batch, d_model]
        x = x.transpose(0, 1);
        auto mask = causal_mask_.to(x.device());

        for (auto &block : blocks)
            x = block->forward(x, mask);

        x = x.transpose(0, 1);       // back to [batch, seq, d_model]
        x = ln_final->forward(x);

        auto last = x.select(1, S - 1); // [batch, d_model]
        return {mu_head->forward(last), lv_head->forward(last)};
    }

    int64_t seq_len_, d_model_;
    torch::nn::Linear token_proj{nullptr}, mu_head{nullptr}, lv_head{nullptr};
    torch::nn::Embedding pos_embed{nullptr};
    torch::nn::Dropout drop{nullptr};
    torch::nn::LayerNorm ln_final{nullptr};
    std::vector<TransformerBlock> blocks;
    torch::Tensor causal_mask_;
};
TORCH_MODULE(TransformerTransitionModel);

// =====================================================================
//  Loss functions & output weights (same as gaussian_delta)
// =====================================================================
static torch::Tensor make_output_weights(torch::Device device)
{
    auto w = torch::ones({kOutputDim}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
    w[0] = 5.0f; w[1] = 5.0f;           // dx, dy — position accuracy is critical for MPC
    w[2] = 3.0f;                         // dyaw — heading matters for navigation
    w[3] = 1.5f; w[4] = 1.5f;           // dvx, dvy — velocity stability
    w[5] = 1.0f;                         // doz
    for (int i = 6; i < 6 + kJointDim; ++i) w[i] = 0.3f;
    for (int i = 6 + kJointDim; i < kOutputDim; ++i) w[i] = 0.1f;
    return w;
}

// Cosine LR with linear warmup
static double cosine_lr(int epoch, int total_epochs, double base_lr,
                        double warmup_frac = 0.05, double min_lr_frac = 0.01)
{
    int warmup_epochs = std::max(1, (int)(total_epochs * warmup_frac));
    if (epoch <= warmup_epochs)
        return base_lr * (double)epoch / (double)warmup_epochs;
    double progress = (double)(epoch - warmup_epochs) / (double)(total_epochs - warmup_epochs);
    return base_lr * (min_lr_frac + (1.0 - min_lr_frac) * 0.5 * (1.0 + std::cos(M_PI * progress)));
}

static torch::Tensor gaussian_nll_loss(const torch::Tensor &mu, const torch::Tensor &lv,
                                       const torch::Tensor &tgt, const torch::Tensor &w)
{
    auto clamped = torch::clamp(lv, -4.0, 4.0);
    auto nll = 0.5 * (clamped + (tgt - mu).pow(2) / torch::exp(clamped));
    return (nll * w).mean();
}

static torch::Tensor weighted_mse(const torch::Tensor &pred, const torch::Tensor &tgt,
                                  const torch::Tensor &w)
{
    return ((pred - tgt).pow(2) * w).mean();
}

// =====================================================================
//  Training & evaluation
// =====================================================================

// Build a normalised token from a (normalised) state-input vector and a
// (normalised) action vector.  Both inputs are [batch, dim].
static torch::Tensor build_token_batch(const torch::Tensor &norm_inp,
                                       const torch::Tensor &norm_act)
{
    return torch::cat({norm_inp, norm_act}, /*dim=*/1); // [batch, kTokenDim]
}

static double train_epoch(TransformerTransitionModel &model,
                          const WindowedData &data,
                          const torch::Tensor &weights,
                          double mse_weight, double multistep_weight,
                          int64_t batch_size,
                          torch::Device device,
                          torch::optim::AdamW &opt)
{
    model->train();
    const int64_t count = data.sequences.size(0);
    auto perm = torch::randperm(count, torch::kLong);
    double total = 0.0;

    for (int64_t s = 0; s < count; s += batch_size)
    {
        int64_t len = std::min(batch_size, count - s);
        auto idx = perm.narrow(0, s, len);
        auto bx = data.sequences.index_select(0, idx).to(device);  // [B, seq, tok]
        auto bt = data.targets.index_select(0, idx).to(device);    // [B, out]

        opt.zero_grad();
        auto [mu, lv] = model->forward(bx);
        auto loss = gaussian_nll_loss(mu, lv, bt, weights)
                  + mse_weight * weighted_mse(mu, bt, weights);

        // ── Multi-step rollout loss ─────────────────────────────────
        if (multistep_weight > 0.0 && data.future_steps > 0)
        {
            auto bft = data.future_targets.index_select(0, idx).to(device); // [B, K, out]
            auto bfa = data.future_actions.index_select(0, idx).to(device); // [B, K, act]

            auto seq = bx.clone();                  // [B, seq, tok]
            auto pred_delta_norm = mu;              // normalised prediction from step 0

            for (int k = 0; k < data.future_steps; ++k)
            {
                // Construct the predicted state-input for the next step by
                // adding the (normalised) predicted delta to the last token's
                // state portion.  This is approximate but avoids needing
                // the full unnormalise -> re-normalise pipeline at train time.
                auto last_inp = seq.select(1, seq.size(1) - 1)
                                    .narrow(1, 0, kInputStateDim); // [B, inp]
                auto new_inp = last_inp + pred_delta_norm.narrow(1, 0, kInputStateDim);

                auto act_k = bfa.select(1, k);  // [B, act] — already normalised
                auto new_tok = build_token_batch(new_inp, act_k).unsqueeze(1);

                // Shift window: drop first token, append new one
                seq = torch::cat({seq.narrow(1, 1, seq.size(1) - 1), new_tok}, 1);

                auto [mu_k, lv_k] = model->forward(seq);
                auto tgt_k = bft.select(1, k);   // [B, out]
                loss = loss + multistep_weight * (
                    gaussian_nll_loss(mu_k, lv_k, tgt_k, weights)
                  + mse_weight * weighted_mse(mu_k, tgt_k, weights));
                pred_delta_norm = mu_k;
            }
        }

        loss.backward();
        torch::nn::utils::clip_grad_norm_(model->parameters(), 1.0);
        opt.step();

        total += loss.item<double>() * (double)len;
    }
    return total / (double)count;
}

static double eval_loss(TransformerTransitionModel &model,
                        const WindowedData &data,
                        const torch::Tensor &weights,
                        double mse_weight, int64_t batch_size, torch::Device device)
{
    torch::NoGradGuard ng;
    model->eval();
    const int64_t count = data.sequences.size(0);
    double total = 0.0;
    for (int64_t s = 0; s < count; s += batch_size)
    {
        int64_t len = std::min(batch_size, count - s);
        auto bx = data.sequences.narrow(0, s, len).to(device);
        auto bt = data.targets.narrow(0, s, len).to(device);
        auto [mu, lv] = model->forward(bx);
        total += (gaussian_nll_loss(mu,lv,bt,weights) + mse_weight*weighted_mse(mu,bt,weights)).item<double>() * (double)len;
    }
    return total / (double)count;
}

// =====================================================================
//  Main
// =====================================================================
int main(int argc, char **argv)
{
    po::options_description desc("Transformer delta transition model trainer (improved)");
    desc.add_options()
        ("help,h",         "show help")
        ("csv",            po::value<std::string>()->required(), "path to transitions.csv")
        ("output-dir",     po::value<std::string>()->default_value("./output_transformer_delta"), "checkpoint directory")
        ("history",        po::value<int>()->default_value(10),      "context window N (timesteps)")
        ("d-model",        po::value<int>()->default_value(128),     "transformer hidden dimension")
        ("n-heads",        po::value<int>()->default_value(4),       "number of attention heads")
        ("n-layers",       po::value<int>()->default_value(4),       "number of transformer blocks")
        ("epochs",         po::value<int>()->default_value(1000),    "max training epochs")
        ("batch-size",     po::value<int>()->default_value(256),     "mini-batch size")
        ("lr",             po::value<double>()->default_value(1e-4), "peak learning rate")
        ("weight-decay",   po::value<double>()->default_value(5e-4), "AdamW weight decay")
        ("patience",       po::value<int>()->default_value(100),     "early stopping patience")
        ("dropout",        po::value<double>()->default_value(0.1),  "dropout rate")
        ("mean-loss-weight", po::value<double>()->default_value(1.0),"MSE weight alongside NLL")
        ("max-time-gap",   po::value<double>()->default_value(0.5),  "max seconds between rows before episode break")
        ("future-steps",   po::value<int>()->default_value(3),       "multi-step rollout loss horizon K")
        ("multistep-weight", po::value<double>()->default_value(0.3),"weight for multi-step loss (annealed 0 -> this)")
        ("seed",           po::value<int>()->default_value(42),      "random seed");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) { std::cout << desc; return 0; }
        po::notify(vm);
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n' << desc; return 1;
    }

    const auto csv_path      = vm["csv"].as<std::string>();
    const auto output_dir    = std::filesystem::path(vm["output-dir"].as<std::string>());
    const int  history       = vm["history"].as<int>();
    const int  d_model       = vm["d-model"].as<int>();
    const int  n_heads       = vm["n-heads"].as<int>();
    const int  n_layers      = vm["n-layers"].as<int>();
    const int  epochs        = vm["epochs"].as<int>();
    const int  batch_size    = vm["batch-size"].as<int>();
    const double lr          = vm["lr"].as<double>();
    const double wd          = vm["weight-decay"].as<double>();
    const int  patience      = vm["patience"].as<int>();
    const double dropout     = vm["dropout"].as<double>();
    const double mse_weight  = vm["mean-loss-weight"].as<double>();
    const double max_gap     = vm["max-time-gap"].as<double>();
    const int  future_steps  = vm["future-steps"].as<int>();
    const double ms_weight_max = vm["multistep-weight"].as<double>();
    const int  seed          = vm["seed"].as<int>();

    std::filesystem::create_directories(output_dir);
    torch::manual_seed(seed);
    std::srand(seed);

    const torch::Device device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);
    std::cout << "Device: " << device << '\n';

    // ── Load CSV ────────────────────────────────────────────────────
    LoadedCsv data = load_csv(csv_path);
    const int64_t total_rows = data.states.size(0);
    std::cout << "Loaded " << total_rows << " transitions from " << csv_path << '\n';

    // ── Compute per-row features, actions, deltas ────────────────
    auto all_inputs  = state_to_input(data.states);
    auto all_actions = data.actions;
    auto all_targets = compute_deltas(data.states, data.next_states);

    // ── 70/20/10 sequential split (before windowing) ──────────────
    const int64_t train_end = static_cast<int64_t>(total_rows * 0.70);
    const int64_t val_end   = static_cast<int64_t>(total_rows * 0.90);

    auto train_inp = all_inputs.narrow(0, 0, train_end).clone();
    auto train_act = all_actions.narrow(0, 0, train_end).clone();
    auto train_tgt = all_targets.narrow(0, 0, train_end).clone();
    auto train_ts  = data.timestamps.narrow(0, 0, train_end).clone();

    auto val_inp = all_inputs.narrow(0, train_end, val_end - train_end).clone();
    auto val_act = all_actions.narrow(0, train_end, val_end - train_end).clone();
    auto val_tgt = all_targets.narrow(0, train_end, val_end - train_end).clone();
    auto val_ts  = data.timestamps.narrow(0, train_end, val_end - train_end).clone();

    auto test_inp = all_inputs.narrow(0, val_end, total_rows - val_end).clone();
    auto test_act = all_actions.narrow(0, val_end, total_rows - val_end).clone();
    auto test_tgt = all_targets.narrow(0, val_end, total_rows - val_end).clone();
    auto test_ts  = data.timestamps.narrow(0, val_end, total_rows - val_end).clone();

    // ── Normaliser (fit on train split) ──────────────────────────
    Normaliser norm;
    norm.fit(train_inp, train_act, train_tgt);
    norm.save(output_dir / "normaliser.txt");

    auto norm_inp  = [&](const torch::Tensor &i) { return norm.norm_input(i);  };
    auto norm_act  = [&](const torch::Tensor &a) { return norm.norm_action(a); };
    auto norm_tgt  = [&](const torch::Tensor &t) { return norm.norm_target(t); };

    // ── Window into sequences ────────────────────────────────────
    std::cout << "Creating windows (history=" << history << ")...\n";

    auto train_win = create_windows(norm_inp(train_inp), norm_act(train_act), norm_tgt(train_tgt), train_ts, history, future_steps, max_gap);
    auto val_win   = create_windows(norm_inp(val_inp),   norm_act(val_act),   norm_tgt(val_tgt),   val_ts,   history, 0, max_gap);
    auto test_win  = create_windows(norm_inp(test_inp),  norm_act(test_act),  norm_tgt(test_tgt),  test_ts,  history, 0, max_gap);

    std::cout << "Windows: train=" << train_win.sequences.size(0)
              << " val=" << val_win.sequences.size(0)
              << " test=" << test_win.sequences.size(0) << '\n';

    // ── Build model ──────────────────────────────────────────────
    TransformerTransitionModel model(kTokenDim, kOutputDim, history, d_model, n_heads, n_layers, dropout);
    model->to(device);

    int64_t param_count = 0;
    for (auto &p : model->parameters()) param_count += p.numel();
    std::cout << "Model: Transformer delta  (token=" << kTokenDim << " d_model=" << d_model
              << " heads=" << n_heads << " layers=" << n_layers
              << " seq=" << history << " params=" << param_count << ")\n";

    auto weights = make_output_weights(device);
    torch::optim::AdamW opt(model->parameters(), torch::optim::AdamWOptions(lr).weight_decay(wd));

    const auto best_path   = output_dir / "transition_transformer_delta_best.pt";
    const auto latest_path = output_dir / "transition_transformer_delta_latest.pt";

    // ── Training loop ────────────────────────────────────────────
    double best_val = std::numeric_limits<double>::infinity();
    int no_improve = 0;

    for (int ep = 1; ep <= epochs; ++ep)
    {
        // Cosine LR schedule with warmup
        double cur_lr = cosine_lr(ep, epochs, lr);
        for (auto &pg : opt.param_groups())
            static_cast<torch::optim::AdamWOptions &>(pg.options()).lr(cur_lr);

        // Anneal multi-step loss weight: ramp from 0 to ms_weight_max over first 30% of epochs
        double ms_frac = std::min(1.0, (double)ep / (0.3 * epochs));
        double ms_weight = ms_weight_max * ms_frac;

        double tl = train_epoch(model, train_win, weights, mse_weight, ms_weight,
                                batch_size, device, opt);
        double vl = eval_loss(model, val_win, weights, mse_weight, batch_size, device);

        if (ep % 10 == 1 || ep == epochs)
            std::cout << "Epoch " << std::setw(4) << ep << '/' << epochs
                      << "  train=" << std::fixed << std::setprecision(6) << tl
                      << "  val=" << vl
                      << "  lr=" << std::scientific << std::setprecision(2) << cur_lr
                      << "  ms_w=" << std::fixed << std::setprecision(3) << ms_weight << '\n';

        torch::save(model, latest_path.string());
        if (vl < best_val)
        {
            best_val = vl;
            no_improve = 0;
            torch::save(model, best_path.string());
        }
        else if (++no_improve >= patience)
        {
            std::cout << "Early stopping at epoch " << ep << " (best val=" << best_val << ")\n";
            break;
        }
    }

    // ── Final test evaluation ────────────────────────────────────
    torch::load(model, best_path.string());
    double test_l = eval_loss(model, test_win, weights, mse_weight, batch_size, device);
    std::cout << "Best val loss: " << best_val << " -> " << best_path.string() << '\n';
    std::cout << "Test loss:     " << test_l << '\n';

    return 0;
}
