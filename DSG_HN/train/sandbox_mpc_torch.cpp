// Torch-only translation unit for sandbox_mpc.
// Compiled with libtorch include paths (no spdlog / robot_bridge here).
#include "sandbox_mpc_torch.h"

#include <torch/torch.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <unordered_map>

static constexpr int kJointDim      = 35;
static constexpr int kInputStateDim = 7 + (2 * kJointDim);  // 77
static constexpr int kActionDim     = 3;
static constexpr int kTokenDim      = kInputStateDim + kActionDim; // 80
static constexpr int kOutputDim     = 6 + (2 * kJointDim);  // 76

static constexpr double kVxMax  =  1.0, kVxMin  = -0.5;
static constexpr double kVyMax  =  0.3, kVyMin  = -0.3;
static constexpr double kYawMax =  1.0, kYawMin = -1.0;

// =====================================================================
//  Model
// =====================================================================
static void init_weights(torch::nn::Module &m) {
    if (auto *lin = m.as<torch::nn::Linear>()) {
        torch::NoGradGuard g;
        torch::nn::init::xavier_uniform_(lin->weight);
        if (lin->bias.defined()) torch::nn::init::constant_(lin->bias, 0.0);
    }
}

struct TransformerBlockImpl : torch::nn::Module {
    TransformerBlockImpl(int64_t d, int64_t h, double dr) {
        attn = register_module("attn",
            torch::nn::MultiheadAttention(torch::nn::MultiheadAttentionOptions(d, h).dropout(dr)));
        ln1 = register_module("ln1", torch::nn::LayerNorm(torch::nn::LayerNormOptions({d})));
        ff  = register_module("ff", torch::nn::Sequential(
            torch::nn::Linear(d, 4*d), torch::nn::SiLU(),
            torch::nn::Dropout(torch::nn::DropoutOptions(dr)),
            torch::nn::Linear(4*d, d),
            torch::nn::Dropout(torch::nn::DropoutOptions(dr))));
        ln2 = register_module("ln2", torch::nn::LayerNorm(torch::nn::LayerNormOptions({d})));
    }
    torch::Tensor forward(torch::Tensor x, const torch::Tensor &mask) {
        auto [ao, _] = attn->forward(x, x, x, {}, false, mask);
        x = ln1->forward(x + ao);
        x = ln2->forward(x + ff->forward(x));
        return x;
    }
    torch::nn::MultiheadAttention attn{nullptr};
    torch::nn::LayerNorm ln1{nullptr}, ln2{nullptr};
    torch::nn::Sequential ff{nullptr};
};
TORCH_MODULE(TransformerBlock);

struct TransformerTransitionModelImpl : torch::nn::Module {
    TransformerTransitionModelImpl(int64_t token_dim, int64_t output_dim,
                                   int64_t seq_len, int64_t d_model,
                                   int64_t n_heads, int64_t n_layers, double dropout)
        : seq_len_(seq_len) {
        token_proj = register_module("token_proj", torch::nn::Linear(token_dim, d_model));
        pos_embed  = register_module("pos_embed",
            torch::nn::Embedding(torch::nn::EmbeddingOptions(seq_len, d_model)));
        drop = register_module("drop", torch::nn::Dropout(torch::nn::DropoutOptions(dropout)));
        for (int64_t i = 0; i < n_layers; ++i)
            blocks.push_back(register_module("block_"+std::to_string(i),
                TransformerBlock(d_model, n_heads, dropout)));
        ln_final = register_module("ln_final", torch::nn::LayerNorm(torch::nn::LayerNormOptions({d_model})));
        mu_head  = register_module("mu_head",  torch::nn::Linear(d_model, output_dim));
        lv_head  = register_module("lv_head",  torch::nn::Linear(d_model, output_dim));
        apply(init_weights);
        causal_mask_ = torch::zeros({seq_len, seq_len});
        causal_mask_.masked_fill_(torch::ones({seq_len,seq_len}).triu(1).to(torch::kBool),
                                  -std::numeric_limits<float>::infinity());
    }
    std::pair<torch::Tensor, torch::Tensor> forward(torch::Tensor x) {
        auto S = x.size(1);
        x = token_proj->forward(x);
        auto pos = torch::arange(S, torch::TensorOptions().dtype(torch::kLong).device(x.device()));
        x = x + pos_embed->forward(pos).unsqueeze(0);
        x = drop->forward(x);
        x = x.transpose(0,1);
        auto mask = causal_mask_.to(x.device());
        for (auto &b : blocks) x = b->forward(x, mask);
        x = x.transpose(0,1);
        x = ln_final->forward(x);
        auto last = x.select(1, S-1);
        return {mu_head->forward(last), lv_head->forward(last)};
    }
    int64_t seq_len_;
    torch::nn::Linear token_proj{nullptr}, mu_head{nullptr}, lv_head{nullptr};
    torch::nn::Embedding pos_embed{nullptr};
    torch::nn::Dropout drop{nullptr};
    torch::nn::LayerNorm ln_final{nullptr};
    std::vector<TransformerBlock> blocks;
    torch::Tensor causal_mask_;
};
TORCH_MODULE(TransformerTransitionModel);

// =====================================================================
//  Normaliser
// =====================================================================
struct Normaliser {
    torch::Tensor input_mean, input_std, action_mean, action_std, target_mean, target_std;
    torch::Tensor norm_input (const torch::Tensor &x) const { return (x-input_mean)/input_std; }
    torch::Tensor norm_action(const torch::Tensor &x) const { return (x-action_mean)/action_std; }
    torch::Tensor unnorm_target(const torch::Tensor &x) const { return x*target_std+target_mean; }

    static Normaliser load(const std::string &path) {
        std::ifstream in(path); if (!in.is_open()) throw std::runtime_error("Cannot open: "+path);
        Normaliser n; std::unordered_map<std::string,torch::Tensor> m; std::string line;
        while (std::getline(in,line)) {
            if (line.empty()) continue;
            std::stringstream ss(line); std::string name; ss >> name;
            std::vector<float> v; float val; while (ss>>val) v.push_back(val);
            auto t = torch::empty({(int64_t)v.size()}, torch::kFloat32);
            std::memcpy(t.data_ptr<float>(), v.data(), v.size()*sizeof(float));
            m.emplace(std::move(name), std::move(t));
        }
        n.input_mean=m.at("input_mean"); n.input_std=m.at("input_std");
        n.action_mean=m.at("action_mean"); n.action_std=m.at("action_std");
        n.target_mean=m.at("target_mean"); n.target_std=m.at("target_std");
        return n;
    }
};

// =====================================================================
//  Helpers
// =====================================================================
static double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

static torch::Tensor rollout_to_input_vec(const RolloutState &s) {
    float qw=(float)std::cos(s.yaw*0.5), qz=(float)std::sin(s.yaw*0.5);
    std::vector<float> f = {qw, 0.f, 0.f, qz, (float)s.vx, (float)s.vy, (float)s.oz};
    for (double v : s.joint_pos) f.push_back((float)v);
    for (double v : s.joint_vel) f.push_back((float)v);
    return torch::tensor(f, torch::kFloat32);
}

static void apply_delta(RolloutState &s, const torch::Tensor &d) {
    s.x  += d[0].item<double>(); s.y += d[1].item<double>();
    s.yaw = wrap_angle(s.yaw + d[2].item<double>());
    s.vx += d[3].item<double>(); s.vy += d[4].item<double>(); s.oz += d[5].item<double>();
    for (int i=0;i<kJointDim;++i) s.joint_pos[i] += d[6+i].item<double>();
    for (int i=0;i<kJointDim;++i) s.joint_vel[i] += d[6+kJointDim+i].item<double>();
}

static torch::Tensor make_token(const Normaliser &norm,
                                const torch::Tensor &sinp,
                                const torch::Tensor &action) {
    return torch::cat({norm.norm_input(sinp.unsqueeze(0)).squeeze(0),
                       norm.norm_action(action.unsqueeze(0)).squeeze(0)});
}

static torch::Tensor predict_delta(TransformerTransitionModel &model,
                                   const Normaliser &norm,
                                   const std::deque<torch::Tensor> &history,
                                   int64_t seq_len,
                                   torch::Device device) {
    std::vector<torch::Tensor> tokens;
    int64_t pad = seq_len - (int64_t)history.size();
    auto zero = torch::zeros({kTokenDim}, torch::kFloat32);
    for (int64_t i=0;i<pad;++i) tokens.push_back(zero);
    for (auto &t : history) tokens.push_back(t);
    auto seq = torch::stack(tokens).unsqueeze(0).to(device);
    torch::NoGradGuard ng;
    auto [mu, _] = model->forward(seq);
    return norm.unnorm_target(mu.to(torch::kCPU)).squeeze(0);
}

// =====================================================================
//  MpcContext — opaque handle exposed via header
// =====================================================================
struct MpcContext {
    TransformerTransitionModel model{nullptr};
    Normaliser norm;
    torch::Device device{torch::kCPU};
    int history;
    std::deque<torch::Tensor> history_buf;
    std::mt19937 rng;
    MpcConfig cfg;
    std::vector<ActionCmd> prev_best_actions;
};

void MpcContextDeleter::operator()(MpcContext *p) const { delete p; }

MpcContextPtr mpc_create(
    const std::string &ckpt_path,
    const std::string &normaliser_path,
    int history, int d_model, int n_heads, int n_layers,
    const MpcConfig &cfg)
{
    MpcContextPtr ctx(new MpcContext());
    ctx->device = torch::Device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);
    ctx->history = history;
    ctx->cfg = cfg;
    ctx->norm = Normaliser::load(normaliser_path);
    ctx->model = TransformerTransitionModel(kTokenDim, kOutputDim, history, d_model, n_heads, n_layers, 0.0);
    torch::load(ctx->model, ckpt_path);
    ctx->model->to(ctx->device);
    ctx->model->eval();
    return ctx;
}

static ActionCmd sample_action(std::mt19937 &rng) {
    std::uniform_real_distribution<double> dvx(kVxMin, kVxMax);
    std::uniform_real_distribution<double> dvy(kVyMin, kVyMax);
    std::uniform_real_distribution<double> dyaw(kYawMin, kYawMax);
    return {dvx(rng), dvy(rng), dyaw(rng)};
}

static double evaluate_candidate(MpcContext &ctx,
                                 const RolloutState &state,
                                 const std::vector<ActionCmd> &actions,
                                 double goal_x, double goal_y)
{
    const auto &cfg = ctx.cfg;
    std::deque<torch::Tensor> hist = ctx.history_buf;
    RolloutState cur = state;
    double cost = 0.0;

    for (int t = 0; t < (int)actions.size(); ++t) {
        auto &a = actions[t];
        auto sinp = rollout_to_input_vec(cur);
        auto act = torch::tensor({(float)a.vx, (float)a.vy, (float)a.yaw}, torch::kFloat32);
        hist.push_back(make_token(ctx.norm, sinp, act));
        if ((int64_t)hist.size() > ctx.history) hist.pop_front();
        auto delta = predict_delta(ctx.model, ctx.norm, hist, ctx.history, ctx.device);
        apply_delta(cur, delta);

        double dx = cur.x - goal_x, dy = cur.y - goal_y;
        cost += cfg.w_pos * (dx * dx + dy * dy);

        double angle_err = wrap_angle(std::atan2(goal_y - cur.y, goal_x - cur.x) - cur.yaw);
        cost += cfg.w_heading * angle_err * angle_err;

        if (t > 0) {
            auto &pa = actions[t - 1];
            double dvx = a.vx - pa.vx, dvy = a.vy - pa.vy, dyaw = a.yaw - pa.yaw;
            cost += cfg.w_smooth * (dvx * dvx + dvy * dvy + dyaw * dyaw);
        }

        if (a.vx < 0)
            cost += cfg.w_backward * a.vx * a.vx;
    }

    double tdx = cur.x - goal_x, tdy = cur.y - goal_y;
    cost += cfg.w_terminal * (tdx * tdx + tdy * tdy);
    return cost;
}

ActionCmd mpc_plan(MpcContext &ctx,
                   const RolloutState &state,
                   double goal_x, double goal_y,
                   unsigned seed_offset)
{
    ctx.rng.seed(seed_offset);
    const auto &cfg = ctx.cfg;
    const int H = cfg.horizon;
    const int N = cfg.candidates;
    const int E = cfg.cem_elites;

    std::vector<std::array<double, 3>> mu(H), sigma(H);

    bool warm = (int)ctx.prev_best_actions.size() == H;
    for (int t = 0; t < H; ++t) {
        if (warm && t < H - 1) {
            auto &pa = ctx.prev_best_actions[t + 1];
            mu[t] = {pa.vx, pa.vy, pa.yaw};
        } else {
            mu[t] = {(kVxMax + kVxMin) * 0.5,
                      (kVyMax + kVyMin) * 0.5,
                      (kYawMax + kYawMin) * 0.5};
        }
        sigma[t] = {(kVxMax - kVxMin) * 0.5,
                     (kVyMax - kVyMin) * 0.5,
                     (kYawMax - kYawMin) * 0.5};
    }

    double best_cost_overall = std::numeric_limits<double>::infinity();
    std::vector<ActionCmd> best_seq(H);

    std::vector<std::vector<ActionCmd>> cands(N, std::vector<ActionCmd>(H));
    std::vector<double> costs(N);

    for (int round = 0; round < cfg.cem_rounds; ++round) {
        for (int c = 0; c < N; ++c) {
            for (int t = 0; t < H; ++t) {
                std::normal_distribution<double> d0(mu[t][0], sigma[t][0]);
                std::normal_distribution<double> d1(mu[t][1], sigma[t][1]);
                std::normal_distribution<double> d2(mu[t][2], sigma[t][2]);
                cands[c][t] = {std::clamp(d0(ctx.rng), kVxMin, kVxMax),
                               std::clamp(d1(ctx.rng), kVyMin, kVyMax),
                               std::clamp(d2(ctx.rng), kYawMin, kYawMax)};
            }
        }

        for (int c = 0; c < N; ++c) {
            costs[c] = evaluate_candidate(ctx, state, cands[c], goal_x, goal_y);
            if (costs[c] < best_cost_overall) {
                best_cost_overall = costs[c];
                best_seq = cands[c];
            }
        }

        if (round < cfg.cem_rounds - 1) {
            std::vector<int> idx(N);
            std::iota(idx.begin(), idx.end(), 0);
            std::partial_sort(idx.begin(), idx.begin() + E, idx.end(),
                [&](int a, int b) { return costs[a] < costs[b]; });

            for (int t = 0; t < H; ++t) {
                double s0 = 0, s1 = 0, s2 = 0;
                for (int e = 0; e < E; ++e) {
                    auto &a = cands[idx[e]][t];
                    s0 += a.vx; s1 += a.vy; s2 += a.yaw;
                }
                mu[t] = {s0 / E, s1 / E, s2 / E};

                double v0 = 0, v1 = 0, v2 = 0;
                for (int e = 0; e < E; ++e) {
                    auto &a = cands[idx[e]][t];
                    v0 += (a.vx  - mu[t][0]) * (a.vx  - mu[t][0]);
                    v1 += (a.vy  - mu[t][1]) * (a.vy  - mu[t][1]);
                    v2 += (a.yaw - mu[t][2]) * (a.yaw - mu[t][2]);
                }
                sigma[t] = {std::sqrt(v0 / E) + 1e-6,
                            std::sqrt(v1 / E) + 1e-6,
                            std::sqrt(v2 / E) + 1e-6};
            }
        }
    }

    ctx.prev_best_actions = best_seq;
    return best_seq[0];
}

void mpc_update_history(MpcContext &ctx,
                        const RolloutState &state,
                        const ActionCmd &cmd)
{
    auto sinp = rollout_to_input_vec(state);
    auto act  = torch::tensor({(float)cmd.vx,(float)cmd.vy,(float)cmd.yaw}, torch::kFloat32);
    ctx.history_buf.push_back(make_token(ctx.norm, sinp, act));
    if ((int64_t)ctx.history_buf.size() > ctx.history) ctx.history_buf.pop_front();
}

void mpc_clear_history(MpcContext &ctx) {
    ctx.history_buf.clear();
}
