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
    std::vector<Obstacle> obstacles;  // updated each _runMPC() call via mpc_set_obstacles()
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

// =====================================================================
//  Batched candidate evaluation — all N candidates in one forward pass
// =====================================================================

// Batched state: [N] parallel rollout states stored as tensors
struct BatchedRollout {
    torch::Tensor x, y, yaw, vx, vy, oz;    // each [N]
    torch::Tensor joint_pos, joint_vel;      // [N, kJointDim]
};

static BatchedRollout init_batched(const RolloutState &s, int N) {
    BatchedRollout b;
    b.x   = torch::full({N}, s.x,   torch::kFloat64);
    b.y   = torch::full({N}, s.y,   torch::kFloat64);
    b.yaw = torch::full({N}, s.yaw, torch::kFloat64);
    b.vx  = torch::full({N}, s.vx,  torch::kFloat64);
    b.vy  = torch::full({N}, s.vy,  torch::kFloat64);
    b.oz  = torch::full({N}, s.oz,  torch::kFloat64);
    auto jp = torch::empty({N, kJointDim}, torch::kFloat64);
    auto jv = torch::empty({N, kJointDim}, torch::kFloat64);
    for (int i = 0; i < kJointDim; ++i) {
        jp.select(1,i).fill_(s.joint_pos[i]);
        jv.select(1,i).fill_(s.joint_vel[i]);
    }
    b.joint_pos = jp; b.joint_vel = jv;
    return b;
}

static torch::Tensor batched_input_vec(const BatchedRollout &b) {
    int N = (int)b.x.size(0);
    auto qw = torch::cos(b.yaw * 0.5).to(torch::kFloat32);
    auto qz = torch::sin(b.yaw * 0.5).to(torch::kFloat32);
    auto zero = torch::zeros({N}, torch::kFloat32);
    // [qw, 0, 0, qz, vx, vy, oz, joint_pos(35), joint_vel(35)] = 77-D
    return torch::stack({
        qw, zero, zero, qz,
        b.vx.to(torch::kFloat32), b.vy.to(torch::kFloat32), b.oz.to(torch::kFloat32)
    }, 1).to(torch::kFloat32);  // [N, 7] — need to cat with joints
}

static torch::Tensor batched_to_input(const BatchedRollout &b) {
    int N = (int)b.x.size(0);
    auto qw = torch::cos(b.yaw * 0.5).to(torch::kFloat32);
    auto qz = torch::sin(b.yaw * 0.5).to(torch::kFloat32);
    auto zero = torch::zeros({N, 1}, torch::kFloat32);
    auto base = torch::cat({
        qw.unsqueeze(1), zero, zero, qz.unsqueeze(1),
        b.vx.unsqueeze(1).to(torch::kFloat32),
        b.vy.unsqueeze(1).to(torch::kFloat32),
        b.oz.unsqueeze(1).to(torch::kFloat32)
    }, 1); // [N, 7]
    return torch::cat({base, b.joint_pos.to(torch::kFloat32),
                             b.joint_vel.to(torch::kFloat32)}, 1); // [N, 77]
}

static void batched_apply_delta(BatchedRollout &b, const torch::Tensor &delta) {
    // delta: [N, kOutputDim=76] on CPU
    auto d = delta.to(torch::kFloat64);
    b.x   += d.select(1, 0);
    b.y   += d.select(1, 1);
    b.yaw  = torch::atan2(torch::sin(b.yaw + d.select(1,2)),
                           torch::cos(b.yaw + d.select(1,2)));
    b.vx  += d.select(1, 3);
    b.vy  += d.select(1, 4);
    b.oz  += d.select(1, 5);
    b.joint_pos += d.narrow(1, 6, kJointDim).to(torch::kFloat64);
    b.joint_vel += d.narrow(1, 6 + kJointDim, kJointDim).to(torch::kFloat64);
}

// Predict delta for all N candidates in one batched forward pass
static torch::Tensor predict_delta_batched(
    TransformerTransitionModel &model,
    const Normaliser &norm,
    const torch::Tensor &seq,   // [N, seq_len, kTokenDim]
    torch::Device device)
{
    torch::NoGradGuard ng;
    auto [mu, _] = model->forward(seq.to(device));
    return norm.unnorm_target(mu.to(torch::kCPU)); // [N, kOutputDim]
}

// Build batched history sequences [N, seq_len, kTokenDim] from shared
// history prefix + per-candidate current token
static torch::Tensor build_batched_sequences(
    const std::deque<torch::Tensor> &hist_buf,
    int seq_len,
    const torch::Tensor &new_tokens)  // [N, kTokenDim]
{
    int N = (int)new_tokens.size(0);
    int hist_len = (int)hist_buf.size();
    int pad = seq_len - hist_len - 1; // -1 for the new token

    std::vector<torch::Tensor> parts;
    // Zero-pad
    if (pad > 0) {
        auto zeros = torch::zeros({N, pad, kTokenDim}, torch::kFloat32);
        parts.push_back(zeros);
    }
    // Shared history (broadcast to N)
    int copy_start = std::max(0, hist_len - (seq_len - 1));
    for (int i = copy_start; i < hist_len; ++i)
        parts.push_back(hist_buf[i].unsqueeze(0).expand({N, -1}).unsqueeze(1));
    // New per-candidate token
    parts.push_back(new_tokens.unsqueeze(1));
    return torch::cat(parts, 1); // [N, seq_len, kTokenDim]
}

static void evaluate_candidates_batched(
    MpcContext &ctx,
    const RolloutState &state,
    const std::vector<std::vector<ActionCmd>> &cands,
    double goal_x, double goal_y,
    std::vector<double> &costs)
{
    const auto &cfg = ctx.cfg;
    const int N = (int)cands.size();
    const int H = (int)cands[0].size();

    std::fill(costs.begin(), costs.end(), 0.0);

    auto br = init_batched(state, N);

    // We need to track per-candidate history sequences.
    // Start with shared history, then diverge as rollouts proceed.
    // seq: [N, seq_len, kTokenDim] — maintained across horizon steps
    // Initialize from shared history buffer
    int seq_len = ctx.history;
    int hist_len = (int)ctx.history_buf.size();
    int pad = seq_len - hist_len;

    std::vector<torch::Tensor> init_parts;
    if (pad > 0)
        init_parts.push_back(torch::zeros({N, pad, kTokenDim}, torch::kFloat32));
    for (int i = 0; i < hist_len; ++i)
        init_parts.push_back(ctx.history_buf[i].unsqueeze(0).expand({N, -1}).unsqueeze(1));
    // If history is empty, ensure we have at least seq_len slots
    auto seq = init_parts.empty()
        ? torch::zeros({N, seq_len, kTokenDim}, torch::kFloat32)
        : torch::cat(init_parts, 1); // [N, hist_len+pad, kTokenDim]
    // Trim to seq_len-1 (leave room for new token at each step)
    if (seq.size(1) > seq_len - 1)
        seq = seq.narrow(1, seq.size(1) - (seq_len - 1), seq_len - 1);

    for (int t = 0; t < H; ++t) {
        // Build action tensor [N, 3]
        auto act_data = torch::empty({N, kActionDim}, torch::kFloat32);
        auto act_acc = act_data.accessor<float, 2>();
        for (int c = 0; c < N; ++c) {
            act_acc[c][0] = (float)cands[c][t].vx;
            act_acc[c][1] = (float)cands[c][t].vy;
            act_acc[c][2] = (float)cands[c][t].yaw;
        }

        // Build input tokens: [N, kTokenDim]
        auto inp = batched_to_input(br);  // [N, 77]
        auto norm_inp = ctx.norm.norm_input(inp);    // [N, 77]
        auto norm_act = ctx.norm.norm_action(act_data); // [N, 3]
        auto new_tok = torch::cat({norm_inp, norm_act}, 1); // [N, 80]

        // Append to sequence and trim to seq_len
        auto full_seq = torch::cat({seq, new_tok.unsqueeze(1)}, 1);
        if (full_seq.size(1) > seq_len)
            full_seq = full_seq.narrow(1, full_seq.size(1) - seq_len, seq_len);
        seq = full_seq;

        // Single batched forward pass for all N candidates
        auto delta = predict_delta_batched(ctx.model, ctx.norm, seq, ctx.device); // [N, 76]
        batched_apply_delta(br, delta);

        // Compute costs (on CPU tensors)
        auto dx = br.x - goal_x;
        auto dy = br.y - goal_y;
        auto dist_sq = dx * dx + dy * dy;
        auto dist_sq_acc = dist_sq.accessor<double, 1>();

        auto heading_to_goal = torch::atan2(
            torch::full({N}, goal_y, torch::kFloat64) - br.y,
            torch::full({N}, goal_x, torch::kFloat64) - br.x);
        auto angle_err = torch::atan2(
            torch::sin(heading_to_goal - br.yaw),
            torch::cos(heading_to_goal - br.yaw));
        auto angle_sq_t = angle_err * angle_err;
        auto angle_sq = angle_sq_t.accessor<double, 1>();

        // Obstacle collision: CPU accessors — O(N * num_obstacles) per step, negligible.
        auto x_acc = br.x.accessor<double, 1>();
        auto y_acc = br.y.accessor<double, 1>();

        for (int c = 0; c < N; ++c) {
            costs[c] += cfg.w_pos * dist_sq_acc[c];
            costs[c] += cfg.w_heading * angle_sq[c];

            if (t > 0) {
                auto &a = cands[c][t], &pa = cands[c][t-1];
                double dvx = a.vx-pa.vx, dvy = a.vy-pa.vy, dyaw = a.yaw-pa.yaw;
                costs[c] += cfg.w_smooth * (dvx*dvx + dvy*dvy + dyaw*dyaw);
            }
            if (cands[c][t].vx < 0)
                costs[c] += cfg.w_backward * cands[c][t].vx * cands[c][t].vx;

            // Soft repulsive obstacle cost: linear penalty based on penetration depth.
            // Binary hard cost (w=1000) caused the CEM to treat all obstacle-adjacent
            // trajectories as equally bad, causing the planner to converge to near-zero
            // velocity. Soft cost preserves gradient so elites guide away from obstacles.
            for (const auto &obs : ctx.obstacles) {
                const double obs_r = obs.type.find("box") != std::string::npos
                    ? std::sqrt((double)obs.size[0]*obs.size[0] + (double)obs.size[1]*obs.size[1])
                    : (double)obs.size[0];
                const double min_safe = cfg.base_radius + obs_r + cfg.clearance;
                const double odx = x_acc[c] - obs.position[0];
                const double ody = y_acc[c] - obs.position[1];
                const double dist = std::sqrt(odx*odx + ody*ody);
                if (dist < min_safe) {
                    // pen in [0,1]: 0 at safe boundary, 1 at full overlap
                    const double pen = (min_safe - dist) / min_safe;
                    costs[c] += cfg.w_collision * pen;
                }
            }
        }
    }

    // Terminal cost
    auto tdx = br.x - goal_x;
    auto tdy = br.y - goal_y;
    auto term_sq_t = tdx*tdx + tdy*tdy;
    auto term_sq = term_sq_t.accessor<double, 1>();
    for (int c = 0; c < N; ++c)
        costs[c] += cfg.w_terminal * term_sq[c];
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

        evaluate_candidates_batched(ctx, state, cands, goal_x, goal_y, costs);

        for (int c = 0; c < N; ++c) {
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

void mpc_set_obstacles(MpcContext &ctx, const std::vector<Obstacle> &obstacles) {
    ctx.obstacles = obstacles;
}

int mpc_obstacle_count(const MpcContext &ctx) {
    return (int)ctx.obstacles.size();
}
