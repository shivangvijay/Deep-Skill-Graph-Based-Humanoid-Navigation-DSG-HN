#include <torch/torch.h>

#include <boost/program_options.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
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

static constexpr int kRawStateDim   = 13;
static constexpr int kJointDim      = 35;
static constexpr int kFullStateDim  = kRawStateDim + (2 * kJointDim);
static constexpr int kInputStateDim = 7 + (2 * kJointDim);
static constexpr int kActionDim     = 3;
static constexpr int kTokenDim      = kInputStateDim + kActionDim;
static constexpr int kOutputDim     = 6 + (2 * kJointDim);
static constexpr double kBoundaryGapSeconds = 0.15;

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
//  CSV helpers
// =====================================================================
static std::string two_digit(int v)
{
    std::ostringstream o; o << std::setw(2) << std::setfill('0') << v; return o.str();
}

static std::vector<std::string> current_state_columns()
{
    std::vector<std::string> c = {"x","y","z","qw","qx","qy","qz","vx","vy","vz","omega_x","omega_y","omega_z"};
    for (int i = 0; i < kJointDim; ++i) c.push_back("joint_pos_" + two_digit(i));
    for (int i = 0; i < kJointDim; ++i) c.push_back("joint_vel_" + two_digit(i));
    return c;
}
static std::vector<std::string> next_state_columns()
{
    std::vector<std::string> c = {"next_x","next_y","next_z","next_qw","next_qx","next_qy","next_qz","next_vx","next_vy","next_vz","next_omega_x","next_omega_y","next_omega_z"};
    for (int i = 0; i < kJointDim; ++i) c.push_back("next_joint_pos_" + two_digit(i));
    for (int i = 0; i < kJointDim; ++i) c.push_back("next_joint_vel_" + two_digit(i));
    return c;
}
static std::vector<std::string> action_columns() { return {"cmd_vx","cmd_vy","cmd_yaw"}; }

static std::vector<std::string> split_csv_line(const std::string &line)
{
    std::vector<std::string> cells; std::string cell; std::stringstream ss(line);
    while (std::getline(ss, cell, ',')) cells.push_back(cell);
    return cells;
}
static std::vector<int> lookup_indices(const std::unordered_map<std::string,int> &h,
                                       const std::vector<std::string> &cols)
{
    std::vector<int> idx; idx.reserve(cols.size());
    for (auto &c : cols) { auto it = h.find(c); if (it==h.end()) throw std::runtime_error("Missing: "+c); idx.push_back(it->second); }
    return idx;
}
static torch::Tensor make_tensor(const std::vector<float> &d, int64_t r, int64_t c)
{
    auto t = torch::empty({r,c}, torch::kFloat32);
    std::memcpy(t.data_ptr<float>(), d.data(), d.size()*sizeof(float)); return t;
}
static torch::Tensor make_double_tensor(const std::vector<double> &d, int64_t r, int64_t c)
{
    auto t = torch::empty({r,c}, torch::kFloat64);
    std::memcpy(t.data_ptr<double>(), d.data(), d.size()*sizeof(double)); return t;
}

struct LoadedCsv { torch::Tensor states, actions, next_states, timestamps; };

static LoadedCsv load_csv(const std::string &path)
{
    std::ifstream f(path); if (!f.is_open()) throw std::runtime_error("Cannot open: "+path);
    std::string hdr; if (!std::getline(f,hdr)) throw std::runtime_error("Empty CSV");
    auto headers = split_csv_line(hdr);
    std::unordered_map<std::string,int> hi;
    for (int i=0;i<(int)headers.size();++i) hi.emplace(headers[i],i);
    auto si=lookup_indices(hi,current_state_columns()), ai=lookup_indices(hi,action_columns()),
         ni=lookup_indices(hi,next_state_columns());
    auto ti=hi.find("timestamp_s"); if(ti==hi.end()) throw std::runtime_error("No timestamp_s");
    int tsi=ti->second;
    std::vector<float> sf,af,nf; std::vector<double> tf; std::string line;
    while(std::getline(f,line)){
        if(line.empty()) continue; auto cells=split_csv_line(line);
        if(cells.size()!=headers.size()) continue;
        try{ tf.push_back(std::stod(cells[tsi]));
             for(int i:si) sf.push_back(std::stof(cells[i]));
             for(int i:ai) af.push_back(std::stof(cells[i]));
             for(int i:ni) nf.push_back(std::stof(cells[i]));
        }catch(...){continue;}
    }
    int64_t n=(int64_t)tf.size(); if(!n) throw std::runtime_error("No rows");
    return {make_tensor(sf,n,kFullStateDim),make_tensor(af,n,kActionDim),
            make_tensor(nf,n,kFullStateDim),make_double_tensor(tf,n,1)};
}

// =====================================================================
//  Normaliser
// =====================================================================
struct Normaliser
{
    torch::Tensor input_mean, input_std, action_mean, action_std, target_mean, target_std;
    torch::Tensor norm_input(const torch::Tensor &x)  const { return (x-input_mean)/input_std; }
    torch::Tensor norm_action(const torch::Tensor &x) const { return (x-action_mean)/action_std; }
    torch::Tensor unnorm_target(const torch::Tensor &x) const { return x*target_std+target_mean; }

    static Normaliser load(const std::string &path)
    {
        std::ifstream in(path); if(!in.is_open()) throw std::runtime_error("Cannot open: "+path);
        Normaliser n; std::unordered_map<std::string,torch::Tensor> m; std::string line;
        while(std::getline(in,line)){
            if(line.empty()) continue; std::stringstream ss(line); std::string name; ss>>name;
            std::vector<float> v; float val; while(ss>>val) v.push_back(val);
            auto t=torch::empty({(int64_t)v.size()},torch::kFloat32);
            std::memcpy(t.data_ptr<float>(),v.data(),v.size()*sizeof(float));
            m.emplace(std::move(name),std::move(t));
        }
        n.input_mean=m.at("input_mean"); n.input_std=m.at("input_std");
        n.action_mean=m.at("action_mean"); n.action_std=m.at("action_std");
        n.target_mean=m.at("target_mean"); n.target_std=m.at("target_std");
        return n;
    }
};

// =====================================================================
//  Transformer model (must match training architecture exactly)
// =====================================================================
static void init_weights(torch::nn::Module &m)
{
    if (auto *lin = m.as<torch::nn::Linear>()) {
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
                torch::nn::MultiheadAttentionOptions(d_model, n_heads).dropout(dropout)));
        ln1 = register_module("ln1", torch::nn::LayerNorm(torch::nn::LayerNormOptions({d_model})));
        ff  = register_module("ff", torch::nn::Sequential(
            torch::nn::Linear(d_model, 4*d_model), torch::nn::SiLU(),
            torch::nn::Dropout(torch::nn::DropoutOptions(dropout)),
            torch::nn::Linear(4*d_model, d_model),
            torch::nn::Dropout(torch::nn::DropoutOptions(dropout))));
        ln2 = register_module("ln2", torch::nn::LayerNorm(torch::nn::LayerNormOptions({d_model})));
    }
    torch::Tensor forward(torch::Tensor x, const torch::Tensor &mask)
    {
        auto [ao, _] = attn->forward(x,x,x,{},false,mask);
        x = ln1->forward(x + ao);
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
            blocks.push_back(register_module("block_"+std::to_string(i),
                TransformerBlock(d_model, n_heads, dropout)));
        ln_final = register_module("ln_final", torch::nn::LayerNorm(torch::nn::LayerNormOptions({d_model})));
        mu_head  = register_module("mu_head", torch::nn::Linear(d_model, output_dim));
        lv_head  = register_module("lv_head", torch::nn::Linear(d_model, output_dim));
        apply(init_weights);
        causal_mask_ = torch::zeros({seq_len, seq_len});
        causal_mask_.masked_fill_(torch::ones({seq_len,seq_len}).triu(1).to(torch::kBool),
                                  -std::numeric_limits<float>::infinity());
    }

    std::pair<torch::Tensor, torch::Tensor> forward(torch::Tensor x)
    {
        auto S = x.size(1);
        x = token_proj->forward(x);
        auto positions = torch::arange(S, torch::TensorOptions().dtype(torch::kLong).device(x.device()));
        x = x + pos_embed->forward(positions).unsqueeze(0);
        x = drop->forward(x);
        x = x.transpose(0,1);
        auto mask = causal_mask_.to(x.device());
        for (auto &b : blocks) x = b->forward(x, mask);
        x = x.transpose(0,1);
        x = ln_final->forward(x);
        auto last = x.select(1, S-1);
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
//  Rollout state & helpers
// =====================================================================
static double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

static double yaw_from_row(const torch::Tensor &r)
{
    double qw=r[IDX_QW].item<double>(), qx=r[IDX_QX].item<double>(),
           qy=r[IDX_QY].item<double>(), qz=r[IDX_QZ].item<double>();
    return std::atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));
}

struct RolloutState
{
    double x=0, y=0, yaw=0, vx=0, vy=0, oz=0;
    std::vector<double> joint_pos, joint_vel;
};

static RolloutState state_from_row(const torch::Tensor &r)
{
    RolloutState s;
    s.x=r[IDX_X].item<double>(); s.y=r[IDX_Y].item<double>();
    s.yaw=yaw_from_row(r); s.vx=r[IDX_VX].item<double>();
    s.vy=r[IDX_VY].item<double>(); s.oz=r[IDX_OZ].item<double>();
    s.joint_pos.resize(kJointDim); s.joint_vel.resize(kJointDim);
    for(int i=0;i<kJointDim;++i){ s.joint_pos[i]=r[kRawStateDim+i].item<double>();
                                   s.joint_vel[i]=r[kRawStateDim+kJointDim+i].item<double>(); }
    return s;
}

static void apply_delta(RolloutState &s, const torch::Tensor &d)
{
    s.x+=d[0].item<double>(); s.y+=d[1].item<double>();
    s.yaw=wrap_angle(s.yaw+d[2].item<double>());
    s.vx+=d[3].item<double>(); s.vy+=d[4].item<double>(); s.oz+=d[5].item<double>();
    for(int i=0;i<kJointDim;++i) s.joint_pos[i]+=d[6+i].item<double>();
    for(int i=0;i<kJointDim;++i) s.joint_vel[i]+=d[6+kJointDim+i].item<double>();
}

static torch::Tensor state_to_input_vec(const RolloutState &s)
{
    float qw=(float)std::cos(s.yaw*0.5), qz=(float)std::sin(s.yaw*0.5);
    std::vector<float> f = {qw, 0.0f, 0.0f, qz, (float)s.vx, (float)s.vy, (float)s.oz};
    for (double v : s.joint_pos) f.push_back((float)v);
    for (double v : s.joint_vel) f.push_back((float)v);
    return torch::tensor(f, torch::kFloat32);
}

// Build a single normalised token from state + action
static torch::Tensor make_token(const Normaliser &norm,
                                const torch::Tensor &state_input, // [kInputStateDim]
                                const torch::Tensor &action)      // [kActionDim]
{
    return torch::cat({norm.norm_input(state_input.unsqueeze(0)).squeeze(0),
                       norm.norm_action(action.unsqueeze(0)).squeeze(0)});
}

// State features from a raw CSV row
static torch::Tensor state_to_input_from_row(const torch::Tensor &row)
{
    std::vector<torch::Tensor> f = {
        row[IDX_QW].unsqueeze(0), row[IDX_QX].unsqueeze(0),
        row[IDX_QY].unsqueeze(0), row[IDX_QZ].unsqueeze(0),
        row[IDX_VX].unsqueeze(0), row[IDX_VY].unsqueeze(0),
        row[IDX_OZ].unsqueeze(0)
    };
    for(int i=0;i<kJointDim;++i) f.push_back(row[kRawStateDim+i].unsqueeze(0));
    for(int i=0;i<kJointDim;++i) f.push_back(row[kRawStateDim+kJointDim+i].unsqueeze(0));
    return torch::cat(f);
}

// =====================================================================
//  Predict with history window
// =====================================================================
static torch::Tensor predict_delta(TransformerTransitionModel &model,
                                   const Normaliser &norm,
                                   const std::deque<torch::Tensor> &history, // deque of [kTokenDim] tokens
                                   int64_t seq_len,
                                   torch::Device device,
                                   bool sample)
{
    // Pad if history is shorter than seq_len
    std::vector<torch::Tensor> tokens;
    int64_t pad = seq_len - (int64_t)history.size();
    if (pad > 0)
    {
        auto zero_tok = torch::zeros({kTokenDim}, torch::kFloat32);
        for (int64_t i = 0; i < pad; ++i) tokens.push_back(zero_tok);
    }
    for (auto &t : history) tokens.push_back(t);

    auto seq = torch::stack(tokens).unsqueeze(0).to(device); // [1, seq_len, kTokenDim]

    torch::NoGradGuard ng;
    auto [mu_norm, lv_norm] = model->forward(seq);
    if (sample)
    {
        auto clamped = torch::clamp(lv_norm, -4.0, 4.0);
        auto noise = torch::exp(0.5*clamped) * torch::randn_like(mu_norm);
        return norm.unnorm_target((mu_norm+noise).to(torch::kCPU)).squeeze(0);
    }
    return norm.unnorm_target(mu_norm.to(torch::kCPU)).squeeze(0);
}

// =====================================================================
//  Metrics
// =====================================================================
struct Metrics
{
    double pos_xy=0, vel_xy=0, heading=0, omega_z=0, joint_pos_sq=0, joint_vel_sq=0;
    int count=0;
};

static void accumulate(Metrics &m, const RolloutState &p, const RolloutState &g)
{
    double dx=p.x-g.x, dy=p.y-g.y, dvx=p.vx-g.vx, dvy=p.vy-g.vy;
    m.pos_xy+=std::sqrt(dx*dx+dy*dy);
    m.vel_xy+=std::sqrt(dvx*dvx+dvy*dvy);
    m.heading+=std::abs(wrap_angle(p.yaw-g.yaw));
    m.omega_z+=std::abs(p.oz-g.oz);
    for(int i=0;i<kJointDim;++i){
        double pe=p.joint_pos[i]-g.joint_pos[i], ve=p.joint_vel[i]-g.joint_vel[i];
        m.joint_pos_sq+=pe*pe; m.joint_vel_sq+=ve*ve;
    }
    ++m.count;
}

static std::vector<double> parse_horizons(const std::string &s)
{
    std::vector<double> h; std::stringstream ss(s); std::string tok;
    while(std::getline(ss,tok,',')) if(!tok.empty()) h.push_back(std::stod(tok));
    if(h.empty()) throw std::runtime_error("No horizons");
    return h;
}

// =====================================================================
//  Evaluate one horizon
// =====================================================================
static Metrics evaluate_horizon(TransformerTransitionModel &model,
                                const LoadedCsv &data,
                                const Normaliser &norm,
                                double horizon_s,
                                int64_t seq_len,
                                torch::Device device,
                                bool sample)
{
    Metrics metrics;
    const int64_t n = data.states.size(0);

    std::vector<bool> boundary(n, false);
    boundary[0] = true;
    for (int64_t i = 1; i < n; ++i)
        boundary[i] = (data.timestamps[i].item<double>() - data.timestamps[i-1].item<double>()) > kBoundaryGapSeconds;

    for (int64_t start = 0; start < n; ++start)
    {
        double t0 = data.timestamps[start].item<double>();
        int64_t end = start + 1;
        while (end < n && (data.timestamps[end].item<double>() - t0) < horizon_s)
            ++end;
        if (end >= n) break;

        bool crosses = false;
        for (int64_t i = start+1; i <= end; ++i) if (boundary[i]) { crosses=true; break; }
        if (crosses) continue;

        // Seed the history with real tokens leading up to `start`
        std::deque<torch::Tensor> history;
        int64_t seed_begin = std::max((int64_t)0, start - seq_len + 1);
        // Make sure we don't cross a boundary while seeding
        for (int64_t i = seed_begin; i < start; ++i)
        {
            if (boundary[i+1]) { history.clear(); continue; }
            auto sinp = state_to_input_from_row(data.states[i]);
            history.push_back(make_token(norm, sinp, data.actions[i]));
        }

        RolloutState current = state_from_row(data.states[start]);

        for (int64_t i = start; i < end; ++i)
        {
            auto sinp = state_to_input_vec(current);
            auto tok = make_token(norm, sinp, data.actions[i]);
            history.push_back(tok);
            if ((int64_t)history.size() > seq_len) history.pop_front();

            auto delta = predict_delta(model, norm, history, seq_len, device, sample);
            apply_delta(current, delta);
        }

        accumulate(metrics, current, state_from_row(data.states[end]));
    }

    return metrics;
}

// =====================================================================
//  Main
// =====================================================================
int main(int argc, char **argv)
{
    po::options_description desc("Transformer delta transition rollout evaluation");
    desc.add_options()
        ("help,h", "show help")
        ("csv",        po::value<std::string>()->required(), "path to transitions.csv")
        ("checkpoint", po::value<std::string>()->required(), "path to best .pt checkpoint")
        ("normaliser", po::value<std::string>()->default_value(""), "normaliser.txt (defaults to checkpoint dir)")
        ("horizons",   po::value<std::string>()->default_value("1,5"), "rollout horizons in seconds")
        ("history",    po::value<int>()->default_value(10), "context window N (must match training)")
        ("d-model",    po::value<int>()->default_value(128), "transformer d_model")
        ("n-heads",    po::value<int>()->default_value(4), "attention heads")
        ("n-layers",   po::value<int>()->default_value(4), "transformer layers")
        ("sample",     po::bool_switch()->default_value(false), "sample from Gaussian")
        ("seed",       po::value<int>()->default_value(42), "random seed");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc,argv,desc),vm);
        if(vm.count("help")){std::cout<<desc;return 0;} po::notify(vm);
    } catch(const std::exception &e){std::cerr<<e.what()<<'\n'<<desc;return 1;}

    const auto csv_path   = vm["csv"].as<std::string>();
    const auto ckpt_path  = vm["checkpoint"].as<std::string>();
    const auto horizons   = parse_horizons(vm["horizons"].as<std::string>());
    const int  history    = vm["history"].as<int>();
    const int  d_model    = vm["d-model"].as<int>();
    const int  n_heads    = vm["n-heads"].as<int>();
    const int  n_layers   = vm["n-layers"].as<int>();
    const bool sample     = vm["sample"].as<bool>();
    torch::manual_seed(vm["seed"].as<int>());

    auto data = load_csv(csv_path);
    const int64_t total = data.states.size(0);
    const int64_t test_start = (int64_t)(total * 0.90);
    const int64_t test_count = total - test_start;
    if (test_count <= 0) { std::cerr << "Not enough data\n"; return 1; }

    LoadedCsv test_data{
        data.states.narrow(0, test_start, test_count).clone(),
        data.actions.narrow(0, test_start, test_count).clone(),
        data.next_states.narrow(0, test_start, test_count).clone(),
        data.timestamps.narrow(0, test_start, test_count).clone()
    };

    std::string norm_path = vm["normaliser"].as<std::string>();
    if (norm_path.empty()) {
        auto sl = ckpt_path.find_last_of('/');
        norm_path = (sl==std::string::npos ? "." : ckpt_path.substr(0,sl)) + "/normaliser.txt";
    }
    Normaliser norm = Normaliser::load(norm_path);

    const torch::Device device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);
    std::cout << "Device: " << device << '\n';

    TransformerTransitionModel model(kTokenDim, kOutputDim, history, d_model, n_heads, n_layers, 0.0);
    torch::load(model, ckpt_path);
    model->to(device);
    model->eval();

    std::cout << "Evaluating on test split: " << test_count << " rows\n";
    std::cout << "Model: Transformer (d=" << d_model << " h=" << n_heads << " L=" << n_layers << " seq=" << history << ")\n\n";

    for (double h : horizons)
    {
        auto m = evaluate_horizon(model, test_data, norm, h, history, device, sample);
        if (m.count == 0) { std::cout << "Rollout @ " << h << "s: no valid windows\n"; continue; }

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Rollout @ " << h << "s (" << m.count << " windows)\n";
        std::cout << "  XY error       = " << (m.pos_xy/m.count)  << " m\n";
        std::cout << "  Vel error      = " << (m.vel_xy/m.count)  << " m/s\n";
        std::cout << "  Heading error  = " << (m.heading/m.count) << " rad\n";
        std::cout << "  Omega_z error  = " << (m.omega_z/m.count) << " rad/s\n";
        std::cout << "  Joint pos RMSE = " << std::sqrt(m.joint_pos_sq/(m.count*kJointDim)) << " rad\n";
        std::cout << "  Joint vel RMSE = " << std::sqrt(m.joint_vel_sq/(m.count*kJointDim)) << " rad/s\n\n";
    }

    return 0;
}
