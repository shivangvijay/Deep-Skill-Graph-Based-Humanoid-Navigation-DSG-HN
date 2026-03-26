#include "dsc.h"

DeepSkillChaining::DeepSkillChaining(
    std::shared_ptr<TrainEnvironment> env,
    torch::Device device,
    std::array<float, 3> global_goal,
    Config cfg)
    : _env(env)
    , _device(device)
    , _global_goal(global_goal)
    , _cfg(std::move(cfg))
    , _poo(env, _cfg.poo_layers, device, _cfg.lr_poo, _cfg.tau, _cfg.gamma, _cfg.batch_size)
{}

void DeepSkillChaining::loadGlobalOption(const std::string &actor_path,
                                          const std::string &critic1_path,
                                          const std::string &critic2_path)
{
    _skills.emplace_back(_makeSkill());
    _skills[0].setAlwaysAvailable();
    torch::load(_skills[0].agent().actor_local,    actor_path);
    torch::load(_skills[0].agent().critic_local_1, critic1_path);
    torch::load(_skills[0].agent().critic_local_2, critic2_path);
    _poo.addOption(0.0f);
    _poo.hardCopy();
    std::cout << "=== Loaded global option oG from " << actor_path << " ===" << std::endl;
}

int DeepSkillChaining::train()
{
    if (_skills.empty()) // must have a global option loaded before training chain skills
    {
        std::cerr << "Error: call loadGlobalOption() before train()." << std::endl;
        return 0;
    }

    _env->setGoal(_global_goal); // set the default environment goal to be the global goal

    // === og1: terminal chain skill, β = reach global goal ===
    std::cout << "\n=== Training og1 (terminal chain skill) ===" << std::endl;
    _skills.emplace_back(_makeSkill());
    _skills[1].initFromSkill(_skills[0]); // copy global option weights as a warm start for og1
    _skills[1].learn(_cfg.steps_per_episode, _cfg.gestation_train_steps,
                     _cfg.gestation_n, _cfg.last_k,
                     _cfg.refinement_eps, _cfg.nu, /*next_skill=*/nullptr, _cfg.start_noise_radius);
    _env->setGoal(_global_goal); // reset goal to global goal 
    _poo.addOption(0.0f); // add og1 to POO with initial value 0.0 (will be updated during POO learning)

    // TODO: call poo.learn() here to update POO? 

    if (_startCovered())
    {
        std::cout << "Start covered by og1. Chain complete." << std::endl;
        return (int)_skills.size();
    }

    // === og2..ogN: chain backward, each terminating at previous skill's initiation set ===
    for (int i = 2; i < _cfg.max_skills + 1; ++i)
    {
        const Skill *next = &_skills[i - 1];
        std::cout << "\n=== Training og" << i
                  << " (terminates at og" << i - 1 << "'s initiation set) ===" << std::endl;

        _skills.emplace_back(_makeSkill());
        _skills[i].initFromSkill(_skills[0]); // every skill initialized from global option weights as a warm start
        _skills[i].learn(_cfg.steps_per_episode, _cfg.gestation_train_steps,
                         _cfg.gestation_n, _cfg.last_k,
                         _cfg.refinement_eps, _cfg.nu, next, _cfg.start_noise_radius);
        _env->setGoal(_global_goal);
        _poo.addOption(0.0f);
        // TODO: call poo.learn() here to update POO? 

        if (_startCovered())
        {
            std::cout << "Start covered by og" << i << ". DSC chain complete." << std::endl;
            return (int)_skills.size();
        }
    }

    std::cout << "Max skills reached without covering start." << std::endl;
    return (int)_skills.size();
}

// this is just to train poo
float DeepSkillChaining::execute()
{
    torch::Tensor state      = _env->reset();
    float         total_reward = 0.0f;
    bool          episode_done = false;

    while (!episode_done)
    {
        int    opt_idx    = _poo.getOption(state);
        Skill &skill      = _skills[opt_idx];
        float  cum_reward = 0.0f;
        int    steps      = 0;
        torch::Tensor last_state = state;

        for (int t = 0; t < _cfg.steps_per_episode; ++t)
        {
            auto [scaled_action, _] = skill.agent().getAction(state, /*eval=*/true);
            auto [next_state, reward, done_t] = _env->step(scaled_action);
            cum_reward += reward.item<float>();
            steps++;
            last_state = next_state;

            bool env_done = done_t.item<float>() > 0.5f;

            // β: oG and og1 terminate at global goal; ogk (k≥2) enters og(k-1)'s initiation set
            bool beta = (opt_idx <= 1)
                ? env_done
                : _skills[opt_idx - 1].canStart(next_state);

            state = next_state;
            if (beta || env_done)
            {
                episode_done = env_done;
                break;
            }
        }

        total_reward += cum_reward;
        _poo.addExperience(state, opt_idx,
                           torch::tensor({cum_reward}, torch::kFloat32),
                           last_state,
                           torch::tensor({episode_done ? 1.0f : 0.0f}, torch::kFloat32),
                           steps);
        _poo.learn();
    }

    return total_reward;
}

void DeepSkillChaining::save(const std::string &dir) const
{
    std::filesystem::create_directories(dir);
    for (size_t i = 0; i < _skills.size(); ++i)
    {
        std::string prefix = dir + "/skill_" + std::to_string(i);
        _skills[i].save(prefix + "_actor.pt",
                        prefix + "_critic1.pt",
                        prefix + "_critic2.pt",
                        prefix + "_classifier.svm");
    }
    std::cout << "Saved " << _skills.size() << " skills to " << dir << std::endl;
}

void DeepSkillChaining::load(const std::string &dir, int num_skills)
{
    _skills.clear();
    for (int i = 0; i < num_skills; ++i)
    {
        _skills.emplace_back(_makeSkill());
        std::string prefix = dir + "/skill_" + std::to_string(i);
        _skills[i].load(prefix + "_actor.pt",
                        prefix + "_critic1.pt",
                        prefix + "_critic2.pt",
                        prefix + "_classifier.svm");
        _poo.addOption(0.0f);
    }
    _poo.hardCopy();
    std::cout << "Loaded " << num_skills << " skills from " << dir << std::endl;
}

int DeepSkillChaining::numSkills() const
{
    return (int)_skills.size();
}

Skill DeepSkillChaining::_makeSkill()
{
    return Skill(_env, _cfg.actor_layers, _cfg.critic_layers, _device,
                 _cfg.lr_actor, _cfg.lr_critic, _cfg.tau, _cfg.gamma,
                 _cfg.batch_size, _cfg.actor_update_freq);
}

bool DeepSkillChaining::_startCovered()
{
    torch::Tensor state = _env->reset();
    return _skills.back().canStart(state);
}

/************************************** main **************************************/

#define SCENE_FILE "../config/scene/test_scene.xml"
#define OG_ACTOR   "best_actor.pt"
#define OG_CRITIC1 "best_critic_1.pt"
#define OG_CRITIC2 "best_critic_2.pt"

#define X_MIN -5.0f
#define X_MAX  5.0f
#define Y_MIN -5.0f
#define Y_MAX  5.0f

static const std::array<float, 3> GLOBAL_GOAL = {3.0f, 3.0f, 0.0f};

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    torch::Device device(torch::kCPU);
    if (torch::cuda::is_available())
    {
        std::cout << "CUDA available — training on GPU." << std::endl;
        device = torch::Device(torch::kCUDA);
    }

    auto robot_bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, /*render=*/false);
    auto train_env = std::make_shared<TrainEnvironment>(robot_bridge, 1000);

    DeepSkillChaining::Config cfg;
    cfg.gestation_n    = 10;
    cfg.last_k         = 10;
    cfg.refinement_eps = 20;
    cfg.nu             = 0.1;
    cfg.max_skills     = 6;

    DeepSkillChaining dsc(train_env, device, GLOBAL_GOAL, cfg);
    dsc.loadGlobalOption(OG_ACTOR, OG_CRITIC1, OG_CRITIC2);

    int n = dsc.train();
    std::cout << "\nTraining complete: " << n << " skill(s) in chain." << std::endl;

    dsc.save("dsc_models");

    std::cout << "\n=== Evaluation (20 episodes) ===" << std::endl;
    float total = 0.0f;
    for (int i = 0; i < 20; ++i)
    {
        float r = dsc.execute();
        total += r;
        std::cout << "  Episode " << i + 1 << ": reward = " << r << std::endl;
    }
    std::cout << "Mean reward: " << (total / 20.0f) << std::endl;

    return 0;
}
