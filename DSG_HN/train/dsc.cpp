#include "dsc.h"

DeepSkillChaining::DeepSkillChaining(
    std::shared_ptr<RobotBridgeTrain> robot_bridge,
    torch::Device device,
    AbstractedState global_goal,
    AbstractedState global_start,
    const std::string &pretrain_actor_path,
    const std::string &pretrain_critic1_path,
    const std::string &pretrain_critic2_path,
    Config cfg)
    : _robot_bridge(robot_bridge), _device(device), _global_goal(global_goal), _global_start(global_start), _cfg(std::move(cfg)),
        _env(std::make_shared<TrainEnvironment>(_robot_bridge, cfg.steps_per_episode)), 
        _poo(_env, _cfg.poo_layers, device, _cfg.lr_poo, _cfg.tau, _cfg.gamma, _cfg.batch_size)
{
    loadGlobalOption(pretrain_actor_path, pretrain_critic1_path, pretrain_critic2_path);

    // todo: put new skill here
    _skills.push_back(_makeSkill(false, nullptr)); // this is the goal option getting pushed
    _poo.addOption(0.0f);
    _poo.hardCopy();
    _unfinished_option_idx = _global_option_idx + 1;
}

void DeepSkillChaining::loadGlobalOption(const std::string &actor_path,
                                         const std::string &critic1_path,
                                         const std::string &critic2_path)
{
    _skills.push_back(_makeSkill(true, nullptr)); // TODO: replace this code for making the skill
    torch::load(_skills[0]->agent().actor_local, actor_path);
    torch::load(_skills[0]->agent().critic_local_1, critic1_path);
    torch::load(_skills[0]->agent().critic_local_2, critic2_path);
    _poo.addOption(0.0f);
    _poo.hardCopy();
    std::cout << "=== Loaded global option oG from " << actor_path << " ===" << std::endl;
}

void DeepSkillChaining::_warmupRollout()
{
    int step = 0;
    bool env_done = false;

    while (step < _cfg.steps_per_episode && !env_done)
    {
        auto [steps_taken, cum_reward, done, first_state_poo, last_state_poo] = _skills[_global_option_idx]->rollout(_global_goal); // when warming up, just use global goal
        env_done = done;
        step += steps_taken;

        _poo.addExperience(first_state_poo, _global_option_idx, cum_reward, last_state_poo, done, steps_taken);
        _poo.learn();
    }
}

std::pair<int, AbstractedState> DeepSkillChaining::_pickOption()
{
    _env->setGoal(_global_goal.position, _global_goal.orientation, _global_goal.velocity, _global_goal.angular_velocity); // need to set global goal so that the state calculation is correct

    auto state = _env->getState();
    auto global_state = _env->getAbstractedState();

    torch::Tensor q_vals = _poo.getOptions(state);

    int best_option = _global_option_idx;
    float best_option_q = -1e5; // TODO: better way to set this
    for (int o = _global_option_idx + 1; o < _skills.size(); o++)
    {
        if (_skills[o]->canStart(global_state))
        {
            float current_q = q_vals[o].item<float>();
            if (current_q > best_option_q)
            {
                best_option_q = current_q;
                best_option = o;
            }
        }
    }

    if (best_option == _global_option_idx)
    {
        // TODO: pick closest option as goal
        float min_dist = 1e5;
        int closest_option = -1;
        for (int o = _global_option_idx + 1; o < _unfinished_option_idx; o++)
        {
            float dist = _skills[o]->distanceToState(global_state);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_option = o;
            }
        }
        if (closest_option != -1)
        {
            return {_global_option_idx, _skills[closest_option]->sampleSubgoalState()};
        }
        else
        {
            return {_global_option_idx, _global_goal};
        }
    }
    else
    {
        return {best_option, _skills[best_option]->getLocalGoal()};
    }
}

void DeepSkillChaining::_dscRollout()
{
    int step = 0;
    bool env_done = false;
    while (step < _cfg.steps_per_episode && !env_done)
    {
        auto [option, goal] = _pickOption();
        auto [steps_taken, cum_reward, done, first_state_poo, last_state_poo] = _skills[option]->rollout(goal);

        env_done = done;
        step += steps_taken;

        _poo.addExperience(first_state_poo, option, cum_reward, last_state_poo, done, steps_taken);
        _poo.learn();

        // make a new skill if we hae finished training the current option, but still have not reached the end goal
        if (option == _unfinished_option_idx && _skills[_unfinished_option_idx]->getTrainingPhase() != "gestation" && !_containsGlobalStartState())
        {
            // make new skill
            _skills.push_back(_makeSkill(false, _skills.back()));
            _poo.addOption(0.0f);
            _poo.hardCopy();
            _unfinished_option_idx++;
            std::cout << "Making New Skill With ID: " << _unfinished_option_idx << std::endl;
        }
    }
}

bool DeepSkillChaining::_containsGlobalStartState()
{
    for (int o = _global_option_idx + 1; o < _unfinished_option_idx; o++)
    {
        if (_skills[o]->getTrainingPhase() != "gestation" && _skills[o]->canStart(_global_start))
            return true;
    }
    return false;
}

std::shared_ptr<Skill> DeepSkillChaining::_makeSkill(bool is_global, std::shared_ptr<Skill> parent)
{
    std::shared_ptr<Skill> new_skill = std::make_shared<Skill>(_env,
                                                               _cfg.actor_layers, _cfg.critic_layers, _device,
                                                               _cfg.lr_actor, _cfg.lr_critic, _cfg.tau, _cfg.gamma, _cfg.max_obstacles, _cfg.actor_warmup_steps,
                                                               _cfg.batch_size, _cfg.actor_update_freq, _cfg.last_k,
                                                               _cfg.max_option_steps, parent, _cfg.gestation_n, is_global, _global_goal);
    if (!is_global)
        new_skill->initFromSkill(_skills[_global_option_idx]);
    return new_skill;
}

int DeepSkillChaining::train(int max_episodes) // max_episodes is the timeout where this will declare no success
{
    for (int episode = 0; episode < max_episodes; episode++)
    {
        // TODO: maybe add some goal biased sampling here
        auto [pos, quat, vel, ang_vel] = _robot_bridge->generateRandomPoseWithVel();
        _env->resetTo(pos, quat, vel, ang_vel); // could just do this with env->reset, but want to make a bit more explicit.

        if (episode < _cfg.warmup_episodes)
        {
            _warmupRollout();
        }
        else
        {
            _dscRollout();
        }
        if (_containsGlobalStartState())
        {
            std::cout << "Success!" << std::endl;
            break;
        }
    }
    return _skills.size();
}

// void DeepSkillChaining::save(const std::string &dir) const
// {
//     std::filesystem::create_directories(dir);
//     for (size_t i = 0; i < _skills.size(); ++i)
//     {
//         std::string prefix = dir + "/skill_" + std::to_string(i);
//         _skills[i]->save(prefix + "_actor.pt",
//                          prefix + "_critic1.pt",
//                          prefix + "_critic2.pt",
//                          prefix + "_classifier.svm");
//     }
//     std::cout << "Saved " << _skills.size() << " skills to " << dir << std::endl;
// }

// void DeepSkillChaining::load(const std::string &dir, int num_skills)
// {
//     _skills.clear();
//     for (int i = 0; i < num_skills; ++i)
//     {
//         _skills.emplace_back(_makeSkill());
//         std::string prefix = dir + "/skill_" + std::to_string(i);
//         _skills[i]->load(prefix + "_actor.pt",
//                          prefix + "_critic1.pt",
//                          prefix + "_critic2.pt",
//                          prefix + "_classifier.svm");
//         _poo.addOption(0.0f);
//     }
//     _poo.hardCopy();
//     std::cout << "Loaded " << num_skills << " skills from " << dir << std::endl;
// }

int DeepSkillChaining::numSkills() const
{
    return (int)_skills.size();
}

/************************************** main **************************************/

#define SCENE_FILE "../config/scene/test_scene_obs_free.xml"
#define OG_ACTOR "best_actor.pt"
#define OG_CRITIC1 "best_critic_1.pt"
#define OG_CRITIC2 "best_critic_2.pt"

#define X_MIN -5.0f
#define X_MAX 5.0f
#define Y_MIN -5.0f
#define Y_MAX 5.0f

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

    DeepSkillChaining::Config cfg;
    cfg.gestation_n = 10;
    cfg.last_k = 10;
    cfg.refinement_eps = 20;
    cfg.nu = 0.1;
    cfg.max_skills = 6;

    AbstractedState global_goal = {{3, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    AbstractedState global_start = {{-1, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    DeepSkillChaining dsc(robot_bridge, device, global_goal, global_start, "../models/best_actor.pt", "../models/best_critic_1.pt", "../models/best_critic_2.pt", cfg);

    int n = dsc.train(20);
    std::cout << "\nTraining complete: " << n << " skill(s) in chain." << std::endl;

    // dsc.save("dsc_models");

    // std::cout << "\n=== Evaluation (20 episodes) ===" << std::endl;
    // float total = 0.0f;
    // for (int i = 0; i < 20; ++i)
    // {
    //     float r = dsc.execute();
    //     total += r;
    //     std::cout << "  Episode " << i + 1 << ": reward = " << r << std::endl;
    // }
    // std::cout << "Mean reward: " << (total / 20.0f) << std::endl;

    return 0;
}
