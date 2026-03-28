#include "dsc.h"

/*
MOST IMPORTANT FIXES:
1) Need ability to turn with low level policy, otherwise having velocity as part of goal is not going to work
2) Figure out clasifier fitting (need to ensure that it is not too pessimistic/optimistic)
3) Improve RL formulation, should be getting pretty good success rate with vel goals (i.e. > 70%)
4) Improve spawning logic
5) Figure out what to do with global option, so that we do not lose information gained when we train local options
*/

// TODO: change vel limit to reflect that it can be assymetrical

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
      _poo(_env, _cfg.poo_layers, device, _cfg.lr_poo, _cfg.tau, _cfg.gamma, _cfg.batch_size), _rng(std::random_device{}())
{
    _loadGlobalOption(pretrain_actor_path, pretrain_critic1_path, pretrain_critic2_path);
    _unfinished_option_idx = _global_option_idx;

    // todo: put new skill here
    _makeSkill(false, nullptr); // this is the goal option getting pushed
}

int DeepSkillChaining::numSkills() const
{
    return (int)_skills.size();
}

int DeepSkillChaining::train(int max_episodes) // max_episodes is the timeout where this will declare no success
{
    for (int episode = 0; episode < max_episodes; episode++)
    {
        // TODO: more intelligent sampling. The big thing is not just randomly sampling, but perhaps sampling in the
        // area of the previously learned skill so that it has a bit of an easier time learning
        std::uniform_real_distribution<float> dis(0.0f, 1.0f);
        float p = dis(_rng);
        if (p < 0.2) // bit of goal biased sampling, may want to do even more intelligent sampling going forward
        {
            _env->resetTo(_global_start);
        }
        else if (p > 0.2 && p < 0.5)
        {
            auto start = _sampleStartNearObstacle();
            _env->resetTo(start);
        }
        else if (p > 0.5 && p < 0.7)
        {
            auto start = _sampleStartInterpolated();
            _env->resetTo(start);
        }
        else
        {
            _env->reset(); // random start
        }

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
    return _skills.size() - 1;
}

float DeepSkillChaining::execute()
{
    _env->resetTo(_global_start);
    return _dscRollout();
}

/*** Private ***/

void DeepSkillChaining::_warmupRollout()
{
    int step = 0;
    bool env_done = false;

    AbstractedState goal = _env->getRandomValidAbstractedState();

    while (step < _cfg.steps_per_episode && !env_done)
    {
        auto [steps_taken, cum_reward, done, first_state_poo, last_state_poo] = _skills[_global_option_idx]->rollout(goal); // when warming up, just use global goal
        env_done = done;
        step += steps_taken;

        _poo.addExperience(first_state_poo, _global_option_idx, cum_reward, last_state_poo, done, steps_taken);
        _poo.learn();
    }
}

// gonna do this the way they do in bagaria et al. code: pick earliest option
std::pair<int, AbstractedState> DeepSkillChaining::_pickOption()
{
    auto poo_state = _env->getStateRelativeToGoal(_global_goal);
    auto global_state = _env->getAbstractedState();

    torch::Tensor q_vals = _poo.getOptions(poo_state);

    int best_option = _global_option_idx;
    for (int o = _global_option_idx + 1; o < _skills.size(); o++)
    {
        if (_skills[o]->canStart(global_state)) // todo: add logic to not use if it is already in the TERMINATION set
        {
            best_option = o;
            break;
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

float DeepSkillChaining::_dscRollout()
{
    int step = 0;
    bool env_done = false;
    float total_reward = 0;
    while (step < _cfg.steps_per_episode && !env_done)
    {
        auto [option, goal] = _pickOption();
        auto [steps_taken, cum_reward, done, first_state_poo, last_state_poo] = _skills[option]->rollout(goal);

        // std::cout << " " << steps_taken << " " << option << " " << done << std::endl;
        if (steps_taken == 0) // this condition occurs when we just finished training a new skill, but then find ourselves in the initiation set of that skill while trying to train the new skill
            break;

        env_done = done;
        step += steps_taken;
        total_reward += cum_reward;

        _poo.addExperience(first_state_poo, option, cum_reward, last_state_poo, done, steps_taken);
        _poo.learn();

        // make a new skill if we have finished training the current option, but still have not reached the end goal
        if (option == _unfinished_option_idx && _skills[_unfinished_option_idx]->getTrainingPhase() != "gestation" && !_containsGlobalStartState())
        {
            _makeSkill(false, _skills.back());
        }
    }
    return total_reward;
}

bool DeepSkillChaining::_containsGlobalStartState()
{
    for (int o = _global_option_idx + 1; o < _skills.size(); o++)
    {
        if (_skills[o]->getTrainingPhase() != "gestation" && _skills[o]->canStart(_global_start))
            return true;
    }
    return false;
}

void DeepSkillChaining::_makeSkill(bool is_global, std::shared_ptr<Skill> parent)
{
    int id = (is_global) ? _global_option_idx : _unfinished_option_idx + 1;
    std::shared_ptr<Skill> new_skill = std::make_shared<Skill>(id, _env,
                                                               _cfg.actor_layers, _cfg.critic_layers, _device,
                                                               _cfg.lr_actor, _cfg.lr_critic, _cfg.tau, _cfg.gamma, _cfg.max_obstacles, _cfg.actor_warmup_steps,
                                                               _cfg.batch_size, _cfg.actor_update_freq, _cfg.last_k,
                                                               _cfg.max_option_steps, _cfg.nu, parent, _cfg.gestation_n, is_global, _global_goal);
    if (!is_global)
        new_skill->initFromSkill(_skills[_global_option_idx]);

    _unfinished_option_idx = id;
    std::cout << "\nMaking New Skill With ID: " << id << std::endl;
    _skills.push_back(new_skill);
    _poo.addOption(0.0f);
    _poo.hardCopy();
}

void DeepSkillChaining::_loadGlobalOption(const std::string &actor_path,
                                          const std::string &critic1_path,
                                          const std::string &critic2_path)
{
    _makeSkill(true, nullptr); // TODO: replace this code for making the skill
    torch::load(_skills[0]->agent().actor_local, actor_path);
    torch::load(_skills[0]->agent().critic_local_1, critic1_path);
    torch::load(_skills[0]->agent().critic_local_2, critic2_path);
    _poo.addOption(0.0f);
    _poo.hardCopy();
    std::cout << "=== Loaded global option oG from " << actor_path << " ===" << std::endl;
}

// sample random
std::array<float, 4> DeepSkillChaining::_getGaussianQuaternionPerturbation(const std::array<float, 4> &q_orig, float sigma_rad)
{
    std::normal_distribution<float> dist(0.0f, sigma_rad);

    float dx = dist(_rng);
    float dy = dist(_rng);
    float dz = dist(_rng);

    // convert to a small-angle quaternion (w, x, y, z)
    // for small angles: cos(theta/2) approx 1, sin(theta/2) approx theta/2
    std::array<float, 4> dq = {1.0f, dx / 2.0f, dy / 2.0f, dz / 2.0f};

    // multiply: q_new = q_orig * dq
    std::array<float, 4> q_new;
    q_new[0] = q_orig[0] * dq[0] - q_orig[1] * dq[1] - q_orig[2] * dq[2] - q_orig[3] * dq[3]; // w
    q_new[1] = q_orig[0] * dq[1] + q_orig[1] * dq[0] + q_orig[2] * dq[3] - q_orig[3] * dq[2]; // x
    q_new[2] = q_orig[0] * dq[2] - q_orig[1] * dq[3] + q_orig[2] * dq[0] + q_orig[3] * dq[1]; // y
    q_new[3] = q_orig[0] * dq[3] + q_orig[1] * dq[2] - q_orig[2] * dq[1] + q_orig[3] * dq[0]; // z

    // normalize to ensure it stays a unit quaternion
    float norm = std::sqrt(q_new[0] * q_new[0] + q_new[1] * q_new[1] + q_new[2] * q_new[2] + q_new[3] * q_new[3]);
    for (int i = 0; i < 4; ++i)
        q_new[i] /= norm;

    return q_new;
}

float DeepSkillChaining::_sampleGaussianDist(float mu, float std)
{
    std::normal_distribution<float> dist(mu, std);
    return dist(_rng);
}

// simple implementation of gaussian sampling
AbstractedState DeepSkillChaining::_sampleStartNearObstacle()
{
    float std_pos = 0.5; // adjust this param based on size/location of obstacles
    float std_rot = 0.1;

    for (int attempts = 0; attempts < 300; attempts++)
    {
        auto [p1, q1, v1, w1] = _robot_bridge->generateRandomPoseWithVel();

        std::array<float, 3> p2;
        p2[0] = p1[0] + _sampleGaussianDist(0.0f, std_pos); // TODO: need to clamp to be in bounds
        p2[1] = p1[1] + _sampleGaussianDist(0.0f, std_pos);
        p2[2] = p1[2];

        std::array<float, 4> q2 = _getGaussianQuaternionPerturbation(q1, std_rot);
        bool valid1 = _robot_bridge->isConfigurationValid(p1, q1, v1, w1);
        bool valid2 = _robot_bridge->isConfigurationValid(p2, q2, v1, w1);

        if (valid1 != valid2)
        {
            if (valid1)
            {
                return {p1, q1, v1, w1};
            }
            else
            {
                return {p2, q2, v1, w1};
            }
        }
    }

    std::cerr << "Warning: Could not generate valid gaussian sample, return random sample" << std::endl;
    return _env->getRandomValidAbstractedState();
}

AbstractedState DeepSkillChaining::_sampleStartInterpolated()
{
    std::uniform_real_distribution<float> dist1(std::min(_global_goal.position[0], _global_start.position[0]), std::max(_global_goal.position[0], _global_start.position[0]));
    std::uniform_real_distribution<float> dist2(std::min(_global_goal.position[1], _global_start.position[1]), std::max(_global_goal.position[1], _global_start.position[1]));

    float pos_std = 1.0f; // tune this based on how much spread you want
    for (int attempts = 0; attempts < 300; attempts++)
    {
        std::array<float, 3> pos;
        pos[0] = dist1(_rng) + _sampleGaussianDist(0.0f, pos_std);
        pos[1] = dist2(_rng) + _sampleGaussianDist(0.0f, pos_std);
        pos[2] = _global_start.position[2];

        auto [_, quat, vel, ang_vel] = _robot_bridge->generateRandomPoseWithVel();
        if (_robot_bridge->isConfigurationValid(pos, quat, vel, ang_vel))
            return {pos, quat, vel, ang_vel};
    }
    std::cerr << "Warning: Could not generate valid linearly interpolated sample, return random sample" << std::endl;
    return _env->getRandomValidAbstractedState();
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

/************************************** main **************************************/

#define SCENE_FILE "../config/scene/test_scene.xml"
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
    cfg.gestation_n = 100;
    cfg.last_k = 10;
    cfg.max_option_steps = 10;
    cfg.refinement_eps = 20;
    cfg.nu = 0.1;
    cfg.max_skills = 6;
    cfg.actor_warmup_steps = 0; // gonna keep at zero for testing purposes as well
    cfg.warmup_episodes = 200;   // keep at zero since I am assuming we have done pretraining

    AbstractedState global_goal = {{3, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    AbstractedState global_start = {{-3, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    DeepSkillChaining dsc(robot_bridge, device, global_goal, global_start, "../models/best_actor.pt", "../models/best_critic_1.pt", "../models/best_critic_2.pt", cfg);

    int n = dsc.train(2000);
    std::cout << "\nTraining complete: " << n << " skill(s) in chain." << std::endl;

    // TODO: implement this stuff to visualize the output and see if things are succesful or not!

    // dsc.save("dsc_models");

    // std::cout << "\n=== Evaluation (20 episodes) ===" << std::endl;
    float total = 0.0f;
    robot_bridge->startRender();
    for (int i = 0; i < 20; ++i)
    {
        float r = dsc.execute();
        total += r;
        std::cout << "  Episode " << i + 1 << ": reward = " << r << std::endl;
    }
    std::cout << "Mean reward: " << (total / 20.0f) << std::endl;

    return 0;
}
