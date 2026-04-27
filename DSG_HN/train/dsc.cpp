#include "dsc.h"
#include <fstream>
#include <stdexcept>
#include <cstdlib>

DeepSkillChaining::DeepSkillChaining(
    std::shared_ptr<RobotBridgeTrain> robot_bridge,
    torch::Device device,
    AbstractedState global_goal,
    AbstractedState global_start,
    const std::string &pretrain_actor_path,
    const std::string &pretrain_critic1_path,
    const std::string &pretrain_critic2_path,
    const std::string &scene_file,
    Config cfg, bool eval, bool make_goal_option)
    : _robot_bridge(robot_bridge), _device(device), _global_goal(global_goal), _global_start(global_start), _scene_file_path(scene_file), _cfg(std::move(cfg)), _eval(eval),
      _env(std::make_shared<TrainEnvironment>(_robot_bridge, cfg.steps_per_episode, cfg.narrow_map)),
      _poo(_env, _cfg.poo_layers, device, _cfg.lr_poo, _cfg.tau, _cfg.gamma, _cfg.poo_batch_size), _rng(std::random_device{}())
{
    _env->setSuccessRadius(_cfg.success_radius);
    _loadGlobalOption(pretrain_actor_path, pretrain_critic1_path, pretrain_critic2_path);
    _unfinished_option_idx = _global_option_idx; // assigning global option index to unfinished option index, as this is the index of the next option we will train, and we have only trained the global option at this point

    if (make_goal_option)
        _makeSkill(false, nullptr); // this is the goal option getting pushed
}

DeepSkillChaining::DeepSkillChaining(
    std::shared_ptr<RobotBridgeTrain> robot_bridge,
    torch::Device device,
    AbstractedState global_goal,
    AbstractedState global_start,
    const std::string &scene_file,
    Config cfg, bool eval, bool make_goal_option)
    : _robot_bridge(robot_bridge), _device(device), _global_goal(global_goal), _global_start(global_start), _scene_file_path(scene_file), _cfg(std::move(cfg)), _eval(eval),
      _env(std::make_shared<TrainEnvironment>(_robot_bridge, cfg.steps_per_episode)),
      _poo(_env, _cfg.poo_layers, device, _cfg.lr_poo, _cfg.tau, _cfg.gamma, _cfg.batch_size), _rng(std::random_device{}())
{
    _env->setSuccessRadius(_cfg.success_radius);
    _makeSkill(true, nullptr);

    _unfinished_option_idx = _global_option_idx; // assigning global option index to unfinished option index, as this is the index of the next option we will train, and we have only trained the global option at this point

    if (make_goal_option)
        _makeSkill(false, nullptr); // this is the goal option getting pushed
}

int DeepSkillChaining::numSkills() const
{
    return (int)_skills.size();
}

int DeepSkillChaining::train(int max_episodes) // max_episodes is the timeout where this will declare no success
{
    if (_cfg.render_training)
        _robot_bridge->startRender();

    for (int episode = 0; episode < max_episodes; episode++)
    {
        _env->clearGoal();
        // TODO: more intelligent sampling. The big thing is not just randomly sampling, but perhaps sampling in the
        // area of the previously learned skill so that it has a bit of an easier time learning
        std::uniform_real_distribution<float> dis(0.0f, 1.0f);
        float p = dis(_rng);
        if (_cfg.strict_sampling) // bit of goal biased sampling, may want to do even more intelligent sampling going forward
        {
            _env->resetTo(_sampleSpawnState());
        }
        else
        {
            std::uniform_real_distribution<float> dis(0.0f, 1.0f);
            float p = dis(_rng);
            if (p < _cfg.sample_global_start_percentage) // bit of goal biased sampling, may want to do even more intelligent sampling going forward
            {
                _env->resetTo(_global_start);
            }
            else if (p >= _cfg.sample_global_start_percentage && p < _cfg.sample_global_start_percentage + _cfg.sample_start_near_boundary_percentage)
            {
                auto start = _sampleStartNearBoundary();
                _env->resetTo(start);
            }
            else
            {
                _env->reset(); // random start
            }
        }

        float ep_reward = 0.0f;
        if (episode < _cfg.warmup_episodes)
        {
            _warmupRollout();
        }
        else
        {
            ep_reward = _dscRollout();
            _validateOption(); // if necessary, will validate the options
        }

        if (_cfg.log_interval > 0 && (episode + 1) % _cfg.log_interval == 0)
        {
            std::cout << "\n[Episode " << (episode + 1) << "] reward=" << ep_reward << "\n";
            std::cout << "=== Skill Status ===\n";
            std::cout << "  ID      Phase       GoalHits\n";
            for (size_t i = 0; i < _skills.size(); ++i)
            {
                std::string label;
                if (i == (size_t)_global_option_idx)
                    label = "global";
                else if (i == (size_t)_global_option_idx + 1)
                    label = "goal";
                else
                    label = "opt-" + std::to_string(i);

                std::string phase = _skills[i]->getTrainingPhase();
                if (i == (size_t)_global_option_idx)
                    phase = "pre-trained";

                std::cout << "  " << label << "   " << phase;
                if (i == (size_t)_global_option_idx)
                    std::cout << "\n";
                else
                    std::cout << "   " << _skills[i]->goalHits() << "/" << _skills[i]->gestationPeriod() << "\n";
            }
            if (_cfg.visualize_initiation_sets)
                visualizeInitiationSets();
        }

        if (_containsGlobalStartState())
        {
            std::cout << "Success!" << std::endl;
            break;
        }
        if (_skills.size() - 1 >= _cfg.max_skills)
        {
            std::cout << "Reached max skill limit. Ending training." << std::endl;
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

void DeepSkillChaining::visualizeInitiationSets()
{
    std::vector<std::vector<std::array<float, 3>>> points_per_skill;
    int max_points = 1000;
    for (const auto &skill : _skills)
    {
        std::vector<std::array<float, 3>> points;
        int point = 0;
        for (const auto &record : skill->getPositiveGestationRecords())
        {
            if (point >= max_points)
                break;
            point++;
            points.push_back(record.state.position);
        }
        points_per_skill.push_back(points);
    }

    std::string temp_file = "/tmp/init_sets.txt";
    std::ofstream out(temp_file);
    int skill_idx = 0;
    for (const auto &points : points_per_skill)
    {
        for (const auto &p : points)
        {
            out << skill_idx << " " << p[0] << " " << p[1] << "\n";
        }
        skill_idx++;
    }
    out.close();

    std::string surface_file = "/tmp/init_surfaces.txt";
    std::ofstream out_surf(surface_file);
    int resolution = 80; // 80x80 grid

    for (int i = 0; i < resolution; ++i)
    {
        for (int j = 0; j < resolution; ++j)
        {
            float x = _robot_bridge->x_min + (float)i * (_robot_bridge->x_max - _robot_bridge->x_min) / (resolution - 1);
            float y = _robot_bridge->y_min + (float)j * (_robot_bridge->y_max - _robot_bridge->y_min) / (resolution - 1);

            AbstractedState dummy_state;
            dummy_state.position = {x, y, 0.0f};

            out_surf << x << " " << y;
            // Append every skill's scores to the same line
            for (const auto &skill : _skills)
            {
                auto [opt, pess] = skill->getDecisionValues(dummy_state);
                out_surf << " " << opt << " " << pess;
            }
            out_surf << "\n";
        }
    }
    out_surf.close();

    std::string cmd = "python3 ../visualize_classifier_surface.py " + _scene_file_path + " " + temp_file + " " + surface_file;
    std::cout << "[DSC] Visualizing decision surfaces..." << std::endl;
    system(cmd.c_str());
}

void DeepSkillChaining::save(const std::string &dir) const
{
    std::filesystem::create_directories(dir);
    for (size_t i = 0; i < _skills.size(); ++i)
    {
        try
        {
            std::string prefix = dir + "/skill_" + std::to_string(i);
            _skills[i]->save(prefix + "_actor.pt",
                             prefix + "_critic1.pt",
                             prefix + "_critic2.pt",
                             prefix + "_classifier.svm");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    std::ofstream meta(dir + "/skill_count.txt");
    meta << _skills.size();
    std::ofstream scene_meta(dir + "/scene_file.txt");
    scene_meta << std::filesystem::path(_scene_file_path).filename().string();
    torch::save(_poo.q, dir + "/poo_q.pt");
    torch::save(_poo.target_q, dir + "/poo_target_q.pt");
    std::cout << "Saved " << _skills.size() << " skills and policy-over-options to " << dir << std::endl;
}

void DeepSkillChaining::load(const std::string &dir, const std::string &scene_file)
{
    std::string current_scene_file = std::filesystem::path(scene_file).filename().string();

    std::ifstream scene_meta(dir + "/scene_file.txt");
    if (!scene_meta.is_open())
        throw std::runtime_error("Missing scene_file.txt in " + dir);
    std::string saved_scene_file;
    scene_meta >> saved_scene_file;

    if (saved_scene_file != current_scene_file)
        throw std::runtime_error("Scene file mismatch: saved with " + saved_scene_file + ", loading with " + current_scene_file);

    std::ifstream meta(dir + "/skill_count.txt");
    if (!meta.is_open())
        throw std::runtime_error("Missing skill_count.txt in " + dir);
    int num_skills;
    meta >> num_skills;

    _skills.clear();
    _poo = PolicyOverOptionsAgent(_env, _cfg.poo_layers, _device, _cfg.lr_poo, _cfg.tau, _cfg.gamma, _cfg.batch_size);
    _unfinished_option_idx = _global_option_idx;

    _skills.clear(); // remove existing skills
    for (int i = 0; i < num_skills; ++i)
    {
        if (i == 0)
            _makeSkill(true, nullptr);
        else if (i == 1)
            _makeSkill(false, nullptr);
        else
            _makeSkill(false, _skills.back());

        std::string prefix = dir + "/skill_" + std::to_string(i);
        _skills[i]->load(prefix + "_actor.pt",
                         prefix + "_critic1.pt",
                         prefix + "_critic2.pt",
                         prefix + "_classifier.svm");
        _skills[i]->agent().hardCopy();
        _skills[i]->agent().toDevice(_device);
    }

    if (std::filesystem::exists(dir + "/poo_q.pt"))
        torch::load(_poo.q, dir + "/poo_q.pt");
    if (std::filesystem::exists(dir + "/poo_target_q.pt"))
        torch::load(_poo.target_q, dir + "/poo_target_q.pt");
    std::cout << "Loaded " << num_skills << " skills and policy-over-options from " << dir << std::endl;
}

void DeepSkillChaining::setEvalMode(bool eval)
{
    _eval = eval;
    for (auto &skill : _skills)
    {
        skill->setEvalMode(eval);
    }
}

/*** Private ***/
void DeepSkillChaining::_validateOption()
{

    int validate_idx = -1;
    for (int o = _global_option_idx + 1; o < _skills.size(); o++)
    {
        if (_skills[o]->getTrainingPhase() == "validation")
        {
            validate_idx = o;
            break;
        }
    }

    if (validate_idx == -1)
        return;

    int num_successes = 0;
    for (int i = 0; i < _cfg.gestation_n; i++)
    {
        auto start = _skills[validate_idx]->sampleSubgoalState();
        auto goal = _skills[validate_idx]->getLocalGoal();
        _env->resetTo(start);
        auto [_, __, ___, ____, _____] = _skills[validate_idx]->rollout(goal);

        auto [reward, done] = _env->computeReward(goal);

        if (_skills[validate_idx]->atTermination(goal))
            num_successes++;
    }

    if ((float)num_successes / (float)_cfg.gestation_n > _cfg.val_accuracy_threshold)
    {
        std::cout << "Option " << validate_idx << " validated. Success Rate: " << (float)num_successes / (float)_cfg.gestation_n << std::endl;
        _skills[validate_idx]->validateSkill(true);
    }
    else
    {
        std::cout << "Option " << validate_idx << " failed validation. Success Rate: " << (float)num_successes / (float)_cfg.gestation_n << std::endl;
        _skills[validate_idx]->validateSkill(false);
    }
}

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
    float best_q_val = std::numeric_limits<float>::lowest();

    // Collect valid options split by pessimistic availability
    std::vector<int> pessimistic_options;
    std::vector<int> optimistic_options;

    for (int o = _global_option_idx + 1; o < _skills.size(); o++)
    {
        if (_skills[o]->canStart(global_state) && !_skills[o]->atTermination(_global_goal))
        {
            if (_skills[o]->canStartPessimistic(global_state))
            {
                pessimistic_options.push_back(o);
            }
            else
            {
                optimistic_options.push_back(o);
            }
        }
    }

    // Pick best from pessimistic, fallback to optimistic
    const auto &candidates = pessimistic_options.empty() ? optimistic_options : pessimistic_options;

    // empirically found that just picking the earliest valid option is more consistent
    // than picking based on Q values
    best_option = candidates.empty() ? _global_option_idx : candidates[0];

    if (best_option == _global_option_idx)
    {
        // TODO: pick closest option as goal
        float min_dist = std::numeric_limits<float>::infinity();
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

    auto eng = _robot_bridge->getEngine();
    if (eng->render_m)
    {
        eng->setGoalMarker(_global_goal.position[0], _global_goal.position[1], 0.5f);
    }

    while (step < _cfg.steps_per_episode && !env_done)
    {
        auto [option, goal] = _pickOption();

        eng->setGoalMarker(goal.position[0], goal.position[1], 0.5f);

        auto [steps_taken, cum_reward, local_done, first_state_poo, last_state_poo] = _skills[option]->rollout(goal);

        auto [g_reward, g_done] = _env->computeReward(_global_goal);
        env_done = g_done.data_ptr<float>()[0] > 0.5f;

        if (steps_taken == 0) // this condition occurs when we just finished training a new skill, but then find ourselves in the initiation set of that skill while trying to train the new skill
            break;

        step += steps_taken;
        total_reward += cum_reward;

        // float clipped_reward = std::clamp(cum_reward, -100.0f, 100.0f); // clipping reward since sometimes not terminating makes the reward spike and want to limit that effect.
        if (!_eval) // only train poo during training, not evaluation
        {
            _poo.addExperience(first_state_poo, option, cum_reward, last_state_poo, env_done, steps_taken);
            _poo.learn();
        }

        if (_cfg.verbose)
            std::cout << "  [Rollout] option=" << option
                      << " phase=" << _skills[option]->getTrainingPhase()
                      << " steps=" << steps_taken
                      << " reward=" << cum_reward
                      << " local_done=" << local_done
                      << " global_done=" << env_done << "\n";

        // make a new skill if we have finished training the current option, but still have not reached the end goal
        if (_shouldCreateNewOption() && !_eval)
        {

            float total_dist = 0;
            for (int i = 0; i < _cfg.gestation_n; i++)
            {
                AbstractedState sample = _skills[_unfinished_option_idx]->sampleSubgoalState();
                float dx = sample.position[0] - _global_start.position[0];
                float dy = sample.position[1] - _global_start.position[1];
                float dz = sample.position[2] - _global_start.position[2];
                total_dist += std::sqrt(dx * dx + dy * dy + dz * dz);
            }
            std::cout << "\n[Option " << _unfinished_option_idx << " matured] Average distance of initiation region to start: " << (total_dist / _cfg.gestation_n) << " m\n";
            _makeSkill(false, _skills.back());
        }
    }
    return total_reward;
}

bool DeepSkillChaining::_shouldCreateNewOption()
{
    bool start_in_init = false;
    for (int o = _global_option_idx + 1; o < _skills.size(); o++)
    {
        if (_skills[o]->getTrainingPhase() != "mature" && !_cfg.strict_sampling) // disable when using strict sampling since if an earlier option can start, no later option will ever pass gestation
            return false;
        if (_skills[o]->canStart(_global_start))
            return false;
    }
    return true;
}

bool DeepSkillChaining::_containsGlobalStartState()
{
    bool start_in_init = false;
    for (int o = _global_option_idx + 1; o < _skills.size(); o++)
    {
        if (_skills[o]->getTrainingPhase() != "mature")
            return false;
        if (_skills[o]->canStart(_global_start) && _skills[o]->getTrainingPhase() == "mature")
            start_in_init = true;
    }
    return start_in_init;
}

void DeepSkillChaining::_makeSkill(bool is_global, std::shared_ptr<Skill> parent, const AbstractedState &local_goal)
{
    int id = (is_global) ? _global_option_idx : _unfinished_option_idx + 1;
    float lr_actor = (is_global) ? _cfg.lr_actor_global : _cfg.lr_actor;
    float lr_critic = (is_global) ? _cfg.lr_critic_global : _cfg.lr_critic;
    std::shared_ptr<Skill> global_option = (is_global) ? nullptr : _skills[_global_option_idx];
    std::shared_ptr<Skill> new_skill = std::make_shared<Skill>(id, _env,
                                                               _cfg.actor_layers, _cfg.critic_layers, _device,
                                                               lr_actor, lr_critic, _cfg.tau, _cfg.gamma, _cfg.actor_warmup_steps,
                                                               _cfg.batch_size, _cfg.actor_update_freq, _cfg.last_k,
                                                               _cfg.max_option_steps, _cfg.nu, _cfg.optimistic_svc_c, _cfg.optimistic_svc_gamma,
                                                               _cfg.optimistic_svc_balance_classes,
                                                               _cfg.optimistic_svc_positive_margin_tolerance,
                                                               _cfg.pessimistic_ocsvm_gamma, _cfg.optimistic_ocsvm_gamma, _cfg.optimistic_ocsvm_nu_divisor,
                                                               _cfg.subgoal_robustness_tolerance, _cfg.negative_samples_per_failure,
                                                               parent, _cfg.gestation_n, is_global, local_goal, global_option, _eval,
                                                               _cfg.exploration_noise_gestation, _cfg.exploration_noise_mature,
                                                               _cfg.use_human_collected_data, _cfg.human_collected_data_path, _cfg.human_data_percentage, _cfg.updates_per_step);
    if (!is_global)
        new_skill->initFromSkill(_skills[_global_option_idx]);

    new_skill->agent().hardCopy();
    new_skill->agent().toDevice(_device);
    _unfinished_option_idx = id;

    if (parent)
        std::cout << "\nMaking New Skill With ID: " << id << " (parent=" << parent->goalHits() << "/" << parent->gestationPeriod() << " id=" << id - 1 << ")\n";
    else
        std::cout << "\nMaking New Skill With ID: " << id << "\n";
    _skills.push_back(new_skill);

    _poo.addOption(0.0f); // global option also needs to be added to poo
    _poo.hardCopy();
}

void DeepSkillChaining::_makeSkill(bool is_global, std::shared_ptr<Skill> parent)
{
    int id = (is_global) ? _global_option_idx : _unfinished_option_idx + 1;
    float lr_actor = (is_global) ? _cfg.lr_actor_global : _cfg.lr_actor;
    float lr_critic = (is_global) ? _cfg.lr_critic_global : _cfg.lr_critic;
    std::shared_ptr<Skill> global_option = (is_global) ? nullptr : _skills[_global_option_idx];
        std::shared_ptr<Skill> new_skill = std::make_shared<Skill>(id, _env,
                                                               _cfg.actor_layers, _cfg.critic_layers, _device,
                                                               lr_actor, lr_critic, _cfg.tau, _cfg.gamma, _cfg.actor_warmup_steps,
                                                               _cfg.batch_size, _cfg.actor_update_freq, _cfg.last_k,
                                                               _cfg.max_option_steps, _cfg.nu, _cfg.optimistic_svc_c, _cfg.optimistic_svc_gamma,
                                                               _cfg.optimistic_svc_balance_classes,
                                                               _cfg.optimistic_svc_positive_margin_tolerance,
                                                               _cfg.pessimistic_ocsvm_gamma, _cfg.optimistic_ocsvm_gamma, _cfg.optimistic_ocsvm_nu_divisor,
                                                               _cfg.subgoal_robustness_tolerance, _cfg.negative_samples_per_failure,
                                                               parent, _cfg.gestation_n, is_global, _global_goal, global_option, _eval,
                                                               _cfg.exploration_noise_gestation, _cfg.exploration_noise_mature,
                                                               _cfg.use_human_collected_data, _cfg.human_collected_data_path, _cfg.human_data_percentage, _cfg.updates_per_step);
    if (!is_global)
        new_skill->initFromSkill(_skills[_global_option_idx]);

    new_skill->agent().hardCopy();
    new_skill->agent().toDevice(_device);
    _unfinished_option_idx = id;
    if (parent)
        std::cout << "\nMaking New Skill With ID: " << id << " (parent=" << parent->goalHits() << "/" << parent->gestationPeriod() << " id=" << id - 1 << ")\n";
    else
        std::cout << "\nMaking New Skill With ID: " << id << "\n";

    _skills.push_back(new_skill);
    _poo.addOption(0.0f); // Start with optimistic value so it is used
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
    _skills[0]->agent().hardCopy();
    _skills[0]->agent().toDevice(_device);
    std::cout << "=== Loaded global option oG from " << actor_path << " ===" << std::endl;
}

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
        v1[0] = 0;
        v1[1] = 0;
        v1[2] = 0;
        w1[0] = 0;
        w1[1] = 0;
        w1[2] = 0;

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

    std::cerr << "Warning: Could not generate valid gaussian sample, returning random sample" << std::endl;
    return _env->getRandomValidAbstractedState();
}

// perhaps: instead of sampling interpolating between goal and start, interpolate between start and random part of inititation set of last mature option
AbstractedState DeepSkillChaining::_sampleStartInterpolated()
{
    float pos_std = 1.0f; // tune this based on how much spread you want
    for (int attempts = 0; attempts < 300; attempts++)
    {
        AbstractedState local_goal = _skills[_unfinished_option_idx]->getLocalGoal();

        std::uniform_real_distribution<float> dist1(std::min(local_goal.position[0], _global_start.position[0]), std::max(local_goal.position[0], _global_start.position[0]));
        std::uniform_real_distribution<float> dist2(std::min(local_goal.position[1], _global_start.position[1]), std::max(local_goal.position[1], _global_start.position[1]));

        std::array<float, 3> pos;
        pos[0] = dist1(_rng) + _sampleGaussianDist(0.0f, pos_std);
        pos[1] = dist2(_rng) + _sampleGaussianDist(0.0f, pos_std);
        pos[2] = _global_start.position[2];

        auto [_, quat, vel, ang_vel] = _robot_bridge->generateRandomPoseWithVel();
        vel[0] = 0;
        vel[1] = 0;
        vel[2] = 0;
        ang_vel[0] = 0;
        ang_vel[1] = 0;
        ang_vel[2] = 0;
        ang_vel[3] = 0;
        if (_robot_bridge->isConfigurationValid(pos, quat, vel, ang_vel))
            return {pos, quat, vel, ang_vel};
    }
    std::cerr << "Warning: Could not generate valid linearly interpolated sample, returning random sample" << std::endl;
    return _env->getRandomValidAbstractedState();
}

AbstractedState DeepSkillChaining::_sampleStartNearBoundary()
{
    float std_pos = 1.5f;
    float std_rot = 0.2f;
    float std_vel = 0.1;
    float std_ang_vel = 0.1;

    for (int attempts = 0; attempts < 300; attempts++)
    {
        AbstractedState state = _skills[_unfinished_option_idx]->getLocalGoal();
        state.position[0] += _sampleGaussianDist(0.0f, std_pos);
        state.position[1] += _sampleGaussianDist(0.0f, std_pos);
        state.position[2] += _sampleGaussianDist(0.0f, std_pos);

        state.velocity[0] = 0;
        state.velocity[1] = 0;
        state.velocity[2] = 0;
        // state.velocity[0] += _sampleGaussianDist(0.0f, std_vel);
        // state.velocity[1] += _sampleGaussianDist(0.0f, std_vel);
        // state.velocity[2] += _sampleGaussianDist(0.0f, std_vel);

        state.angular_velocity[0] = 0; //+= _sampleGaussianDist(0.0f, std_ang_vel);
        state.angular_velocity[1] = 0; //+= _sampleGaussianDist(0.0f, std_ang_vel);
        state.angular_velocity[2] = 0; //+= _sampleGaussianDist(0.0f, std_ang_vel);

        state.orientation = _getGaussianQuaternionPerturbation(state.orientation, std_rot);

        bool any_predecessor_can_start = false;
        for (int o = _global_option_idx + 1; o < _unfinished_option_idx; o++)
        {
            if (_skills[o]->canStart(state) || _skills[o]->canStartPessimistic(state))
            {
                any_predecessor_can_start = true;
                break;
            }
        }

        if (_robot_bridge->isConfigurationValid(state.position, state.orientation, state.velocity, state.angular_velocity) &&
            !any_predecessor_can_start)
        {
            return state;
        }
    }
    std::cerr << "Warning: Could not generate sample near boundary of previous option, returning random sample" << std::endl;

    return _env->getRandomValidAbstractedState();
}

AbstractedState DeepSkillChaining::_sampleSpawnState()
{
    return _global_start;
}

/************************************** main **************************************/

#ifndef DSG_BUILD
#define SCENE_FILE "../config/scene/umaze_scene.xml"
#define OG_ACTOR "../models/pretrain_actor_umaze.pt"
#define OG_CRITIC1 "../models/pretrain_critic_1_umaze.pt"
#define OG_CRITIC2 "../models/pretrain_critic_2_umaze.pt"
#define DSC_SAVE_PATH "../dsc_models_umaze"
#define TEST true // if set to true, will not train, will just load and run testing
#define RENDER_TRAINING false

#define X_MIN -7.0f
#define X_MAX 7.0f
#define Y_MIN -7.0f
#define Y_MAX 7.0f

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
    else if (torch::mps::is_available())
    {
        std::cout << "MPS is available! Training on Apple GPU." << std::endl;
        device = torch::Device(torch::kMPS);
    }

    auto robot_bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, /*render=*/RENDER_TRAINING);

    DeepSkillChaining::Config cfg;
    // configs for narrow map
    // cfg.gestation_n = 120; // number of total successes that should be collected during gestation phase. Perhaps use a percentage for the option being currently learnt?
    // cfg.last_k = 50;
    // cfg.max_option_steps = 100; // each option should be meaningful enough. 5Hz and 20 steps means each option can run for up to 4 seconds
    // cfg.narrow_map = true;
    // cfg.strict_sampling = true;
    // cfg.human_collected_data_path = "../../sandbox/transitions_narrow_point_to_point.csv";
    // cfg.use_human_collected_data = true;
    // cfg.human_data_percentage = 0.2;
    // cfg.exploration_noise_gestation = 0.15;
    // cfg.exploration_noise_mature = 0.1;
    // cfg.nu = 0.01;
    // cfg.optimistic_svc_c = 0.1;
    // cfg.optimistic_svc_gamma = 0.5;

    // configs for umaze
    cfg.gestation_n = 120;
    cfg.last_k = 15;
    cfg.max_option_steps = 30;
    cfg.narrow_map = false;
    cfg.strict_sampling = false;
    cfg.exploration_noise_gestation = 0.3;
    cfg.exploration_noise_mature = 0.1;
    cfg.nu = 0.05;
    cfg.optimistic_svc_c = 1.0;
    cfg.optimistic_svc_gamma = 0.5;
    cfg.sample_global_start_percentage = 0.1;
    cfg.sample_start_near_boundary_percentage = 0.6;

    // configs for test scene
    // cfg.gestation_n = 60;
    // cfg.last_k = 30;
    // cfg.max_option_steps = 50;
    // cfg.narrow_map = false;
    // cfg.strict_sampling = false;
    // cfg.exploration_noise_gestation = 0.3;
    // cfg.exploration_noise_mature = 0.1;
    // cfg.nu = 0.05;
    // cfg.optimistic_svc_c = 1.0;
    // cfg.optimistic_svc_gamma = 0.5;
    // cfg.sample_global_start_percentage = 0.6;
    // cfg.sample_start_near_boundary_percentage = 0.1;

    // shared params
    cfg.actor_warmup_steps = 0; // gonna keep at zero for testing purposes as well
    cfg.warmup_episodes = 0;    // keep at zero since I am assuming we have done pretraining
    cfg.verbose = true;         // set to true for per-rollout console output
    cfg.log_interval = 10;      // print skill status table every N episodes
    cfg.visualize_initiation_sets = false;
    cfg.max_skills = 50;
    cfg.val_accuracy_threshold = 0.0f;
    cfg.success_radius = 0.5f;
    cfg.updates_per_step = 2;
    cfg.subgoal_robustness_tolerance = 0.25;
    cfg.negative_samples_per_failure = 1;
    cfg.pessimistic_ocsvm_gamma = 0.5;
    cfg.optimistic_svc_balance_classes = false;

    // goal for UMaze
    AbstractedState global_goal = {{-4.5, 4.1, 0.}, {0, 0, 0, -1}, {0, 0, 0}, {0, 0, 0}};
    AbstractedState global_start = {{-5.3, -4.5, 0.}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    // goal for narrow map
    // AbstractedState global_goal = {{2.0, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    // AbstractedState global_start = {{-2.0, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    // AbstractedState global_start = {{4.0, 4.0, 0.}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    // AbstractedState global_goal = {{0.0, 0.0, 0.}, {0, 0, 0, -1}, {0, 0, 0}, {0, 0, 0}};

    std::string pretrained_actor_path = OG_ACTOR;
    std::string pretrained_critic_1_path = OG_CRITIC1;
    std::string pretrained_critic_2_path = OG_CRITIC2;
    std::string scene_file = SCENE_FILE;
    DeepSkillChaining dsc(robot_bridge, device, global_goal, global_start, pretrained_actor_path, pretrained_critic_1_path, pretrained_critic_2_path, scene_file, cfg);

    if (!TEST)
    {
        int n = dsc.train(40000);
        std::cout << "\nTraining complete: " << n << " skill(s) in chain." << std::endl;
        dsc.save(DSC_SAVE_PATH);
        dsc.visualizeInitiationSets();
    }
    else
    {
        dsc.load(DSC_SAVE_PATH, SCENE_FILE);
        // dsc.visualizeInitiationSets();
    }

    dsc.setEvalMode(true);
    // std::cout << "\n=== Evaluation (20 episodes) ===" << std::endl;
    robot_bridge->startRender();
    int success = 0;
    for (int i = 0; i < 50; ++i)
    {
        float total = 0.0f;
        float r = dsc.execute();
        auto state = robot_bridge->getRobotState();
        bool episode_success = false;
        if (std::sqrt((state.position[0] - global_goal.position[0]) * (state.position[0] - global_goal.position[0]) + (state.position[1] - global_goal.position[1]) * (state.position[1] - global_goal.position[1])) < 0.5)
            {
        success++;
                episode_success = true;
            }
        total += r;
        std::cout << "  Episode " << i + 1 << ": reward = " << r << " Success: " << episode_success << std::endl;
    }
    std::cout << "Num Successed: " << success << " / " << 50 << std::endl;

    return 0;
}
#endif // DSG_BUILD
