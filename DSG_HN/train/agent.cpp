#include "agent.h"

/****************************************TD3 AGENT****************************************/

TD3Agent::TD3Agent(
    std::shared_ptr<TrainEnvironment> env,
    const std::vector<int> &actor_layer_sizes,
    const std::vector<int> &critic_layer_sizes,
    torch::Device device_, float lr_actor,
    float lr_critic,
    float tau,
    float gamma,
    int batch_size,
    int actor_update_freq,
    int actor_warmup_steps_, 
    bool use_human_buffer_, 
    float human_data_percentage_) : device(device_), actor_warmup_steps(actor_warmup_steps_), use_human_buffer(use_human_buffer_), human_data_percentage(human_data_percentage_),
                                                       actor_local(env->state_dim, env->obstacle_feature_dim, env->action_dim, actor_layer_sizes, device_),
                                                       actor_target(env->state_dim, env->obstacle_feature_dim, env->action_dim, actor_layer_sizes, device_),
                                                       critic_local_1(env->state_dim, env->obstacle_feature_dim, env->action_dim, critic_layer_sizes, device_),
                                                       critic_target_1(env->state_dim, env->obstacle_feature_dim, env->action_dim, critic_layer_sizes, device_),
                                                       critic_local_2(env->state_dim, env->obstacle_feature_dim, env->action_dim, critic_layer_sizes, device_),
                                                       critic_target_2(env->state_dim, env->obstacle_feature_dim, env->action_dim, critic_layer_sizes, device_),
                                                       actor_optimizer(actor_local->parameters(), torch::optim::AdamOptions(lr_actor)),
                                                       critic_optimizer_1(critic_local_1->parameters(), torch::optim::AdamOptions(lr_critic)),
                                                       critic_optimizer_2(critic_local_2->parameters(), torch::optim::AdamOptions(lr_critic)),
                                                       tau(tau), gamma(gamma), batch_size(batch_size), actor_update_freq(actor_update_freq), lr_actor(lr_actor), lr_critic(lr_critic)
{
    action_scaling_factors = torch::tensor(env->action_scaling_factors);
    action_shift_factors = torch::tensor(env->action_shift_factors);
    toDevice(device);
    hardCopy();
    ou_noise = std::make_unique<OUNoise>(env->action_dim, 0.0f, 0.15f, exploration_noise);
}

void TD3Agent::resetNoise()
{
    ou_noise->reset();
}

std::pair<torch::Tensor, torch::Tensor> TD3Agent::getAction(torch::Tensor state, bool eval)
{
    if (!eval && learn_step < actor_warmup_steps)
    {
        // Generates values between -1.0 and 1.0
        torch::Tensor random_action = torch::rand({action_scaling_factors.size(0)}) * 2.0 - 1.0;
        return {random_action * action_scaling_factors + action_shift_factors, random_action};
    }

    actor_local->eval();
    torch::NoGradGuard no_grad;
    auto action = actor_local->forward(state.to(device)).to(torch::kCPU);
    if (!eval)
    {
        auto noise = ou_noise->sample();
        action = torch::clamp(action + noise, -1.0, 1.0);
    }
    torch::Tensor scaled_action = action * action_scaling_factors + action_shift_factors; // Scale action to environment limits
    actor_local->train();

    return {scaled_action, action}; // action here 1 dimensionsal (no batch dim since state does not have batch dim)
}

void TD3Agent::addExperience(torch::Tensor state, torch::Tensor action, torch::Tensor reward, torch::Tensor next_state, torch::Tensor done)
{
    replay_buffer.addExperienceState(state, action, reward, next_state, done);
}

void TD3Agent::setExplorationNoise(float noise)
{
    ou_noise->setSigma(noise);
}

void TD3Agent::learn()
{
    if (replay_buffer.getLength() < batch_size)
        return;

    Experience agent_exp;
    Experience human_exp;

    torch::Tensor states, actions, rewards, next_states, dones;

    if (use_human_buffer && human_replay_buffer.getLength() > 0)
    {
        int num_human = static_cast<int>(batch_size * human_data_percentage);
        int num_agent = batch_size - num_human;

        auto a_batch = replay_buffer.sample(num_agent);
        auto h_batch = human_replay_buffer.sample(std::min((int)human_replay_buffer.getLength(), num_human));

        states = torch::cat({std::get<0>(a_batch), std::get<0>(h_batch)}, 0);
        actions = torch::cat({std::get<1>(a_batch), std::get<1>(h_batch)}, 0);
        rewards = torch::cat({std::get<2>(a_batch), std::get<2>(h_batch)}, 0);
        next_states = torch::cat({std::get<3>(a_batch), std::get<3>(h_batch)}, 0);
        dones = torch::cat({std::get<4>(a_batch), std::get<4>(h_batch)}, 0);
    }
    else
    {
        auto experiences = replay_buffer.sample(batch_size);
        states = std::get<0>(experiences);
        actions = std::get<1>(experiences);
        rewards = std::get<2>(experiences);
        next_states = std::get<3>(experiences);
        dones = std::get<4>(experiences);
    }

    states = states.to(device);
    actions = actions.to(device);
    rewards = rewards.to(device);
    next_states = next_states.to(device);
    dones = dones.to(device);

    auto next_actions = actor_target->forward(next_states);
    auto noise = (torch::randn_like(next_actions) * 0.1).clamp(-0.2, 0.2);
    next_actions = (next_actions + noise).clamp(-1.0, 1.0);

    auto target_q1 = critic_target_1->forward(next_states, next_actions);
    auto target_q2 = critic_target_2->forward(next_states, next_actions);
    auto target_q = torch::min(target_q1, target_q2);
    auto expected_q = rewards + (gamma * target_q * (1 - dones));

    auto current_q1 = critic_local_1->forward(states, actions);
    auto current_q2 = critic_local_2->forward(states, actions);

    auto critic_loss_1 = torch::mse_loss(current_q1, expected_q.detach());
    auto critic_loss_2 = torch::mse_loss(current_q2, expected_q.detach());
    auto critic_loss = critic_loss_1 + critic_loss_2;

    critic_optimizer_1.zero_grad();
    critic_optimizer_2.zero_grad();
    critic_loss.backward();
    torch::nn::utils::clip_grad_norm_(critic_local_1->parameters(), 1.0);
    torch::nn::utils::clip_grad_norm_(critic_local_2->parameters(), 1.0);
    critic_optimizer_1.step();
    critic_optimizer_2.step();

    total_critic_loss += critic_loss.item<double>();
    if (learn_step % actor_update_freq == 0 && learn_step > actor_warmup_steps)
    {
        auto actor_loss = -critic_local_1->forward(states, actor_local->forward(states)).mean();

        actor_optimizer.zero_grad();
        actor_loss.backward();
        torch::nn::utils::clip_grad_norm_(actor_local->parameters(), 1.0);

        actor_optimizer.step();

        total_actor_loss += actor_loss.item<double>();

        softUpdate();
    }
    learn_step++;
}

void TD3Agent::toDevice(torch::Device d)
{
    device = d;
    actor_local->to(d);
    actor_target->to(d);
    critic_local_1->to(d);
    critic_target_1->to(d);
    critic_local_2->to(d);
    critic_target_2->to(d);
}

void TD3Agent::hardCopy()
{
    torch::NoGradGuard no_grad;
    for (size_t i = 0; i < actor_target->parameters().size(); i++)
        actor_target->parameters()[i].copy_(actor_local->parameters()[i]);
    for (size_t i = 0; i < critic_target_1->parameters().size(); i++)
        critic_target_1->parameters()[i].copy_(critic_local_1->parameters()[i]);
    for (size_t i = 0; i < critic_target_2->parameters().size(); i++)
        critic_target_2->parameters()[i].copy_(critic_local_2->parameters()[i]);
}

void TD3Agent::softUpdate() // TODO: if this is too slow, methods to make more efficient
{
    torch::NoGradGuard no_grad; //       disables calulation of gradients
    for (size_t i = 0; i < actor_target->parameters().size(); i++)
        actor_target->parameters()[i].copy_(tau * actor_local->parameters()[i] + (1.0 - tau) * actor_target->parameters()[i]); // global tau
    for (size_t i = 0; i < critic_target_1->parameters().size(); i++)
        critic_target_1->parameters()[i].copy_(tau * critic_local_1->parameters()[i] + (1.0 - tau) * critic_target_1->parameters()[i]); // global tau
    for (size_t i = 0; i < critic_target_2->parameters().size(); i++)
        critic_target_2->parameters()[i].copy_(tau * critic_local_2->parameters()[i] + (1.0 - tau) * critic_target_2->parameters()[i]); // global tau
}

void TD3Agent::pretrainFromHumanData(int iterations)
{
    if (human_replay_buffer.getLength() < batch_size)
        return;

    std::cout << "Seeding Agent with Expert Knowledge (BC + Critic Grounding)..." << std::endl;

    actor_local->train();
    critic_local_1->train();
    critic_local_2->train();
    float noise_std = 0.05f;
    for (int i = 0; i < iterations; i++)
    {
        auto experiences = human_replay_buffer.sample(batch_size);
        auto states = std::get<0>(experiences).to(device);
        auto actions = std::get<1>(experiences).to(device);
        auto rewards = std::get<2>(experiences).to(device);
        auto next_states = std::get<3>(experiences).to(device);
        auto dones = std::get<4>(experiences).to(device);

        auto predicted_actions = actor_local->forward(states);
        auto actor_loss = torch::mse_loss(predicted_actions, actions);

        actor_optimizer.zero_grad();
        actor_loss.backward();
        actor_optimizer.step();

        torch::Tensor expected_q;
        {
            torch::NoGradGuard no_grad;
            auto next_actions = actor_target->forward(next_states);
            auto target_q1 = critic_target_1->forward(next_states, next_actions);
            auto target_q2 = critic_target_2->forward(next_states, next_actions);
            auto target_q = torch::min(target_q1, target_q2);

            expected_q = rewards + (gamma * target_q * (1 - dones));
        }

        auto current_q1 = critic_local_1->forward(states, actions);
        auto current_q2 = critic_local_2->forward(states, actions);
        auto critic_loss = torch::mse_loss(current_q1, expected_q.detach()) +
                           torch::mse_loss(current_q2, expected_q.detach());

        critic_optimizer_1.zero_grad();
        critic_optimizer_2.zero_grad();
        critic_loss.backward();
        critic_optimizer_1.step();
        critic_optimizer_2.step();

        softUpdate();

        if (i % 100 == 0)
        {
            std::cout << "\rIter: " << i << "/" << iterations
                      << " | A_Loss: " << std::fixed << std::setprecision(4) << actor_loss.item<float>()
                      << " | C_Loss: " << critic_loss.item<float>() << std::flush;
        }
    }
    hardCopy();
    std::cout << "\nExpert seeding complete." << std::endl;
}

void TD3Agent::loadHumanData(std::shared_ptr<TrainEnvironment> env, const std::string &filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open human data file: " << filepath << std::endl;
        return;
    }

    std::string line;
    // Skip the header line
    std::getline(file, line);

    int s_dim = env->state_dim + env->obstacle_dim;
    int a_dim = env->action_dim;
    int count = 0;

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;
        std::vector<float> row;

        while (std::getline(ss, value, ','))
        {
            row.push_back(std::stof(value));
        }

        // CSV Structure: [0]timestamp, [1...s]state, [s+1...s+a]action, [s+a+1...2s+a]next_state, [end-1]reward, [end]done
        // Note: adjust indices if your CSV includes more or fewer columns

        auto state_vec = std::vector<float>(row.begin() + 1, row.begin() + 1 + s_dim);
        auto action_vec = std::vector<float>(row.begin() + 1 + s_dim, row.begin() + 1 + s_dim + a_dim);
        auto next_state_vec = std::vector<float>(row.begin() + 1 + s_dim + a_dim, row.begin() + 1 + 2 * s_dim + a_dim);
        float reward_val = row[row.size() - 2];
        float done_val = row.back();

        // Convert to Tensors
        torch::Tensor s = torch::tensor(state_vec);
        torch::Tensor a = torch::tensor(action_vec);
        torch::Tensor ns = torch::tensor(next_state_vec);
        torch::Tensor r = torch::tensor({reward_val});
        torch::Tensor d = torch::tensor({done_val});

        // Add specifically to the human buffer
        human_replay_buffer.addExperienceState(s, a, r, ns, d);
        count++;
    }

    std::cout << "Successfully loaded " << count << " expert transitions into the Human Buffer." << std::endl;
}

void TD3Agent::setLearningRates(float lr_actor, float lr_critic)
{
    for (auto &param_group : actor_optimizer.param_groups())
    {
        static_cast<torch::optim::AdamOptions &>(param_group.options()).lr(lr_actor);
    }
    for (auto &param_group : critic_optimizer_1.param_groups())
    {
        static_cast<torch::optim::AdamOptions &>(param_group.options()).lr(lr_critic);
    }
    for (auto &param_group : critic_optimizer_2.param_groups())
    {
        static_cast<torch::optim::AdamOptions &>(param_group.options()).lr(lr_critic);
    }
}

/****************************************POLICY OVER OPTIONS AGENT****************************************/

PolicyOverOptionsAgent::PolicyOverOptionsAgent(
    std::shared_ptr<TrainEnvironment> env,
    const std::vector<int> &layer_sizes,
    torch::Device device_,
    float lr_,
    float tau_,
    float gamma_,
    int batch_size_) : device(device_),
                       lr(lr_), tau(tau_), gamma(gamma_), batch_size(batch_size_),
                       q(env->state_dim, env->obstacle_feature_dim, layer_sizes, device_),
                       target_q(env->state_dim, env->obstacle_feature_dim, layer_sizes, device_)
{
    optimizer = std::make_unique<torch::optim::Adam>(q->parameters(), torch::optim::AdamOptions(lr));
    hardCopy();
}

void PolicyOverOptionsAgent::addOption(float initial_bias)
{
    option_count++;
    q->addOption(initial_bias);
    target_q->addOption(initial_bias);
    hardCopy();                                                                                       // ensure target_q has the same new parameters as q
    optimizer = std::make_unique<torch::optim::Adam>(q->parameters(), torch::optim::AdamOptions(lr)); // reset optimizer to include new parameters
}

torch::Tensor PolicyOverOptionsAgent::getOptions(torch::Tensor state)
{
    q->eval();                  // Set to evaluation mode (disables dropout, uses batch norm running stats)
    torch::NoGradGuard no_grad; // Disable gradient computation for inference
    auto q_values = q->forward(state.to(q->device)).to(torch::kCPU).squeeze();
    q->train(); // Reset to training mode
    return q_values;
}

void PolicyOverOptionsAgent::addExperience(torch::Tensor state, int option, float cumulative_reward, torch::Tensor next_state, bool done, int num_steps)
{
    addExperience(state, torch::tensor({(float)option}, torch::kInt64), torch::tensor({(float)cumulative_reward}), next_state, torch::tensor({(float)done}), torch::tensor({(float)num_steps}));
}

void PolicyOverOptionsAgent::addExperience(torch::Tensor state, torch::Tensor option, torch::Tensor cumulative_reward, torch::Tensor next_state, torch::Tensor done, torch::Tensor num_steps)
{
    int option_idx = option.item<int>();
    replay_buffers[option_idx].addExperienceState(state, option, cumulative_reward, next_state, done, num_steps);
}

void PolicyOverOptionsAgent::learn()
{
    for (auto &[option_idx, replay_buffer] : replay_buffers)
    {
        if (replay_buffer.getLength() < batch_size)
            continue;

        auto experiences = replay_buffer.sample(batch_size);
        auto states = std::get<0>(experiences).to(q->device);
        auto options = std::get<1>(experiences).to(q->device).to(torch::kInt64);
        auto cumulative_rewards = std::get<2>(experiences).to(q->device);
        auto next_states = std::get<3>(experiences).to(q->device);
        auto dones = std::get<4>(experiences).to(q->device).narrow(-1, 0, 1).to(torch::kInt64); // note that dones and num_steps are concatenated in the same tensor, with num_steps in the last dimension
        auto num_steps = std::get<4>(experiences).to(q->device).narrow(-1, 1, 1).to(torch::kInt64);
        auto q_values = q->forward(states);

        // Compute target Q values using target_q in eval mode (for consistency)
        target_q->eval();
        auto best_options = std::get<1>(torch::max(q->forward(next_states), -1, true)); // in torch, max returns values, indices
        // Double DQN: pick actions based on current q, but evaluate them with the target q. Gather allows us to select values of selected indices along a given dim
        auto target_q_values = target_q->forward(next_states).gather(-1, best_options).detach();
        target_q->train(); // Reset to training mode

        auto discount_factor = torch::pow(gamma, num_steps);
        auto y = cumulative_rewards + (discount_factor * target_q_values * (1 - dones));

        if (options.dim() == 1)
            options = options.unsqueeze(-1);
        auto q_values_selected = q_values.gather(-1, options);
        auto loss = torch::nn::functional::mse_loss(q_values_selected, y);

        optimizer->zero_grad();
        loss.backward();
        torch::nn::utils::clip_grad_norm_(q->parameters(), 1.0);
        optimizer->step();
    }

    softUpdate();
}

void PolicyOverOptionsAgent::hardCopy()
{
    torch::NoGradGuard no_grad;
    for (size_t i = 0; i < target_q->parameters().size(); i++)
        target_q->parameters()[i].copy_(q->parameters()[i]);
}

void PolicyOverOptionsAgent::softUpdate()
{
    torch::NoGradGuard no_grad;
    for (size_t i = 0; i < target_q->parameters().size(); i++)
        target_q->parameters()[i].copy_(tau * q->parameters()[i] + (1.0 - tau) * target_q->parameters()[i]);
}

/****************************************OU NOISE****************************************/

OUNoise::OUNoise(int size, float mu, float theta, float sigma)
    : size(size), mu(mu), theta(theta), sigma(sigma)
{
    state = torch::ones({size}) * mu;
}

void OUNoise::reset()
{
    state.fill_(mu);
}

void OUNoise::setSigma(float s)
{
    sigma = s;
}

torch::Tensor OUNoise::sample()
{
    // dx = theta * (mu - x) + sigma * dw
    auto noise = torch::randn({size});
    state += theta * (mu - state) + sigma * noise;
    return state;
}
