#include "agent.h"

/****************************************TD3 AGENT****************************************/

TD3Agent::TD3Agent(
    std::shared_ptr<TrainEnvironment> env,
    const std::vector<int> &actor_layer_sizes,
    std::vector<int> &critic_layer_sizes,
    torch::Device device_, float lr_actor,
    float lr_critic,
    float tau,
    float gamma,
    int batch_size,
    int actor_update_freq,
    int max_obstacles_,
    int actor_warmup_steps_) : device(device_), total_state_dim(max_obstacles_ * 4 + env->state_dim), actor_warmup_steps(actor_warmup_steps_),
                               actor_local(env->state_dim + max_obstacles_ * 4, env->action_dim, actor_layer_sizes, device_),
                               actor_target(env->state_dim + max_obstacles_ * 4, env->action_dim, actor_layer_sizes, device_),
                               critic_local_1(env->state_dim + max_obstacles_ * 4, env->action_dim, critic_layer_sizes, device_),
                               critic_target_1(env->state_dim + max_obstacles_ * 4, env->action_dim, critic_layer_sizes, device_),
                               critic_local_2(env->state_dim + max_obstacles_ * 4, env->action_dim, critic_layer_sizes, device_),
                               critic_target_2(env->state_dim + max_obstacles_ * 4, env->action_dim, critic_layer_sizes, device_),
                               actor_optimizer(actor_local->parameters(), torch::optim::AdamOptions(lr_actor)),
                               critic_optimizer_1(critic_local_1->parameters(), torch::optim::AdamOptions(lr_critic)),
                               critic_optimizer_2(critic_local_2->parameters(), torch::optim::AdamOptions(lr_critic)),
                               tau(tau), gamma(gamma), batch_size(batch_size), actor_update_freq(actor_update_freq), lr_actor(lr_actor), lr_critic(lr_critic)
{
    action_limits = torch::tensor(env->action_limits);
    hardCopy();
}

std::pair<torch::Tensor, torch::Tensor> TD3Agent::getAction(torch::Tensor state, bool eval)
{
    int64_t pad_size = total_state_dim - state.size(-1);
    torch::Tensor augmented_state;
    if (pad_size > 0)
    {
        augmented_state = torch::constant_pad_nd(state, {0, pad_size}, 0);
    }
    else
    {
        augmented_state = state.narrow(-1, 0, total_state_dim);
    }

    actor_local->eval();
    torch::NoGradGuard no_grad;
    auto action = actor_local->forward(augmented_state).to(torch::kCPU);
    if (!eval)
    {
        auto noise = (torch::randn_like(action) * 0.1).clamp(-0.2, 0.2); // Add some noise for exploration. Need to respect action limits
        action = torch::clamp(action + noise, -1.0, 1.0);
    }
    torch::Tensor scaled_action = action * action_limits; // Scale action to environment limits
    actor_local->train();

    return {scaled_action, action};
}

void TD3Agent::addExperience(torch::Tensor state, torch::Tensor action, torch::Tensor reward, torch::Tensor next_state, torch::Tensor done)
{
    int64_t pad_size = total_state_dim - state.size(-1);
    torch::Tensor augmented_state;
    if (pad_size > 0)
    {
        augmented_state = torch::constant_pad_nd(state, {0, pad_size}, 0);
    }
    else
    {
        augmented_state = state.narrow(-1, 0, total_state_dim);
    }

    pad_size = total_state_dim - next_state.size(-1);
    torch::Tensor augmented_next_state;
    if (pad_size > 0)
    {
        augmented_next_state = torch::constant_pad_nd(next_state, {0, pad_size}, 0);
    }
    else
    {
        augmented_next_state = next_state.narrow(-1, 0, total_state_dim);
    }

    replay_buffer.addExperienceState(augmented_state, action, reward, augmented_next_state, done);
}

void TD3Agent::learn()
{
    if (replay_buffer.getLength() < batch_size)
        return;

    auto experiences = replay_buffer.sample(batch_size);
    auto states = std::get<0>(experiences).to(actor_local->device);
    auto actions = std::get<1>(experiences).to(actor_local->device);
    auto rewards = std::get<2>(experiences).to(actor_local->device);
    auto next_states = std::get<3>(experiences).to(actor_local->device);
    auto dones = std::get<4>(experiences).to(actor_local->device);

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
                       q(env->state_dim + env->obstacle_dim, layer_sizes, device_),
                       target_q(env->state_dim + env->obstacle_dim, layer_sizes, device_)
{
    optimizer = std::make_unique<torch::optim::Adam>(q->parameters(), torch::optim::AdamOptions(lr));
    hardCopy();
}

void PolicyOverOptionsAgent::addOption(float initial_bias)
{
    q->addOption(initial_bias);
    target_q->addOption(initial_bias);
    hardCopy();                                                                                       // ensure target_q has the same new parameters as q
    optimizer = std::make_unique<torch::optim::Adam>(q->parameters(), torch::optim::AdamOptions(lr)); // reset optimizer to include new parameters
}

int PolicyOverOptionsAgent::getOption(torch::Tensor state)
{
    auto options = q->forward(state).to(torch::kCPU);
    auto best_option = options.argmax(-1).item<int>();
    // std::cout << "Max Q-Value: " << std::get<0>(options.max(-1)).item<float>() << std::endl;
    return best_option;
}

void PolicyOverOptionsAgent::addExperience(torch::Tensor state, int option, torch::Tensor cumulative_reward, torch::Tensor next_state, torch::Tensor done, int num_steps)
{
    addExperience(state, torch::tensor({(float)option}, torch::kInt64), cumulative_reward, next_state, done, torch::tensor({(float)num_steps}));
}

void PolicyOverOptionsAgent::addExperience(torch::Tensor state, torch::Tensor option, torch::Tensor cumulative_reward, torch::Tensor next_state, torch::Tensor done, torch::Tensor num_steps)
{
    replay_buffer.addExperienceState(state, option, cumulative_reward, next_state, done, num_steps);
}

void PolicyOverOptionsAgent::learn()
{
    if (replay_buffer.getLength() < batch_size)
        return;

    auto experiences = replay_buffer.sample(batch_size);
    auto states = std::get<0>(experiences).to(q->device);
    auto options = std::get<1>(experiences).to(q->device).to(torch::kInt64);
    auto cumulative_rewards = std::get<2>(experiences).to(q->device);
    auto next_states = std::get<3>(experiences).to(q->device);
    auto dones = std::get<4>(experiences).to(q->device).narrow(-1, 0, 1).to(torch::kInt64); // note that dones and num_steps are concatenated in the same tensor, with num_steps in the last dimension
    auto num_steps = std::get<4>(experiences).to(q->device).narrow(-1, 1, 1).to(torch::kInt64);
    auto q_values = q->forward(states);

    auto target_q_values = std::get<0>(torch::max(target_q->forward(next_states), -1, true));
    auto discount_factor = torch::pow(gamma, num_steps);
    auto y = cumulative_rewards + (discount_factor * target_q_values * (1 - dones));

    if (options.dim() == 1)
        options = options.unsqueeze(-1);
    auto q_values_selected = q_values.gather(-1, options);
    auto loss = torch::nn::functional::mse_loss(q_values_selected, y.detach());

    optimizer->zero_grad();
    loss.backward();
    torch::nn::utils::clip_grad_norm_(q->parameters(), 1.0);
    optimizer->step();

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