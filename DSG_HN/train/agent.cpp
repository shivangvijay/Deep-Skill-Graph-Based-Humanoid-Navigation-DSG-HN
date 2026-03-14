#include "agent.h"

TD3Agent::TD3Agent(
    std::shared_ptr<TrainEnvironment> env,
    const std::vector<int> &actor_layer_sizes,
    std::vector<int> &critic_layer_sizes,
    torch::Device device_, float lr_actor,
    float lr_critic,
    float tau,
    float gamma,
    int batch_size,
    int actor_update_freq) : device(device_),
                             actor_local(env->state_dim, env->action_dim, actor_layer_sizes, device_),
                             actor_target(env->state_dim, env->action_dim, actor_layer_sizes, device_),
                             critic_local_1(env->state_dim, env->action_dim, critic_layer_sizes, device_),
                             critic_target_1(env->state_dim, env->action_dim, critic_layer_sizes, device_),
                             critic_local_2(env->state_dim, env->action_dim, critic_layer_sizes, device_),
                             critic_target_2(env->state_dim, env->action_dim, critic_layer_sizes, device_),
                             actor_optimizer(actor_local->parameters(), torch::optim::AdamOptions(lr_actor)),
                             critic_optimizer_1(critic_local_1->parameters(), torch::optim::AdamOptions(lr_critic)),
                             critic_optimizer_2(critic_local_2->parameters(), torch::optim::AdamOptions(lr_critic)),
                             tau(tau), gamma(gamma), batch_size(batch_size), actor_update_freq(actor_update_freq), lr_actor(lr_actor), lr_critic(lr_critic)
{
    action_limits = torch::tensor(env->action_limits);
    last_action = torch::zeros({env->action_dim});
    hard_copy();
}

std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> TD3Agent::act(std::shared_ptr<TrainEnvironment> env, torch::Tensor state, bool eval)
{
    if (is_first_step)
    {
        last_action = torch::zeros({env->action_dim});
        is_first_step = false;
    }
    actor_local->eval();
    torch::NoGradGuard no_grad;
    torch::Tensor augmented_state = torch::cat({state, last_action}, -1);
    auto action = actor_local->forward(augmented_state).to(torch::kCPU);
    if (!eval)
    {
        auto noise = torch::randn_like(action) * 0.05; // Add some noise for exploration. Need to respect action limits
        action = torch::clamp(action + noise, -1.0, 1.0);
    }
    torch::Tensor scaled_action = action * action_limits; // Scale action to environment limits

    last_action = action.clone().squeeze(0); // Store the action for the next step. Remove batch dimension if it exists
    // std::cout << "Action taken: " << scaled_action.data_ptr<float>()[0] << " " << scaled_action.data_ptr<float>()[1] << " " << scaled_action.data_ptr<float>()[2] << std::endl;

    // scaled_action = torch::zeros_like(scaled_action).to(torch::kCPU); // --- IGNORE --- Remove this line to enable actual actions, currently just testing with zero actions
    auto [next_state, reward, done] = env->step(scaled_action);
    total_reward += reward.item<double>();
    replay_buffer.addExperienceState(augmented_state, action, reward, next_state, done);
    actor_local->train();

    if (done.data_ptr<float>()[0] > 0.5) // episode ended
    {
        is_first_step = true;
    }

    return {next_state, reward, done};
}

void TD3Agent::learn(std::shared_ptr<TrainEnvironment> env)
{
    if (replay_buffer.getLength() < batch_size)
        return;

    auto experiences = replay_buffer.sample(batch_size);
    auto states = std::get<0>(experiences).to(actor_local->device);
    auto actions = std::get<1>(experiences).to(actor_local->device);
    auto rewards = std::get<2>(experiences).to(actor_local->device);
    auto next_states = std::get<3>(experiences).to(actor_local->device);
    auto next_states_augmented = torch::cat({next_states, actions}, -1); // Augment next states with last actions for critic input
    auto dones = std::get<4>(experiences).to(actor_local->device);

    auto next_actions = actor_target->forward(next_states_augmented);
    auto noise = torch::randn_like(next_actions) * 0.2;
    next_actions = (next_actions + noise).clamp(-1.0, 1.0);

    auto target_q1 = critic_target_1->forward(next_states_augmented, next_actions);
    auto target_q2 = critic_target_2->forward(next_states_augmented, next_actions);
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

    // std::cout << "Critic loss: " << critic_loss.item<double>() << std::endl;

    total_critic_loss += critic_loss.item<double>();
    if (learn_step % actor_update_freq == 0)
    {
        // std::cout << "Updating actor..." << std::endl;
        auto actor_loss = -critic_local_1->forward(states, actor_local->forward(states)).mean();

        actor_optimizer.zero_grad();
        actor_loss.backward();
        torch::nn::utils::clip_grad_norm_(actor_local->parameters(), 1.0);

        actor_optimizer.step();

        total_actor_loss += actor_loss.item<double>();

        soft_update();
    }
}

void TD3Agent::hard_copy()
{
    torch::NoGradGuard no_grad;
    for (size_t i = 0; i < actor_target->parameters().size(); i++)
        actor_target->parameters()[i].copy_(actor_local->parameters()[i]);
    for (size_t i = 0; i < critic_target_1->parameters().size(); i++)
        critic_target_1->parameters()[i].copy_(critic_local_1->parameters()[i]);
    for (size_t i = 0; i < critic_target_2->parameters().size(); i++)
        critic_target_2->parameters()[i].copy_(critic_local_2->parameters()[i]);
}

void TD3Agent::soft_update() // TODO: if this is too slow, methods to make more efficient
{
    torch::NoGradGuard no_grad; //       disables calulation of gradients
    for (size_t i = 0; i < actor_target->parameters().size(); i++)
        actor_target->parameters()[i].copy_(tau * actor_local->parameters()[i] + (1.0 - tau) * actor_target->parameters()[i]); // global tau
    for (size_t i = 0; i < critic_target_1->parameters().size(); i++)
        critic_target_1->parameters()[i].copy_(tau * critic_local_1->parameters()[i] + (1.0 - tau) * critic_target_1->parameters()[i]); // global tau
    for (size_t i = 0; i < critic_target_2->parameters().size(); i++)
        critic_target_2->parameters()[i].copy_(tau * critic_local_2->parameters()[i] + (1.0 - tau) * critic_target_2->parameters()[i]); // global tau
}
