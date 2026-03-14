#pragma once
#include <torch/torch.h>
#include "robot_bridge_train.h"
#include <vector>
#include <math.h>

class TrainEnvironment
{
public:
    TrainEnvironment(std::shared_ptr<RobotBridgeTrain> robot_bridge_, int max_steps_) : robot_bridge(robot_bridge_), max_steps(max_steps_)
    {
        RobotState state = robot_bridge->getRobotState();
        state_dim = state.q.size() +
                    state.dq.size() +
                    state.position.size() +
                    state.velocity.size() +
                    state.accel.size() +
                    state.orientation.size() +
                    state.angular_velocity.size();
    }

    torch::Tensor reset()
    {
        robot_bridge->resetRobot();
        RobotState state = robot_bridge->getRobotState();
        goal_position = robot_bridge->generateRandomPos().first;
        current_step = 0;

        return robotStateToTensor(state);
    }

    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> step(const torch::Tensor &action)
    {
        std::vector<float> cmd(3, 0.0);
        cmd[0] = action.data_ptr<float>()[0];
        cmd[1] = action.data_ptr<float>()[1];
        cmd[2] = action.data_ptr<float>()[2];

        robot_bridge->publishVelCommand(cmd);
        robot_bridge->update();
        current_step++;
        auto [reward, terminated] = computeReward();

        return {
            robotStateToTensor(robot_bridge->getRobotState()),
            torch::tensor({reward}, torch::kFloat32),
            torch::tensor({(float)terminated}, torch::kFloat32) // Usually better to store 'done' as a float (0.0 or 1.0) for RL math
        };
    }
    int state_dim;
    int action_dim = 3;
    std::vector<float> action_limits = {0.5, 0.3, 0.2};

private:
    std::shared_ptr<RobotBridgeTrain> robot_bridge;
    std::array<float, 3> goal_position = {0.0, 0.0, 0.0};
    int max_steps;
    int current_step = 0;

    // torch::Tensor robotStateToTensor(const RobotState &state)
    // {
    //     std::vector<float> flat_state;

    //     // reserve space to avoid multiple reallocations
    //     flat_state.reserve(state_dim);

    //     float rel_x = goal_position[0] - state.position[0];
    //     float rel_y = goal_position[1] - state.position[1];
    //     float rel_z = goal_position[2] - state.position[2];

    //     flat_state.insert(flat_state.end(), state.q.begin(), state.q.end());
    //     flat_state.insert(flat_state.end(), state.dq.begin(), state.dq.end());
    //     flat_state.insert(flat_state.end(), rel_x);
    //     flat_state.insert(flat_state.end(), rel_y);
    //     flat_state.insert(flat_state.end(), rel_z);
    //     flat_state.insert(flat_state.end(), state.velocity.begin(), state.velocity.end());
    //     flat_state.insert(flat_state.end(), state.accel.begin(), state.accel.end());
    //     flat_state.insert(flat_state.end(), state.orientation.begin(), state.orientation.end());
    //     flat_state.insert(flat_state.end(), state.angular_velocity.begin(), state.angular_velocity.end());

    //     return torch::from_blob(flat_state.data(), {(int64_t)flat_state.size()}, torch::kFloat32).clone(); // tensor is essentially a pointer. Need the .clone() to make sure the data is owned by the tensor
    // }

    torch::Tensor robotStateToTensor(const RobotState &state)
    {
        auto options = torch::TensorOptions().dtype(torch::kFloat32);
        torch::Tensor tensor_state = torch::empty({(int64_t)state_dim}, options);

        float *data_ptr = tensor_state.data_ptr<float>();
        int offset = 0;

        // Use 'auto' to support both std::vector and std::array
        auto copy_to_ptr = [&](auto &src)
        {
            std::copy(src.begin(), src.end(), data_ptr + offset);
            offset += src.size();
        };

        // This will now work for state.q (array) and state.velocity (vector)
        copy_to_ptr(state.q);
        copy_to_ptr(state.dq);

        data_ptr[offset++] = goal_position[0] - state.position[0];
        data_ptr[offset++] = goal_position[1] - state.position[1];
        data_ptr[offset++] = goal_position[2] - state.position[2];

        copy_to_ptr(state.velocity);
        copy_to_ptr(state.accel);
        copy_to_ptr(state.orientation);
        copy_to_ptr(state.angular_velocity);

        return tensor_state;
    }

    std::pair<float, bool> computeReward()
    {
        RobotState state = robot_bridge->getRobotState();
        bool collision = robot_bridge->inCollision();

        float distance_to_goal = std::sqrt((state.position[0] - goal_position[0]) * (state.position[0] - goal_position[0]) +
                                           (state.position[1] - goal_position[1]) * (state.position[1] - goal_position[1]));

        float reward = 0;
        bool terminated = false;
        if (collision)
        {
            reward -= 1;
            terminated = true;
        }
        else if (distance_to_goal < 0.25) // ignoring velocity and orientation for now
        {
            reward += 1;
            terminated = true;
        }
        else
        {
            reward -= distance_to_goal / 10.0;
        }
        if (current_step >= max_steps)
        {
            terminated = true;
        }
        return {reward, terminated};
    }
};
