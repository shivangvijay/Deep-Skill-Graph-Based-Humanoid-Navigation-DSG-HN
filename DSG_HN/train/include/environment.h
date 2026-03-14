#pragma once
#include <torch/torch.h>
#include "robot_bridge_train.h"
#include <vector>
#include <math.h>

class TrainEnvironment
{
public:
    TrainEnvironment(std::shared_ptr<RobotBridgeTrain> robot_bridge_) : robot_bridge(robot_bridge_)
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

        return robotStateToTensor(state);
    }

    std::tuple<torch::Tensor, float, bool> step(const torch::Tensor &action)
    {
        std::vector<float> cmd(action.data_ptr<float>(), action.data_ptr<float>() + action.size(0));
        robot_bridge->publishVelCommand(cmd);
        robot_bridge->update();
        auto [reward, terminated] = computeReward();

        return {robotStateToTensor(robot_bridge->getRobotState()), reward, terminated};
    }

private:
    std::shared_ptr<RobotBridgeTrain> robot_bridge;
    int state_dim;
    std::array<float, 3> goal_position = {0.0, 0.0, 0.0}; 

    torch::Tensor robotStateToTensor(const RobotState &state)
    {
        std::vector<float> flat_state;

        // reserve space to avoid multiple reallocations
        flat_state.reserve(state_dim);

        flat_state.insert(flat_state.end(), state.q.begin(), state.q.end());
        flat_state.insert(flat_state.end(), state.dq.begin(), state.dq.end());
        flat_state.insert(flat_state.end(), state.position.begin(), state.position.end());
        flat_state.insert(flat_state.end(), state.velocity.begin(), state.velocity.end());
        flat_state.insert(flat_state.end(), state.accel.begin(), state.accel.end());
        flat_state.insert(flat_state.end(), state.orientation.begin(), state.orientation.end());
        flat_state.insert(flat_state.end(), state.angular_velocity.begin(), state.angular_velocity.end());

        return torch::from_blob(flat_state.data(), {(int64_t)flat_state.size()}, torch::kFloat32).clone(); // tensor is essentially a pointer
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
            reward -= 10;
            terminated = true;
        } else if (distance_to_goal < 0.25)
        {
            reward += 10;
            terminated = true;
        }
        else
        {
        reward -= distance_to_goal;
        }
        return {reward, terminated};
    }
};
