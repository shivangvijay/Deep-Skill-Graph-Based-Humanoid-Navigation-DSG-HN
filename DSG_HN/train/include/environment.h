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
        obstacles = robot_bridge->getObstacles();
        int obstacle_dim = int(obstacles.size()) * 4; // size + relative pos of obstacles. Assuming obstacles are cylinders for now

        state_dim = state.q.size() +
                    state.dq.size() +
                    state.position.size() +
                    state.velocity.size() +
                    state.accel.size() +
                    state.orientation.size() +
                    state.angular_velocity.size() +
                    obstacle_dim;
    }

    torch::Tensor reset()
    {
        robot_bridge->resetRobot();
        obstacles = robot_bridge->getObstacles();
        RobotState state = robot_bridge->getRobotState();
        auto [goal_pos, goal_orientation] = robot_bridge->generateRandomPos();
        int attempts = 0;
        while (robot_bridge->distanceToNearestObstacle(goal_pos, goal_orientation) < 0.5f)
        {
            std::tie(goal_pos, goal_orientation) = robot_bridge->generateRandomPos();
            if (attempts++ > 100) throw std::runtime_error("Could Note Respawn Robot.");
        }
        if (!_goal_fixed)
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
    torch::Tensor resetTo(const std::array<float, 3> &pos, const std::array<float, 4> &quat)
    {
        robot_bridge->resetRobot(pos, quat);
        obstacles = robot_bridge->getObstacles();
        if (!_goal_fixed)
            goal_position = robot_bridge->generateRandomPos().first;
        current_step = 0;
        return robotStateToTensor(robot_bridge->getRobotState());
    }

    std::pair<std::array<float, 3>, std::array<float, 4>> getRobotPose() const
    {
        RobotState s = robot_bridge->getRobotState();
        return {s.position, s.orientation};
    }

    // Fix goal_position to a specific point (e.g. next skill's subgoal).
    // reset() and resetTo() will not randomize goal_position while fixed.
    void setGoal(const std::array<float, 3> &pos)
    {
        goal_position = pos;
        _goal_fixed   = true;
    }

    void clearGoal()
    {
        _goal_fixed = false;
    }

    int state_dim;
    int action_dim = 3;
    std::vector<float> action_limits = {0.5, 0.3, 0.2};

private:
    std::shared_ptr<RobotBridgeTrain> robot_bridge;
    std::array<float, 3> goal_position = {0.0, 0.0, 0.0};
    bool _goal_fixed = false;
    int max_steps;
    int current_step = 0;
    std::vector<Obstacle> obstacles;

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

        // TODO: update RL formulation to reflect the fact that termination/initiation are function
        // of velociy and orientation as well

        copy_to_ptr(state.q);
        copy_to_ptr(state.dq);

        data_ptr[offset++] = goal_position[0] - state.position[0];
        data_ptr[offset++] = goal_position[1] - state.position[1];
        data_ptr[offset++] = goal_position[2] - state.position[2];

        copy_to_ptr(state.velocity);
        copy_to_ptr(state.accel);
        copy_to_ptr(state.orientation);
        copy_to_ptr(state.angular_velocity);

        for (const auto &obs : obstacles)
        {
            data_ptr[offset++] = obs.position[0] - state.position[0];
            data_ptr[offset++] = obs.position[1] - state.position[1]; 
            data_ptr[offset++] = obs.position[2] - state.position[2];
            data_ptr[offset++] = obs.size[0]; // assuming cylindrical, so only care about radius
        }

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
            reward -= 10;
            terminated = true;
        }
        else if (distance_to_goal < 0.5) // ignoring velocity and orientation for now
        {
            reward += 50;
            terminated = true;
        }
        else
        {
            reward -= distance_to_goal / 100.0;
        }
        if (current_step >= max_steps || distance_to_goal > 20)
        {
            terminated = true;
        }
        return {reward, terminated};
    }
};
