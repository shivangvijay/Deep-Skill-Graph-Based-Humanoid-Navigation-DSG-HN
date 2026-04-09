#pragma once
#include <torch/torch.h>
#include "robot_bridge_train.h"
#include <vector>
#include <math.h>
#include <algorithm>

/*

THIS FILE CONTAINS UTILITIES FOR USE WITH THE DEEP LEARNING AGENT

*/

class TrainEnvironment
{
public:
    TrainEnvironment(std::shared_ptr<RobotBridgeTrain> robot_bridge_, int max_steps_) : robot_bridge(robot_bridge_), max_steps(max_steps_)
    {
        RobotState state = robot_bridge->getRobotState();
        obstacles = robot_bridge->getObstacles();

        int gravity_dim = 3;                                // local_up
        int self_dyn_dim = 3 + 3;                            // local_vel, local_ang_vel
        int goal_dim = 3;                                    // relative_pos (body frame)
        int boundary_dim = 4;
        int last_action_dim = 3;                             // previous velocity command
        obstacle_dim = (int)obstacles.size() * obstacle_feature_dim;

        state_dim = gravity_dim + self_dyn_dim + goal_dim + boundary_dim + last_action_dim;

        std::array<float, 3> pos_scales = {(robot_bridge->x_max - robot_bridge->x_min) / 2, (robot_bridge->y_max - robot_bridge->y_min) / 2, 0.5}; // some of these may need to be tuned later
        std::array<float, 4> orientation_scales = {1, 1, 1, 1};
        std::array<float, 3> vel_scales = {action_scaling_factors[0], action_scaling_factors[1], 1};
        std::array<float, 3> ang_vel_scales = {action_scaling_factors[2], action_scaling_factors[2], action_scaling_factors[2]};

        env_scaling_factors = {pos_scales, orientation_scales, vel_scales, ang_vel_scales}; // these contain the scaling factor for each dim in teh env
    }

    torch::Tensor reset() // if no arguments passed, just reset to random position. If goal fixed, use that, else sample randomly
    {
        auto start = robot_bridge->generateRandomValidConfiguration();
        start.velocity[0] = 0;
        start.velocity[1] = 0;
        start.velocity[2] = 0;
        start.angular_velocity[0] = 0;
        start.angular_velocity[1] = 0;
        start.angular_velocity[2] = 0;

        if (!_goal_fixed)
        {
            int attempts = 0;
            do
            {
                goal = robot_bridge->generateRandomValidConfiguration();
                float dist = _euclidean2D(start.position, goal.position);
                if (dist >= min_goal_distance && dist <= max_goal_distance)
                    break;
            } while (++attempts < 200);

            goal.velocity[0] = 0;
            goal.velocity[1] = 0;
            goal.velocity[2] = 0;
            goal.angular_velocity[0] = 0;
            goal.angular_velocity[1] = 0;
            goal.angular_velocity[2] = 0;
        }

        robot_bridge->resetRobot(start.position, start.orientation, start.velocity, start.angular_velocity);

        obstacles = robot_bridge->getObstacles();

        current_step = 0;
        last_action = {0.0f, 0.0f, 0.0f};
        prev_action = {0.0f, 0.0f, 0.0f};
        prev_dist_to_goal = _euclidean2D(start.position, goal.position);

        return transformState(robot_bridge->getRobotState());
    }

    torch::Tensor getState() // returns everything in local frame (used for TD3 agent)
    {
        return transformState(robot_bridge->getRobotState());
    }

    torch::Tensor getStateRelativeToGoal(const AbstractedState &query_goal)
    {
        return transformState(robot_bridge->getRobotState(), query_goal);
    }

    std::pair<RobotState, bool> getUnderlyingState()
    {
        auto state = robot_bridge->getRobotState();
        bool collision = robot_bridge->inCollision() || state.position[0] > robot_bridge->x_max || state.position[0] < robot_bridge->x_min ||
                         state.position[1] > robot_bridge->y_max || state.position[1] < robot_bridge->y_min;
        return {state, collision};
    }

    AbstractedState getAbstractedState() // return underlying robot bridge state (where things are in global)
    {
        RobotState full_state = robot_bridge->getRobotState();
        AbstractedState abs_state = {full_state.position, full_state.orientation, full_state.velocity, full_state.angular_velocity};
        return abs_state;
    }

    AbstractedState getRandomValidAbstractedState()
    {
        return robot_bridge->generateRandomValidConfiguration();
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
        prev_action = last_action;
        last_action = {cmd[0], cmd[1], cmd[2]};
        auto [reward, terminated] = computeReward(goal);

        return {
            transformState(robot_bridge->getRobotState()),
            reward,
            terminated};
    }

    torch::Tensor resetTo(const AbstractedState &state)
    {
        return resetTo(state.position, state.orientation, state.velocity, state.angular_velocity);
    }

    // see note a top about how you need to set vel and ang vel
    torch::Tensor resetTo(const std::array<float, 3> &pos, const std::array<float, 4> &quat, const std::array<float, 3> &vel, const std::array<float, 3> &ang_vel)
    {
        // if required, pick random goal
        if (!_goal_fixed)
        {
            goal = robot_bridge->generateRandomValidConfiguration();
        }

        std::array<float, 3> clamped_pos = {
            std::max(robot_bridge->x_min + 0.5f, std::min(robot_bridge->x_max - 0.5f, pos[0])),
            std::max(robot_bridge->y_min + 0.5f, std::min(robot_bridge->y_max - 0.5f, pos[1])),
            pos[2]};

        robot_bridge->resetRobot(clamped_pos, quat, vel, ang_vel);
        obstacles = robot_bridge->getObstacles();

        current_step = 0;
        last_action = {0.0f, 0.0f, 0.0f};
        prev_action = {0.0f, 0.0f, 0.0f};
        prev_dist_to_goal = _euclidean2D(clamped_pos, goal.position);
        return transformState(robot_bridge->getRobotState());
    }

    std::array<float, 3> getGoalPosition() const { return goal.position; }

    std::pair<std::array<float, 3>, std::array<float, 4>> getRobotPose() const
    {
        RobotState s = robot_bridge->getRobotState();
        return {s.position, s.orientation};
    }

    // Fix goal_position to a specific point (e.g. next skill's subgoal).
    // reset() and resetTo() will not randomize goal_position while fixed.

    void setGoal(const AbstractedState &state)
    {
        goal = state;
    }

    // see note a top about how you need to set vel and ang vel
    void setGoal(const std::array<float, 3> &pos, const std::array<float, 4> &quat, const std::array<float, 3> &vel, const std::array<float, 3> &ang_vel)
    {
        goal = {pos, quat, vel, ang_vel}; // AbstractedState aggregate init
        _goal_fixed = true;
    }

    // Convenience: goal position only, zero velocity, identity orientation
    void setGoal(const std::array<float, 3> &pos)
    {
        setGoal(pos, {1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
    }

    // Convenience: reset to pos + quat, zero velocity
    torch::Tensor resetTo(const std::array<float, 3> &pos, const std::array<float, 4> &quat)
    {
        return resetTo(pos, quat, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
    }

    void clearGoal()
    {
        _goal_fixed = false;
    }

    void increaseGoalDistance(float delta)
    {
        max_goal_distance = std::min(max_goal_distance + delta, 11.0f);
    }

    void decreaseGoalDistance(float delta)
    {
        max_goal_distance = std::max(max_goal_distance - delta, min_goal_distance + 0.5f);
    }

    float getMaxGoalDistance() const { return max_goal_distance; }

    void updateGoalMarker()
    {
        robot_bridge->getEngine()->setGoalMarker(goal.position[0], goal.position[1], 0.5f);
    }

    std::pair<torch::Tensor, torch::Tensor> computeReward()
    {
        return computeReward(goal);
    }

    std::pair<torch::Tensor, torch::Tensor> computeReward(const AbstractedState &goal_)
    {
        RobotState state = robot_bridge->getRobotState();
        bool collision = robot_bridge->inCollision();
        return computeReward(state, collision, goal_);
    }

    std::pair<torch::Tensor, torch::Tensor> computeReward(const RobotState &state, bool collision, const AbstractedState &goal_, bool use_goal_radius = true)
    {
        auto goal_position = goal_.position;

        float pos_error = std::sqrt((state.position[0] - goal_position[0]) * (state.position[0] - goal_position[0]) +
                                    (state.position[1] - goal_position[1]) * (state.position[1] - goal_position[1]));

        float reward = 0;
        bool terminated = false;

        // 1. Obstacle proximity penalty (dense)
        float min_obs_dist = robot_bridge->distanceToNearestObstacle(state.position, state.orientation);
        constexpr float safe_margin = 0.8f;
        if (min_obs_dist < safe_margin)
        {
            reward -= 2.0f * (safe_margin - min_obs_dist);
        }

        // 2. Terminal conditions
        if (collision ||
            state.position[0] > robot_bridge->x_max || state.position[0] < robot_bridge->x_min ||
            state.position[1] > robot_bridge->y_max || state.position[1] < robot_bridge->y_min)
        {
            reward -= 30;
            terminated = true;
        }
        else if (use_goal_radius && pos_error < 0.5)
        {
            reward += 50;
            terminated = true;
        }
        else
        {
            // 3. Distance reduction reward (dense, directional)
            float dist_reduction = prev_dist_to_goal - pos_error;
            reward += 5.0f * dist_reduction;

            // 4. Time penalty (dense, direction-agnostic)
            reward -= 0.1f;

            // 5. Smoothness penalty (dense, direction-agnostic)
            float action_jerk = std::abs(last_action[0] - prev_action[0]) +
                                std::abs(last_action[1] - prev_action[1]) +
                                std::abs(last_action[2] - prev_action[2]);
            reward -= 0.5f * action_jerk;
        }

        // 6. Timeout penalty
        if (current_step >= max_steps)
        {
            reward -= 10;
            terminated = true;
        }

        prev_dist_to_goal = pos_error;

        return {torch::tensor({reward}, torch::kFloat32),
                torch::tensor({(float)terminated}, torch::kFloat32)};
    }

    torch::Tensor transformState(const RobotState &state, const AbstractedState &goal_)
    {
        // when designing this, note that the state is a mix of being in the global and local reference frame
        // however, the final policy output is velocity RELATIVE to the robot

        auto options = torch::TensorOptions().dtype(torch::kFloat32);
        torch::Tensor tensor_state = torch::empty({(int64_t)state_dim + obstacle_dim}, options);
        float *data_ptr = tensor_state.data_ptr<float>();
        int offset = 0;

        auto copy_to_ptr = [&](const auto &src)
        {
            std::copy(src.begin(), src.end(), data_ptr + offset);
            offset += src.size();
        };

        // local dynamics
        copy_to_ptr(rotateVectorByQuat({0, 0, 1}, state.orientation, true));      // local_up (3)
        copy_to_ptr(rotateVectorByQuat(state.velocity, state.orientation, true)); // local_vel (3)
        copy_to_ptr(state.angular_velocity);                                      // Already local (3)

        // goal-relative position in body frame
        auto &g_pos = goal_.position;

        std::array<float, 3> r_pos = {g_pos[0] - state.position[0], g_pos[1] - state.position[1], g_pos[2] - state.position[2]};
        r_pos[0] /= env_scaling_factors.position[0];
        r_pos[1] /= env_scaling_factors.position[1];
        r_pos[2] /= env_scaling_factors.position[2];

        copy_to_ptr(rotateVectorByQuat(r_pos, state.orientation, true));          // relative_pos (3)

        // relative distance to boundary
        float dist_left = state.position[0] - robot_bridge->x_min;
        float dist_right = robot_bridge->x_max - state.position[0];
        float dist_back = state.position[1] - robot_bridge->y_min;
        float dist_front = robot_bridge->y_max - state.position[1];

        data_ptr[offset++] = dist_left / env_scaling_factors.position[0];
        data_ptr[offset++] = dist_right / env_scaling_factors.position[0];
        data_ptr[offset++] = dist_back / env_scaling_factors.position[1];
        data_ptr[offset++] = dist_front / env_scaling_factors.position[1];

        for (int i = 0; i < obstacles.size(); i++)
        {
            const auto &obs = obstacles[i];
            std::array<float, 3> r_obs = {obs.position[0] - state.position[0],
                                          obs.position[1] - state.position[1],
                                          obs.position[2] - state.position[2]};
            r_obs[0] /= env_scaling_factors.position[0];
            r_obs[1] /= env_scaling_factors.position[1];
            r_obs[2] /= env_scaling_factors.position[2];
            copy_to_ptr(rotateVectorByQuat(r_obs, state.orientation, true));
            data_ptr[offset++] = obs.size[0];
        }

        copy_to_ptr(last_action);                                                // last_action (3)

        return tensor_state;
    }

    int state_dim;
    int obstacle_dim;
    int obstacle_feature_dim = 4;
    int action_dim = 3;
    std::vector<float> action_scaling_factors = {0.75, 0.3, 1.0};
    std::vector<float> action_shift_factors = {0.25, 0.0, 0.0};
    AbstractedState env_scaling_factors;

    float max_goal_distance = 3.0f;
    float min_goal_distance = 1.0f;

    const std::vector<Obstacle> &getObstacles() const { return obstacles; }

private:
    std::shared_ptr<RobotBridgeTrain> robot_bridge;
    AbstractedState goal = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    bool _goal_fixed = false;
    int max_steps;
    int current_step = 0;
    std::vector<Obstacle> obstacles;
    std::array<float, 3> last_action = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> prev_action = {0.0f, 0.0f, 0.0f};
    float prev_dist_to_goal = 0.0f;

    float _euclidean2D(const std::array<float, 3> &a, const std::array<float, 3> &b) const
    {
        float dx = a[0] - b[0];
        float dy = a[1] - b[1];
        return std::sqrt(dx * dx + dy * dy);
    }

    std::array<float, 4> quaternionDelta(const std::array<float, 4> &a, const std::array<float, 4> &b) // returns a - b
    {
        float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
        std::array<float, 4> a_adj = a;
        if (dot < 0.0f) // note that for a quaterion (-1, 0, 0, 0) is same as (1, 0, 0, 0)
        {
            for (int i = 0; i < 4; i++)
                a_adj[i] = -a[i];
        }

        std::array<float, 4> b_inv = {
            b[0],  // w
            -b[1], // -x
            -b[2], // -y
            -b[3]  // -z
        };

        // compute the Hamilton Product: delta = q_inv * goal_orientation
        std::array<float, 4> orientation_delta;
        const auto &q1 = b_inv;
        const auto &q2 = a_adj;

        orientation_delta[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]; // w
        orientation_delta[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]; // x
        orientation_delta[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]; // y
        orientation_delta[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]; // z
        return orientation_delta;
    }

    // used to put vector into another frame. Set use_inverse to true if you want to go from world->local
    std::array<float, 3> rotateVectorByQuat(const std::array<float, 3> &v,
                                            const std::array<float, 4> &q,
                                            bool use_inverse = false) const
    {
        std::array<float, 4> q_rot = q;
        if (use_inverse)
        {
            q_rot[1] = -q[1];
            q_rot[2] = -q[2];
            q_rot[3] = -q[3];
        }

        // quaternion-Vector multiplication: q * v * q_inv
        // We treat the vector as a pure quaternion [0, vx, vy, vz]

        float w1 = -q_rot[1] * v[0] - q_rot[2] * v[1] - q_rot[3] * v[2];
        float x1 = q_rot[0] * v[0] + q_rot[2] * v[2] - q_rot[3] * v[1];
        float y1 = q_rot[0] * v[1] + q_rot[3] * v[0] - q_rot[2] * v[2];

        float qw = q_rot[0], qx = q_rot[1], qy = q_rot[2], qz = q_rot[3];
        float vx = v[0], vy = v[1], vz = v[2];

        float tx = 2.0f * (qy * vz - qz * vy);
        float ty = 2.0f * (qz * vx - qx * vz);
        float tz = 2.0f * (qx * vy - qy * vx);

        std::array<float, 3> result;
        result[0] = vx + qw * tx + (qy * tz - qz * ty);
        result[1] = vy + qw * ty + (qz * tx - qx * tz);
        result[2] = vz + qw * tz + (qx * ty - qy * tx);

        return result;
    }

    torch::Tensor transformState(const RobotState &state)
    {
        return transformState(state, goal);
    }
};
