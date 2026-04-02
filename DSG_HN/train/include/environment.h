#pragma once
#include <torch/torch.h>
#include "robot_bridge_train.h"
#include <vector>
#include <math.h>

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

        int proprio_dim = state.q.size() + state.dq.size(); // 2 * DOF
        int gravity_dim = 3;                                // local_up
        int self_dyn_dim = 3 + 3 + 3;                       // local_vel, local_ang_vel, local_accel
        int goal_dim = 3 + 4 + 3 + 3;                       // relative_pos, releative_orientation, relative_vel, relative angular vel
        obstacle_dim = (int)obstacles.size() * 4;

        state_dim = proprio_dim + gravity_dim + self_dyn_dim + goal_dim;

        std::array<float, 3> pos_scales = {(robot_bridge->x_max - robot_bridge->x_min) / 2, (robot_bridge->y_max - robot_bridge->y_min) / 2, 0.5}; // some of these may need to be tuned later
        std::array<float, 4> orientation_scales = {1, 1, 1, 1};
        std::array<float, 3> vel_scales = {action_scaling_factors[0], action_scaling_factors[1], 1};
        std::array<float, 3> ang_vel_scales = {action_scaling_factors[2], action_scaling_factors[2], action_scaling_factors[2]};

        env_scaling_factors = {pos_scales, orientation_scales, vel_scales, ang_vel_scales}; // these contain the scaling factor for each dim in teh env
    }

    torch::Tensor reset() // if no arguments passed, just reset to random position. If goal fixed, use that, else sample randomly
    {
        // pick random valid goal position
        // Clamp to policy command limits: scaled_action = raw * scaling + shift → range = [shift-scaling, shift+scaling]
        auto clamp_f = [](float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); };
        float vx_min = action_shift_factors[0] - action_scaling_factors[0]; // 0.25 - 0.75 = -0.5
        float vx_max = action_shift_factors[0] + action_scaling_factors[0]; // 0.25 + 0.75 = 1.0
        float vy_lim = action_scaling_factors[1];                            // 0.3
        float yaw_lim = action_scaling_factors[2];                           // 1.0

        if (!_goal_fixed)
        {
            goal = robot_bridge->generateRandomValidConfiguration();
            goal.velocity[0]         = clamp_f(goal.velocity[0], vx_min, vx_max);
            goal.velocity[1]         = clamp_f(goal.velocity[1], -vy_lim, vy_lim);
            goal.velocity[2]         = 0.0f;
            goal.angular_velocity[0] = 0.0f;   // roll — near-zero during walking
            goal.angular_velocity[1] = 0.0f;   // pitch — near-zero during walking
            goal.angular_velocity[2] = clamp_f(goal.angular_velocity[2], -yaw_lim, yaw_lim); // yaw only
        }

        auto start = robot_bridge->generateRandomValidConfiguration();
        start.velocity[0]         = clamp_f(start.velocity[0], vx_min, vx_max);
        start.velocity[1]         = clamp_f(start.velocity[1], -vy_lim, vy_lim);
        start.velocity[2]         = 0.0f;
        start.angular_velocity[0] = 0.0f;   // roll — near-zero during walking
        start.angular_velocity[1] = 0.0f;   // pitch — near-zero during walking
        start.angular_velocity[2] = clamp_f(start.angular_velocity[2], -yaw_lim, yaw_lim); // yaw only
        robot_bridge->resetRobot(start.position, start.orientation, start.velocity, start.angular_velocity);

        obstacles = robot_bridge->getObstacles();

        current_step = 0;

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
        return {robot_bridge->getRobotState(), robot_bridge->inCollision()};
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

        auto clamp_f = [](float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); };
        float vx_min = action_shift_factors[0] - action_scaling_factors[0];
        float vx_max = action_shift_factors[0] + action_scaling_factors[0];
        float vy_lim = action_scaling_factors[1];
        float yaw_lim = action_scaling_factors[2];

        std::array<float, 3> clamped_pos = {
            clamp_f(pos[0], robot_bridge->x_min + 0.5f, robot_bridge->x_max - 0.5f),
            clamp_f(pos[1], robot_bridge->y_min + 0.5f, robot_bridge->y_max - 0.5f),
            pos[2]};

        std::array<float, 3> clamped_vel = {
            clamp_f(vel[0], vx_min, vx_max),
            clamp_f(vel[1], -vy_lim, vy_lim),
            0.0f};

        std::array<float, 3> clamped_ang_vel = {
            0.0f,
            0.0f,
            clamp_f(ang_vel[2], -yaw_lim, yaw_lim)};

        robot_bridge->resetRobot(clamped_pos, quat, clamped_vel, clamped_ang_vel);

        // If the fixed spawn position is in collision (e.g. gestation record near an obstacle),
        // fall back to a guaranteed collision-free random configuration rather than starting
        // an episode that immediately terminates with a -30 penalty.
        if (robot_bridge->inCollision())
        {
            auto fallback = robot_bridge->generateRandomValidConfiguration();
            robot_bridge->resetRobot(fallback.position, fallback.orientation, fallback.velocity, fallback.angular_velocity);
        }

        obstacles = robot_bridge->getObstacles();

        current_step = 0;
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

    std::pair<torch::Tensor, torch::Tensor> computeReward()
    {
        return computeReward(goal);
    }

    std::pair<torch::Tensor, torch::Tensor> computeReward(const AbstractedState &goal_)
    {
        RobotState state = robot_bridge->getRobotState();
        bool collision = robot_bridge->inCollision();
        return computeReward(state, collision, goal);
    }

    // If we do not have the ability to turn, then velocity cannot be part of goal condition
    std::pair<torch::Tensor, torch::Tensor> computeReward(const RobotState& state, bool collision, const AbstractedState &goal_)
    {
        auto goal_position      = goal_.position;
        auto goal_orientation   = goal_.orientation;
        auto goal_velocity      = goal_.velocity;
        auto goal_angular_velocity = goal_.angular_velocity;

        float pos_error = std::sqrt(
            std::pow(state.position[0] - goal_position[0], 2) +
            std::pow(state.position[1] - goal_position[1], 2));

        // XY linear velocity error
        float vel_error = std::sqrt(
            std::pow(state.velocity[0] - goal_velocity[0], 2) +
            std::pow(state.velocity[1] - goal_velocity[1], 2));

        // Yaw rate only — roll/pitch are near-zero during locomotion and not meaningful targets
        float ang_vel_error = std::abs(state.angular_velocity[2] - goal_angular_velocity[2]);

        float reward = 0;
        bool terminated = false;

        if (collision)
        {
            reward -= 30;
            terminated = true;
        }
        else
        {
            // Success: position must match; velocity must also match when weight > 0
            bool pos_ok = pos_error < 0.25f;
            bool vel_ok = velocity_weight < 1e-3f ||
                          (vel_error < vel_success_threshold &&
                           ang_vel_error < ang_vel_success_threshold);

            if (pos_ok && vel_ok)
            {
                reward += 50;
                terminated = true;
            }
            else
            {
                // Dense position shaping (always active)
                reward -= (pos_error / 50.0f);

                // Velocity shaping: only active when close to target AND weight > 0
                if (velocity_weight > 1e-3f && pos_error < vel_shaping_radius)
                {
                    reward -= velocity_weight * (vel_error / vel_penalty_scale);
                    reward -= velocity_weight * (ang_vel_error / vel_penalty_scale);
                }
            }
        }

        if (current_step >= max_steps || pos_error > 20)
            terminated = true;

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

        // note: pos, vel, and orientation are given in global frame
        // accel, angular vel given in local frame

        copy_to_ptr(state.q);
        copy_to_ptr(state.dq);

        // local dynamics
        copy_to_ptr(rotateVectorByQuat({0, 0, 1}, state.orientation, true));      // local_up
        copy_to_ptr(rotateVectorByQuat(state.velocity, state.orientation, true)); // local_vel
        copy_to_ptr(state.angular_velocity);                                      // Already local
        copy_to_ptr(state.accel);                                                 // Already local

        // relative values
        auto &g_pos = goal_.position;
        auto &g_quat = goal_.orientation;
        auto &g_vel = goal_.velocity;
        auto &g_ang_vel = goal_.angular_velocity;

        std::array<float, 3> r_pos = {g_pos[0] - state.position[0], g_pos[1] - state.position[1], g_pos[2] - state.position[2]};
        copy_to_ptr(rotateVectorByQuat(r_pos, state.orientation, true));

        copy_to_ptr(quaternionDelta(g_quat, state.orientation));

        std::array<float, 3> v_err = {g_vel[0] - state.velocity[0], g_vel[1] - state.velocity[1], g_vel[2] - state.velocity[2]};
        copy_to_ptr(rotateVectorByQuat(v_err, state.orientation, true));

        std::array<float, 3> local_goal_ang_vel = rotateVectorByQuat(g_ang_vel, state.orientation, true);

        std::array<float, 3> ang_vel_err = {
            local_goal_ang_vel[0] - state.angular_velocity[0],
            local_goal_ang_vel[1] - state.angular_velocity[1],
            local_goal_ang_vel[2] - state.angular_velocity[2]};

        copy_to_ptr(ang_vel_err);

        // local obstacles

        std::vector<Obstacle> sorted_obs = obstacles;
        std::sort(sorted_obs.begin(), sorted_obs.end(), [&](const Obstacle &a, const Obstacle &b)
                  {
        float distA = std::pow(a.position[0]-state.position[0], 2) + std::pow(a.position[1]-state.position[1], 2);
        float distB = std::pow(b.position[0]-state.position[0], 2) + std::pow(b.position[1]-state.position[1], 2);
        return distA < distB; });

        for (int i = 0; i < sorted_obs.size(); i++)
        {
            const auto &obs = sorted_obs[i];
            std::array<float, 3> r_obs = {obs.position[0] - state.position[0],
                                          obs.position[1] - state.position[1],
                                          obs.position[2] - state.position[2]};
            copy_to_ptr(rotateVectorByQuat(r_obs, state.orientation, true));
            data_ptr[offset++] = obs.size[0];
        }

        return tensor_state;
    }

    int state_dim;
    int obstacle_dim;
    int action_dim = 3;
    std::vector<float> action_scaling_factors = {0.75, 0.3, 1.0};
    std::vector<float> action_shift_factors = {0.25, 0.0, 0.0};
    AbstractedState env_scaling_factors;

    // Velocity reward config — set by DSC each episode via curriculum ramp
    float velocity_weight           = 0.0f;
    float vel_success_threshold     = 0.4f;
    float ang_vel_success_threshold = 0.4f;
    float vel_shaping_radius        = 1.0f;
    float vel_penalty_scale         = 20.0f;

private:
    std::shared_ptr<RobotBridgeTrain> robot_bridge;
    AbstractedState goal = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    bool _goal_fixed = false;
    int max_steps;
    int current_step = 0;
    std::vector<Obstacle> obstacles;

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
