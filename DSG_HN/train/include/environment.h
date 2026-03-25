#pragma once
#include <torch/torch.h>
#include "robot_bridge_train.h"
#include <vector>
#include <math.h>


/*

IMPORTANT: WHEN SETTING GOAL: NOTE THAT FOR VEL, THE LAST COMPONENT OF THE LINEAR VEL SHOULD BE ZERO. FOR ANGULAR VEL, ONLY THE LAST
COMPONENT (YAW) SHOULD BE NON-ZERO

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
        int obstacle_dim = (int)obstacles.size() * 4;

        state_dim = proprio_dim + gravity_dim + self_dyn_dim + goal_dim + obstacle_dim;
    }

    torch::Tensor reset() // if no arguments passed, just reset to random position. If goal fixed, use that, else sample randomly
    {
        auto [pos, quat, vel, ang_vel] = robot_bridge->generateRandomPoseWithVel();
        robot_bridge->resetRobot(pos, quat, vel, ang_vel);
        obstacles = robot_bridge->getObstacles();

        if (!_goal_fixed)
        {
            auto [rp, rq, rv, ra] = robot_bridge->generateRandomPoseWithVel();
            goal = {rp, rq, rv, ra};
        }
        current_step = 0;
        last_pos_error = 0;
        last_vel_error = 0;

        return transformState(robot_bridge->getRobotState());
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
            transformState(robot_bridge->getRobotState()),
            torch::tensor({reward}, torch::kFloat32),
            torch::tensor({(float)terminated}, torch::kFloat32) // Usually better to store 'done' as a float (0.0 or 1.0) for RL math
        };
    }

    // see note a top about how you need to set vel and ang vel
    torch::Tensor resetTo(const std::array<float, 3> &pos, const std::array<float, 4> &quat, const std::array<float, 3> &vel, const std::array<float, 3> &ang_vel)
    {
        robot_bridge->resetRobot(pos, quat, vel, ang_vel);
        obstacles = robot_bridge->getObstacles();
        if (!_goal_fixed)
            auto [rp, rq, rv, ra] = robot_bridge->generateRandomPoseWithVel();
            goal = {rp, rq, rv, ra};
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

    // see note a top about how you need to set vel and ang vel
    void setGoal(const std::array<float, 3> &pos, const std::array<float, 4> &quat, const std::array<float, 3> &vel, const std::array<float, 3> &ang_vel)
    {
        goal = {pos, quat, vel, ang_vel};  // AbstractedState aggregate init
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

    int state_dim;
    int action_dim = 3;
    std::vector<float> action_limits = {0.5, 0.3, 0.2};

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
        // when designing this, note that the state is a mix of being in the global and local reference frame
        // however, the final policy output is velocity RELATIVE to the robot

        auto options = torch::TensorOptions().dtype(torch::kFloat32);
        torch::Tensor tensor_state = torch::empty({(int64_t)state_dim}, options);
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
        auto &g_pos     = goal.position;
        auto &g_quat    = goal.orientation;
        auto &g_vel     = goal.velocity;
        auto &g_ang_vel = goal.angular_velocity;

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
        for (const auto &obs : obstacles)
        {
            std::array<float, 3> r_obs = {obs.position[0] - state.position[0], obs.position[1] - state.position[1], obs.position[2] - state.position[2]};
            copy_to_ptr(rotateVectorByQuat(r_obs, state.orientation, true));
            data_ptr[offset++] = obs.size[0];
        }

        return tensor_state;
    }

    // formulation where we reward for moving towards target seems better than one that just has negative distances

    // std::pair<float, bool> computeReward()
    // {
    //     RobotState state = robot_bridge->getRobotState();
    //     bool collision = robot_bridge->inCollision();

    //     auto &[goal_position, goal_orientation, goal_velocity, goal_angular_velocity] = goal;

    //     float pos_error = std::sqrt((state.position[0] - goal_position[0]) * (state.position[0] - goal_position[0]) +
    //                                 (state.position[1] - goal_position[1]) * (state.position[1] - goal_position[1]));

    //     float vel_error = std::sqrt((state.velocity[0] - goal_velocity[0]) * (state.velocity[0] - goal_velocity[0]) +
    //                                 (state.velocity[1] - goal_velocity[1]) * (state.velocity[1] - goal_velocity[1]));

    //     // std::cout << " " << vel_error << std::endl;

    //     float reward = 0;
    //     bool terminated = false;
    //     if (collision)
    //     {
    //         reward -= 10;
    //         terminated = true;
    //     }
    //     else if (pos_error < 0.5 && vel_error < 0.25) // ignoring orientation for now
    //     {
    //         reward += 50;
    //         terminated = true;
    //     }
    //     else
    //     {
    //         reward -= (pos_error / 10.0) + (vel_error / 10.0);
    //     }
    //     if (current_step >= max_steps ||
    //         state.position[0] > robot_bridge->x_max || state.position[0] < robot_bridge->x_min ||
    //         state.position[1] > robot_bridge->y_max || state.position[1] < robot_bridge->y_min)
    //     {
    //         terminated = true;
    //     }
    //     return {reward, terminated};
    // }
    float last_pos_error = 0;
    float last_vel_error = 0;
    std::pair<float, bool> computeReward()
    {
        RobotState state = robot_bridge->getRobotState();
        bool collision = robot_bridge->inCollision();

        auto &goal_position         = goal.position;
        auto &goal_orientation      = goal.orientation;
        auto &goal_velocity         = goal.velocity;
        auto &goal_angular_velocity = goal.angular_velocity;

        float cur_pos_error = std::sqrt(std::pow(state.position[0] - g_pos[0], 2) +
                                      std::pow(state.position[1] - g_pos[1], 2));

        float cur_vel_error = std::sqrt(std::pow(state.velocity[0] - g_vel[0], 2) +
                                      std::pow(state.velocity[1] - g_vel[1], 2));

        float reward = 0;
        bool terminated = false;

        if (collision)
        {
            reward = -20.0f;
            terminated = true;
        }
        else if (cur_pos_error < 0.5 && cur_vel_error < 0.25)
        {
            reward = 100.0f;
            terminated = true;
        }
        else
        {
            // moving 1m closer = +5.0 reward
            float pos_progress = (last_pos_error - cur_pos_error) * 5.0f;

            // closing the velocity gap by 1m/s = +10.0 reward
            float vel_progress = (last_vel_error - cur_vel_error) * 10.0f;

            reward = pos_progress + vel_progress - 0.01f;
        }

        if (current_step >= max_steps ||
            state.position[0] > robot_bridge->x_max || state.position[0] < robot_bridge->x_min ||
            state.position[1] > robot_bridge->y_max || state.position[1] < robot_bridge->y_min)
        {
            terminated = true;
        }

        last_pos_error = cur_pos_error;
        last_vel_error = cur_vel_error;

        return {reward, terminated};
    }
};
