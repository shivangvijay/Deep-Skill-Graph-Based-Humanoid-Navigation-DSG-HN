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
    enum class TerminationCause
    {
        None,
        GoalReached,
        CollisionOrOutOfBounds,
        Timeout
    };

    static const char *terminationCauseToString(TerminationCause cause)
    {
        switch (cause)
        {
        case TerminationCause::GoalReached:
            return "goal_reached";
        case TerminationCause::CollisionOrOutOfBounds:
            return "collision_or_oob";
        case TerminationCause::Timeout:
            return "timeout";
        default:
            return "none";
        }
    }

    TrainEnvironment(std::shared_ptr<RobotBridgeTrain> robot_bridge_, int max_steps_, bool narrow_map_ = false) : robot_bridge(robot_bridge_), max_steps(max_steps_), narrow_map(narrow_map_)
    {
        RobotState state = robot_bridge->getRobotState();
        obstacles = robot_bridge->getObstacles();

        int proprio_dim = state.q.size() + state.dq.size();
        int gravity_dim = 3;      // local_up
        int self_dyn_dim = 3 + 3; // local_vel, local_ang_vel
        int goal_dim = 3;         // relative_pos (body frame)
        int boundary_dim = 4;
        int last_action_dim = 3; // previous velocity command
        int orientation_dim = 2; // cos and sin of yaw
        obstacle_dim = (int)obstacles.size() * obstacle_feature_dim;

        if (narrow_map)
            state_dim = proprio_dim + gravity_dim + self_dyn_dim + goal_dim + boundary_dim + orientation_dim;
        else
            state_dim = gravity_dim + self_dyn_dim + goal_dim + boundary_dim + last_action_dim;

        std::array<float, 3> pos_scales = {(robot_bridge->x_max - robot_bridge->x_min) / 2, (robot_bridge->y_max - robot_bridge->y_min) / 2, 0.5}; // some of these may need to be tuned later
        std::array<float, 4> orientation_scales = {1, 1, 1, 1};
        std::array<float, 3> vel_scales = {action_scaling_factors[0], action_scaling_factors[1], 1};
        std::array<float, 3> ang_vel_scales = {action_scaling_factors[2], action_scaling_factors[2], action_scaling_factors[2]};

        success_val = (narrow_map) ? 15.0 : 45.0;
        env_scaling_factors = {pos_scales, orientation_scales, vel_scales, ang_vel_scales}; // these contain the scaling factor for each dim in teh env
    }

    torch::Tensor reset() // if no arguments passed, just reset to random position. If goal fixed, use that, else sample randomly
    {
        AbstractedState start;
        int start_attempts = 0;
        do
        {
            start = robot_bridge->generateRandomValidConfiguration();
            float obs_dist = robot_bridge->distanceToNearestObstacle(start.position, start.orientation);
            if (obs_dist >= 1.0f)
                break;
        } while (++start_attempts < 200);
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
                float obs_dist = robot_bridge->distanceToNearestObstacle(goal.position, goal.orientation);
                if (dist >= min_goal_distance && dist <= max_goal_distance && obs_dist >= 0.6f)
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
        prev_obs_dist = robot_bridge->distanceToNearestObstacle(start.position, start.orientation);

        _collision = false;
        _last_termination_cause = TerminationCause::None;
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
        auto a_squeezed = action;
        if (action.dim() > 1)
            a_squeezed = action.squeeze(0);

        auto a = a_squeezed.accessor<float, 1>();
        cmd[0] = a[0]; // more memory safe way of doing this
        cmd[1] = a[1];
        cmd[2] = a[2];

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
        prev_obs_dist = robot_bridge->distanceToNearestObstacle(clamped_pos, quat);
        _collision = false;
        _last_termination_cause = TerminationCause::None;
        return transformState(robot_bridge->getRobotState());
    }

    std::array<float, 3> getGoalPosition() const { return goal.position; }

    std::pair<std::array<float, 3>, std::array<float, 4>> getRobotPose() const
    {
        RobotState s = robot_bridge->getRobotState();
        return {s.position, s.orientation};
    }

    TerminationCause getLastTerminationCause() const { return _last_termination_cause; }

    // Fix goal_position to a specific point (e.g. next skill's subgoal).
    // reset() and resetTo() will not randomize goal_position while fixed.

    void setGoal(const AbstractedState &state)
    {
        goal = state;
        _goal_fixed = true;
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

    void setGoalDistance(float distance)
    {
        max_goal_distance = distance;
    }

    float getMaxGoalDistance() const { return max_goal_distance; }
    void setSuccessRadius(float r) { success_radius = std::max(0.0f, r); }
    float getSuccessRadius() const { return success_radius; }

    void updateGoalMarker(const AbstractedState &goal_)
    {
        robot_bridge->getEngine()->setGoalMarker(goal_.position[0], goal_.position[1], 0.5f);
    }

    void updateGoalMarker()
    {
        robot_bridge->getEngine()->setGoalMarker(goal.position[0], goal.position[1], 0.5f);
    }

    void showObstacleMargins()
    {
        std::vector<MuJoCoEngine::ObstacleMarker> markers;
        for (const auto &obs : obstacles)
            markers.push_back({obs.position[0], obs.position[1], obs.size[0]});
        robot_bridge->getEngine()->setObstacleMarkers(markers, 0.8f);
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

    // use this compute reward if going with the tight map
    std::pair<torch::Tensor, torch::Tensor> computeReward(const RobotState &state, bool collision, const AbstractedState &goal_, bool use_goal_radius = true)
    {
        _last_termination_cause = TerminationCause::None;
        if (narrow_map)
        {
            auto goal_position = goal_.position;

            float pos_error = std::sqrt((state.position[0] - goal_position[0]) * (state.position[0] - goal_position[0]) +
                                        (state.position[1] - goal_position[1]) * (state.position[1] - goal_position[1]));

            float reward = 0;
            bool terminated = false;

            float min_obs_dist = robot_bridge->distanceToNearestObstacle(state.position, state.orientation);
            constexpr float safe_margin = 1.2f;
            if (min_obs_dist < safe_margin)
            {
                reward -= 2.0f * (safe_margin - min_obs_dist);

                float obs_dist_change = min_obs_dist - prev_obs_dist;
                reward += 3.0f * obs_dist_change;
            }

            if (collision || _collision ||
                state.position[0] > robot_bridge->x_max || state.position[0] < robot_bridge->x_min ||
                state.position[1] > robot_bridge->y_max || state.position[1] < robot_bridge->y_min)
            {
                reward -= 100;
                _collision = true;
                terminated = true;
                _last_termination_cause = TerminationCause::CollisionOrOutOfBounds;
            }
            else if (use_goal_radius && pos_error < success_radius)
            {
                reward += 250;
                terminated = true;
                _last_termination_cause = TerminationCause::GoalReached;
            }
            else
            {
                float dist_reduction = prev_dist_to_goal - pos_error;
                reward += 5.0f * dist_reduction;

                reward -= 0.1f;

                float action_jerk = std::abs(last_action[0] - prev_action[0]) +
                                    std::abs(last_action[1] - prev_action[1]) +
                                    std::abs(last_action[2] - prev_action[2]);
                reward -= 0.5f * action_jerk;
            }

            if (!terminated && current_step >= max_steps)
            {
                reward -= 10;
                terminated = true;
                _last_termination_cause = TerminationCause::Timeout;
            }

            prev_dist_to_goal = pos_error;
            prev_obs_dist = min_obs_dist;
            reward /= 10;

            return {torch::tensor({reward}, torch::kFloat32),
                    torch::tensor({(float)terminated}, torch::kFloat32)};
        }
        else
        {
            auto goal_position = goal_.position;

            float pos_error = std::sqrt((state.position[0] - goal_position[0]) * (state.position[0] - goal_position[0]) +
                                        (state.position[1] - goal_position[1]) * (state.position[1] - goal_position[1]));

            float reward = 0;
            bool terminated = false;

            float min_obs_dist = robot_bridge->distanceToNearestObstacle(state.position, state.orientation);
            constexpr float safe_margin = 1.2f;
            if (min_obs_dist < safe_margin)
            {
                reward -= 2.0f * (safe_margin - min_obs_dist);

                float obs_dist_change = min_obs_dist - prev_obs_dist;
                reward += 3.0f * obs_dist_change;
            }

            if (collision || _collision ||
                state.position[0] > robot_bridge->x_max || state.position[0] < robot_bridge->x_min ||
                state.position[1] > robot_bridge->y_max || state.position[1] < robot_bridge->y_min)
            {
                reward -= 30;
                _collision = true;
                terminated = true;
                _last_termination_cause = TerminationCause::CollisionOrOutOfBounds;
            }
            else if (use_goal_radius && pos_error < success_radius)
            {
                reward += 50;
                terminated = true;
                _last_termination_cause = TerminationCause::GoalReached;
            }
            else
            {
                float dist_reduction = prev_dist_to_goal - pos_error;
                reward += 5.0f * dist_reduction;

                reward -= 0.1f;

                float action_jerk = std::abs(last_action[0] - prev_action[0]) +
                                    std::abs(last_action[1] - prev_action[1]) +
                                    std::abs(last_action[2] - prev_action[2]);
                reward -= 0.5f * action_jerk;
            }

            if (!terminated && current_step >= max_steps)
            {
                reward -= 10;
                terminated = true;
                _last_termination_cause = TerminationCause::Timeout;
            }

            prev_dist_to_goal = pos_error;
            prev_obs_dist = min_obs_dist;

            return {torch::tensor({reward}, torch::kFloat32),
                    torch::tensor({(float)terminated}, torch::kFloat32)};
        }
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

        if (narrow_map)
        {
            copy_to_ptr(state.q);
            std::array<float, DOF> dq_scaled;

            for (int j = 0; j < DOF; j++)
            {
                dq_scaled[j] = state.dq[j] / 20; // dq scaling factor
            }
            copy_to_ptr(dq_scaled);
        }

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

        copy_to_ptr(rotateVectorByQuat(r_pos, state.orientation, true)); // relative_pos (3)

        // relative distance to boundary
        float dist_left = state.position[0] - robot_bridge->x_min;
        float dist_right = robot_bridge->x_max - state.position[0];
        float dist_back = state.position[1] - robot_bridge->y_min;
        float dist_front = robot_bridge->y_max - state.position[1];

        data_ptr[offset++] = dist_left / env_scaling_factors.position[0];
        data_ptr[offset++] = dist_right / env_scaling_factors.position[0];
        data_ptr[offset++] = dist_back / env_scaling_factors.position[1];
        data_ptr[offset++] = dist_front / env_scaling_factors.position[1];

        if (narrow_map)
        {
            for (int i = 0; i < obstacles.size(); i++)
            {
                const auto &obs = obstacles[i];

                std::array<float, 3> r_obs = {obs.position[0] - state.position[0],
                                              obs.position[1] - state.position[1],
                                              0.0f}; // Keep it 2D for simplicity

                float dist_to_center = std::sqrt(r_obs[0] * r_obs[0] + r_obs[1] * r_obs[1]);

                // calculate vector to the NEAREST EDGE
                // (r_obs / dist_to_center) is the unit direction to the obstacle
                float dist_to_edge = dist_to_center - obs.size[0]; // size[0] is radius

                std::array<float, 3> r_edge = {
                    (r_obs[0] / dist_to_center) * dist_to_edge,
                    (r_obs[1] / dist_to_center) * dist_to_edge,
                    0.0f};

                auto local_edge = rotateVectorByQuat(r_edge, state.orientation, true);

                // 4. Normalize and Store
                data_ptr[offset++] = local_edge[0] / env_scaling_factors.position[0];
                data_ptr[offset++] = local_edge[1] / env_scaling_factors.position[1];
                data_ptr[offset++] = dist_to_edge / 1.0f; // Explicit "Danger" signal
                data_ptr[offset++] = obs.size[0] / 0.5f;  // Scaled radius (should be ~1.0)
            }
            float qw = state.orientation[0], qx = state.orientation[1], qy = state.orientation[2], qz = state.orientation[3];
            float yaw = std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
            data_ptr[offset++] = std::cos(yaw);
            data_ptr[offset++] = std::sin(yaw);
        }
        else
        {
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
            copy_to_ptr(last_action);
        }

        return tensor_state;
    }

    int state_dim;
    int obstacle_dim;
    int obstacle_feature_dim = 4;
    int action_dim = 3;
    std::vector<float> action_scaling_factors = {0.75, 0.3, 1.0};
    std::vector<float> action_shift_factors = {0.25, 0.0, 0.0};
    AbstractedState env_scaling_factors;
    bool narrow_map = false; // flag that helps determine which reward and transform state func needed
    float success_val = 45.0f; // threshold for reward, at which you know traj was a success

    float max_goal_distance = 3.0f;
    float min_goal_distance = 1.0f;
    float success_radius = 0.5f;


    const std::vector<Obstacle> &getObstacles() const { return obstacles; }

    float distanceToNearestObstacle() const
    {
        RobotState s = robot_bridge->getRobotState();
        return robot_bridge->distanceToNearestObstacle(s.position, s.orientation);
    }

    float distanceToNearestObstacle(const std::array<float, 3> &pos, const std::array<float, 4> &orient) const
    {
        return robot_bridge->distanceToNearestObstacle(pos, orient);
    }

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
    float prev_obs_dist = 10.0f;
    bool _collision = false;
    TerminationCause _last_termination_cause = TerminationCause::None;

    float _euclidean2D(const std::array<float, 3> &a, const std::array<float, 3> &b) const
    {
        float dx = a[0] - b[0];
        float dy = a[1] - b[1];
        return std::sqrt(dx * dx + dy * dy);
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
