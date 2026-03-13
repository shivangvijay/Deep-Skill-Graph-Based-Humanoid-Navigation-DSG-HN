#include "robot_bridge_train.h"

static RobotBridgeTrain *ptr = nullptr;

namespace isaaclab
{
    REGISTER_OBSERVATION(dsg_velocity_commands)
    {
        if (ptr == nullptr)
            return {0.0, 0.0, 0.0};
        return ptr->GetCurrentCmd();
    }
}

RobotBridgeTrain::RobotBridgeTrain(std::string scene_file, float x_min, float x_max, float y_min, float y_max, std::shared_ptr<MuJoCoEngine> eng_, std::unique_ptr<isaaclab::ManagerBasedRLEnv> env_, bool render_)
    : RobotBridge(scene_file, x_min, x_max, y_min, y_max), eng(eng_), env(std::move(env_)), render(render_)
{
    ptr = this;
    num_motors = eng->getModel()->nu;
    sim_dt = eng->getModel()->opt.timestep;
}

void RobotBridgeTrain::publishVelCommand(const std::vector<float> &cmd)
{
    current_cmd = cmd;
}

void RobotBridgeTrain::update()
{
    int vel_policy_decimation = static_cast<int>(velocity_policy_dt / LOCMOTION_POLICY_DT);
    for (int i = 0; i < vel_policy_decimation; i++)
    {
        int low_level_decimation = static_cast<int>(LOCMOTION_POLICY_DT / sim_dt);
        env->step();
        auto action = env->action_manager->processed_actions();
        std::vector<float> target_q(num_motors, 0.0);
        for (int i = 0; i < env->robot->data.joint_ids_map.size(); i++)
        {
            int motor_idx = env->robot->data.joint_ids_map[i];
            target_q[motor_idx] = action[i];
        }

        for (int t = 0; t < low_level_decimation; t++)
        {
            auto d = eng->getData();
            std::vector<double> current_torques(num_motors, 0.0);
            for (int i = 0; i < num_motors; i++)
            {
                auto kp = env->robot->data.joint_stiffness[i];
                auto kd = env->robot->data.joint_damping[i];

                double cur_q = d->sensordata[i];
                double cur_dq = d->sensordata[i + num_motors];

                current_torques[i] = kp * (target_q[i] - cur_q) + kd * (0.0 - cur_dq);
            }

            eng->setControl(current_torques.data());
            eng->step(); // Advance physics by 0.002s
        }
        if (render)
        {
            eng->render();
            std::this_thread::sleep_for(std::chrono::milliseconds(int(LOCMOTION_POLICY_DT * 1000)));
        }
    }
}

RobotState RobotBridgeTrain::getRobotState()
{
    RobotState s;
    return s;
}
void RobotBridgeTrain::resetRobot(const std::array<float, 3> &pos, const std::array<float, 4> &quat)
{
    eng->reset(pos, quat);
    env->reset();
}