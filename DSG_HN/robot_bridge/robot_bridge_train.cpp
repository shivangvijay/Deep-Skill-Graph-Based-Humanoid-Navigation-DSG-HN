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
    initSensorAddresses();
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
    mjModel *m = eng->getModel();
    mjData *d = eng->getData();

    // keep the msg here so that I am doubly sure that I am copying things into the format that the observations expect
    ::unitree_hg::msg::dds_::IMUState_ imu_msg;

    int num_motor = m->nu;
    for (int i = 0; i < num_motor; i++)
    {
        s.q[i] = d->sensordata[i];
        s.dq[i] = d->sensordata[i + num_motor];
    }

    // kinda odd, but to get imu data, need to use secondary imu, which is the imu state of the torso
    if (imu_quat_adr >= 0)
    {
        s.orientation[0] = d->sensordata[imu_quat_adr + 0];
        s.orientation[1] = d->sensordata[imu_quat_adr + 1];
        s.orientation[2] = d->sensordata[imu_quat_adr + 2];
        s.orientation[3] = d->sensordata[imu_quat_adr + 3];
    }

    if (imu_gyro_adr >= 0)
    {
        s.angular_velocity[0] = d->sensordata[imu_gyro_adr + 0];
        s.angular_velocity[1] = d->sensordata[imu_gyro_adr + 1];
        s.angular_velocity[2] = d->sensordata[imu_gyro_adr + 2];
    }

    if (imu_accel_adr >= 0)
    {
        s.accel[0] = d->sensordata[imu_accel_adr + 0];
        s.accel[1] = d->sensordata[imu_accel_adr + 1];
        s.accel[2] = d->sensordata[imu_accel_adr + 2];
    }

    if (frame_pos_adr >= 0)
    {
        s.position[0] = d->sensordata[frame_pos_adr + 0];
        s.position[1] = d->sensordata[frame_pos_adr + 1];
        s.position[2] = d->sensordata[frame_pos_adr + 2];
    }

    if (frame_vel_adr >= 0)
    {
        s.velocity[0] = d->sensordata[frame_vel_adr + 0];
        s.velocity[1] = d->sensordata[frame_vel_adr + 1];
        s.velocity[2] = d->sensordata[frame_vel_adr + 2];
    }

    return s;
}
void RobotBridgeTrain::resetRobot(const std::array<float, 3> &pos, const std::array<float, 4> &quat)
{
    eng->reset(pos, quat);
    env->reset();
}

void RobotBridgeTrain::initSensorAddresses()
{
    mjModel *m = eng->getModel();

    // Secondary IMU quaternion
    int q_id = mj_name2id(m, mjOBJ_SENSOR, "secondary_imu_quat");
    if (q_id >= 0)
    {
        imu_quat_adr = m->sensor_adr[q_id];
    }

    // Secondary IMU gyroscope
    int g_id = mj_name2id(m, mjOBJ_SENSOR, "secondary_imu_gyro");
    if (g_id >= 0)
    {
        imu_gyro_adr = m->sensor_adr[g_id];
    }

    // Secondary IMU accelerometer
    int a_id = mj_name2id(m, mjOBJ_SENSOR, "secondary_imu_acc");
    if (a_id >= 0)
    {
        imu_accel_adr = m->sensor_adr[a_id];
    }

    // Frame position
    int fp_id = mj_name2id(m, mjOBJ_SENSOR, "frame_pos");
    if (fp_id >= 0)
    {
        frame_pos_adr = m->sensor_adr[fp_id];
    }

    // Frame velocity
    int fv_id = mj_name2id(m, mjOBJ_SENSOR, "frame_vel");
    if (fv_id >= 0)
    {
        frame_vel_adr = m->sensor_adr[fv_id];
    }
}

bool RobotBridgeTrain::inCollision()
{
    return eng->inCollision();
}