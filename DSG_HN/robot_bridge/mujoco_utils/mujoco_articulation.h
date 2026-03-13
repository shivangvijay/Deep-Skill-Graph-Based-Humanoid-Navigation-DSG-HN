#pragma once
#include "mujoco_engine.h"       // The class we just wrote
#include "isaaclab/assets/articulation/articulation.h"
#include "unitree/idl/hg/LowState_.hpp"

namespace unitree
{

    class MuJoCoArticulation : public isaaclab::Articulation
    {
    public:
        MuJoCoArticulation(std::shared_ptr<MuJoCoEngine> engine)
            : engine_(engine)
        {
            _init_sensor_addresses();

            mjModel *m = engine_->GetModel();
        }

        void update()
        {
            mjModel *m = engine_->GetModel();
            mjData *d = engine_->GetData();

            // step the sim before before processing the new data. Note that when we go to do the training loop, will need
            // to set the time and frequency of calls to mirror the Hz at which this policy is called, and at which we want to call the vel policy
            // engine_->Step(); Probably best if this is in the main loop

            // keep the msg here so that I am doubly sure that I am copying things into the format that the observations expect
            ::unitree_hg::msg::dds_::LowState_ msg;

            int num_motor = m->nu;
            for (int i = 0; i < num_motor; i++)
            {
                msg.motor_state()[i].q() = d->sensordata[i];
                msg.motor_state()[i].dq() = d->sensordata[i + num_motor];
            }

            if (imu_quat_adr_ >= 0)
            {
                for (int i = 0; i < 4; i++)
                    msg.imu_state().quaternion()[i] = d->sensordata[imu_quat_adr_ + i];
            }
            if (imu_gyro_adr_ >= 0)
            {
                for (int i = 0; i < 3; i++)
                    msg.imu_state().gyroscope()[i] = d->sensordata[imu_gyro_adr_ + i];
            }

            for (int i(0); i < 3; i++)
            {
                data.root_ang_vel_b[i] = msg.imu_state().gyroscope()[i];
            }

            // project_gravity_body
            data.root_quat_w = Eigen::Quaternionf(
                msg.imu_state().quaternion()[0],
                msg.imu_state().quaternion()[1],
                msg.imu_state().quaternion()[2],
                msg.imu_state().quaternion()[3]);

            data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;
            // joint positions and velocities
            for (int i(0); i < data.joint_ids_map.size(); i++)
            {
                data.joint_pos[i] = msg.motor_state()[data.joint_ids_map[i]].q();
                data.joint_vel[i] = msg.motor_state()[data.joint_ids_map[i]].dq();
            }
        }

    private:
        std::shared_ptr<MuJoCoEngine> engine_;
        int imu_quat_adr_ = -1;
        int imu_gyro_adr_ = -1;

        void _init_sensor_addresses()
        {
            mjModel *m = engine_->GetModel();
            int q_id = mj_name2id(m, mjOBJ_SENSOR, "imu_quat");
            if (q_id >= 0)
                imu_quat_adr_ = m->sensor_adr[q_id];

            int g_id = mj_name2id(m, mjOBJ_SENSOR, "imu_gyro");
            if (g_id >= 0)
                imu_gyro_adr_ = m->sensor_adr[g_id];
        }
    };

} // namespace unitree