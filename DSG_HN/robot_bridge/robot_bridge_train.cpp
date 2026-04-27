#include "robot_bridge_train.h"

static RobotBridgeTrain *ptr = nullptr;

namespace isaaclab
{
    REGISTER_OBSERVATION(dsg_velocity_commands)
    {
        if (ptr == nullptr)
            return {0.0, 0.0, 0.0};
        return ptr->getCurrentCmd();
    }
}

RobotBridgeTrain::RobotBridgeTrain(std::string scene_file, float x_min, float x_max, float y_min, float y_max, std::filesystem::path policy_dir, bool render_) // std::shared_ptr<MuJoCoEngine> eng_, std::unique_ptr<isaaclab::ManagerBasedRLEnv> env_, bool render_)
    : RobotBridge(scene_file, x_min, x_max, y_min, y_max), render(render_)
{
    ptr = this;
    eng = std::make_shared<MuJoCoEngine>(render);
    eng->initialize(scene_file);

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::MuJoCoArticulation>(eng));
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    d_scratch = mj_makeData(eng->getModel());
    num_motors = eng->getModel()->nu;
    sim_dt = eng->getModel()->opt.timestep;
    initSensorAddresses();
}

RobotBridgeTrain::~RobotBridgeTrain()
{
    if (d_scratch)
        mj_deleteData(d_scratch);
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
            target_q[motor_idx] = std::clamp(action[i], -1.0f, 1.0f);
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
            if (render_realtime)
                std::this_thread::sleep_for(std::chrono::milliseconds(int(LOCMOTION_POLICY_DT * 1000)));
        }
    }
}

AbstractedState RobotBridgeTrain::generateRandomValidConfiguration()
{
    int attempts = 0;
    std::array<float, 3> pos;
    std::array<float, 4> quat;
    std::array<float, 3> vel;
    std::array<float, 3> ang_vel;

    bool found = false;
    while (attempts++ < 100)
    {
        std::tie(pos, quat, vel, ang_vel) = generateRandomPoseWithVel();
        
        if (isConfigurationValid(pos, quat, vel, ang_vel)) {
            found = true;
            break;
        }
    }

    if (!found)
        throw std::runtime_error("Could Not Generate Valid Random Configuration");

    return {pos, quat, vel, ang_vel};
}

bool RobotBridgeTrain::isConfigurationValid(const AbstractedState &state)
{
    return isConfigurationValid(state.position, state.orientation, state.velocity, state.angular_velocity);
}

bool RobotBridgeTrain::isConfigurationValid(const std::array<float, 3> &pos, const std::array<float, 4> &quat, const std::array<float, 3> &vel, const std::array<float, 3> &ang_vel)
{
    if (pos[0] < x_min || pos[0] > x_max || pos[1] < y_min || pos[1] > y_max)
        return false;

    if (distanceToNearestObstacle(pos, quat) < min_spawn_distance_from_obstacles)
        return false;

    mjModel *m = eng->getModel();

    mj_resetData(m, d_scratch);
    
    d_scratch->qpos[0] = pos[0]; d_scratch->qpos[1] = pos[1]; d_scratch->qpos[2] = pos[2];
    d_scratch->qpos[3] = quat[0]; d_scratch->qpos[4] = quat[1]; 
    d_scratch->qpos[5] = quat[2]; d_scratch->qpos[6] = quat[3];
    
    d_scratch->qvel[0] = vel[0]; d_scratch->qvel[1] = vel[1]; d_scratch->qvel[2] = vel[2];
    d_scratch->qvel[3] = ang_vel[0]; d_scratch->qvel[4] = ang_vel[1]; d_scratch->qvel[5] = ang_vel[2];

    mj_forward(m, d_scratch);

    for (int i = 0; i < d_scratch->ncon; i++)
    {
        mjContact *contact = &d_scratch->contact[i];

        int geom1 = contact->geom1;
        int geom2 = contact->geom2;

        std::string name1 = mj_id2name(m, mjOBJ_GEOM, geom1) ? mj_id2name(m, mjOBJ_GEOM, geom1) : "";
        std::string name2 = mj_id2name(m, mjOBJ_GEOM, geom2) ? mj_id2name(m, mjOBJ_GEOM, geom2) : "";

        // want to only ignore collisions with the floor/ground on the
        if (name1 != "floor" && name1 != "ground" &&
            name2 != "floor" && name2 != "ground")
        {
            return false;
        }
    }

    return true;
}

RobotState RobotBridgeTrain::getRobotState()
{
    RobotState s;
    mjModel *m = eng->getModel();
    mjData *d = eng->getData();

    int num_motor = m->nu;
    for (int i = 0; i < num_motor; i++)
    {
        s.q[i] = d->sensordata[i];
        s.dq[i] = d->sensordata[i + num_motor];
    }

    for (int i = 0; i < DOF - num_motor; i++)
    {
        s.q[i + num_motor] = 0.0;
        s.dq[i + num_motor] = 0.0;
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
    current_cmd = {0.0, 0.0, 0.0};
    
    // Settle the robot by stepping with zero velocity commands
    for (int i = 0; i < 2; i++)
    {
        update();
    }
}

void RobotBridgeTrain::resetRobot(const std::array<float, 3> &pos, const std::array<float, 4> &quat, const std::array<float, 3> &vel, const std::array<float, 3> &ang_vel)
{
    eng->reset(pos, quat, vel, ang_vel);
    env->reset();
    current_cmd = {0.0, 0.0, 0.0};
    
    // Settle the robot by stepping with zero velocity commands
    // Kinda assuming spawning with 0 velocity right now, but in the future may want to change this
    // if required to spawn with non-zero velocity
    for (int i = 0; i < 2; i++)
    {
        update();
    }
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

void RobotBridgeTrain::startRender()
{
    if (render)
        return;
    render = true;
    // Keep MuJoCoEngine render state in sync when rendering is enabled after construction.
    eng->render_m = true;
    eng->initViz();
}
