#include "robot_bridge.h"
#include "mujoco_utils/mujoco_articulation.h"
#include "mujoco_utils/mujoco_engine.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

#define LOCMOTION_POLICY_DT 0.02

class RobotBridgeTrain : public RobotBridge
{
public:
    // Pass in your existing env and engine pointers. TODO: unify this and other RobotBridge by just passing in vm
    RobotBridgeTrain(std::string scene_file, float x_min, float x_max, float y_min, float y_max, std::shared_ptr<MuJoCoEngine> eng, std::unique_ptr<isaaclab::ManagerBasedRLEnv> env, bool render);

    virtual ~RobotBridgeTrain() = default;

    void publishVelCommand(const std::vector<float> &cmd) override;
    using RobotBridge::resetRobot;
    void resetRobot(const std::array<float, 3> &pos, const std::array<float, 4> &quat) override;
    RobotState getRobotState() override;
    void update() override;
    bool inCollision();

private:
    std::shared_ptr<MuJoCoEngine> eng;
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;

    int num_motors;
    float sim_dt;
    bool render;
    int imu_quat_adr;
    int imu_gyro_adr;
    int imu_accel_adr;
    int frame_pos_adr;
    int frame_vel_adr;

    void initSensorAddresses();
};