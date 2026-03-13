#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "mujoco_utils/mujoco_articulation.h"
#include "mujoco_utils/mujoco_engine.h"
#include <string>
#include "param.h"
#include <algorithm>
#include "robot_bridge_train.h"

#define SCENE_FILE "ai_maker_space_scene.xml"
#define POLICY_DIR "config/policy/velocity"
#define CONFIG_PATH "config/config.yaml"

#define X_MIN -2.0f
#define X_MAX 2.0f
#define Y_MIN -2.0f
#define Y_MAX 2.0f

std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;


int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    bool render = true;
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);
    auto eng = std::make_shared<MuJoCoEngine>(render);
    eng->initialize(SCENE_FILE);

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::MuJoCoArticulation>(eng));
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    RobotBridgeTrain robot_bridge(SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, eng, std::move(env), true);

    std::vector<float> cmd = {0.0, 1.0, 0.0};
    int step = 0;

    while (true)
    {
        if (step % 100 == 0) 
        {
            robot_bridge.resetRobot();
            robot_bridge.update();
        }
        robot_bridge.publishVelCommand(cmd);
        robot_bridge.update();
        step++;
    }
    // int num_motors = eng->GetModel()->nu;
    // auto sim_dt = eng->GetModel()->opt.timestep;
    // int decimation = static_cast<int>(POLICY_STEP_DT / sim_dt);

    // while (!render || eng->IsWindowOpen())
    // {
    //     env->step();
    //     auto action = env->action_manager->processed_actions();

    //     std::vector<float> target_q(num_motors, 0.0);
    //     for (int i = 0; i < env->robot->data.joint_ids_map.size(); i++)
    //     {
    //         int motor_idx = env->robot->data.joint_ids_map[i];
    //         target_q[motor_idx] = action[i];
    //     }

    //     for (int t = 0; t < decimation; t++)
    //     {
    //         auto d = eng->GetData();
    //         std::vector<double> current_torques(num_motors, 0.0);

    //         for (int i = 0; i < num_motors; i++)
    //         {
    //             auto kp = env->robot->data.joint_stiffness[i];
    //             auto kd = env->robot->data.joint_damping[i];

    //             double cur_q = d->sensordata[i];
    //             double cur_dq = d->sensordata[i + num_motors];

    //             current_torques[i] = kp * (target_q[i] - cur_q) + kd * (0.0 - cur_dq);
    //         }

    //         eng->SetControl(current_torques.data());
    //         eng->Step(); // Advance physics by 0.002s
    //     }

    //     if (render)
    //         eng->Render();
    // }
}