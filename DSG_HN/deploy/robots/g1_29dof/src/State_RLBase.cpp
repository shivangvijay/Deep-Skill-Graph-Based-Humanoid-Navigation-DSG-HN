#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "dds/vel_subscriber.h"
#include "dds/vel_publisher.h"
namespace isaaclab
{
    // keyboard velocity commands example
    // change "velocity_commands" observation name in policy deploy.yaml to "keyboard_velocity_commands"
    REGISTER_OBSERVATION(keyboard_velocity_commands)
    {
        static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

        // Physical range limits (read once from deploy.yaml)
        static const float VX_MAX  =  cfg["lin_vel_x"][1].as<float>();  //  1.0 m/s
        static const float VX_MIN  =  cfg["lin_vel_x"][0].as<float>();  // -0.5 m/s
        static const float VY_MAX  =  cfg["lin_vel_y"][1].as<float>();  //  0.3 m/s
        static const float VY_MIN  =  cfg["lin_vel_y"][0].as<float>();  // -0.3 m/s
        static const float YAW_MAX =  cfg["ang_vel_z"][1].as<float>();  //  0.2 rad/s
        static const float YAW_MIN =  cfg["ang_vel_z"][0].as<float>();  // -0.2 rad/s

        // Map single key to velocity command within policy training ranges
        auto k = FSMState::keyboard->key();
        float vx = 0.f, vy = 0.f, yaw = 0.f;
        if      (k == "w") vx  = VX_MAX;
        else if (k == "s") vx  = VX_MIN;
        else if (k == "a") vy  = VY_MAX;
        else if (k == "d") vy  = VY_MIN;
        else if (k == "q") yaw = YAW_MAX;
        else if (k == "e") yaw = YAW_MIN;

        std::vector<float> cmd = {vx, vy, yaw};

        // Mirror command to DDS so external processes (e.g. collect_transitions) can observe it
        static VelPublisher kb_vel_pub = []() {
            VelPublisher p; p.Init(); return p;
        }();
        kb_vel_pub.publishVelCommand(cmd);

        return cmd;
    }

    // TODO: need to add a new interface that can recieve vel command messages via DDS and then use that to inform the policy
    // use dds commands that are available in the G1

    // steps: 
    // 1: define new DDS topic and message type for vel commands
    // 2: add a DDS subscriber here to listen to the vel commands and send them
    // 3: create new function that can publish vel commands on that topic
    // 4: looks like there are some existing publishers, need to just find topics and subscribe to get robot state (do this in higher level control block)

    // 5: to add reset logic, create new publisher that is subscribed to by something inside the sim loop, that can then trigger a reset of the sim
    REGISTER_OBSERVATION(dsg_velocity_commands)
    {
        static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

        const auto &vel_cmd = FSMState::vel_subscriber->getVelCmd();
        // std::cout << "Received vel command: " << vel_cmd[0] << " " << vel_cmd[1] << " " << vel_cmd[2] << std::endl;
        //std::vector<float> cmd = {0.0f, 0.0f, 1.0f};
        return vel_cmd;

    }
}

// believe that issue with robot flopping is the fact that there is a delay when the sim is started, where the sim runs but it does not recieve
// any DDS commands. Need to add reset functionality in sim, that allows for spawning, and waits for the controller to be connected before proceeding

// The above is the big task for tomorrow
// what you may need to do is add a flag to the main sim loop that is set to true once DDS messages are being recieved, and only
// allow physics to be stepped after that happens. For resetting, add DDS subscriber that listens for a reset message, and when that message
// is recieved, set the sim to the new state
State_RLBase::State_RLBase(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    std::cout << cfg["policy_dir"] << std::endl;

    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate));
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    // this->registered_checks.emplace_back(
    //     std::make_pair(
    //         [&]() -> bool
    //         { return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
    //         FSMStringMap.right.at("Passive")));
}

void State_RLBase::run()
{
    if (FSMState::reset_subscriber->reset) {
        env->reset();
        FSMState::reset_subscriber->reset = false;
        return;
    }

    auto action = env->action_manager->processed_actions();
    
    for (int i(0); i < env->robot->data.joint_ids_map.size(); i++)
    {
        action[i] = std::clamp(action[i], -1.0f, 1.0f);
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }

}