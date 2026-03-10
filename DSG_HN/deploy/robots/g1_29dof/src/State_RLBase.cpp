#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "dds/vel_subscriber.h"
#include <unordered_map>

namespace isaaclab
{
    // keyboard velocity commands example
    // change "velocity_commands" observation name in policy deploy.yaml to "keyboard_velocity_commands"
    REGISTER_OBSERVATION(keyboard_velocity_commands)
    {
        std::string key = FSMState::keyboard->key();
        static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

        static std::unordered_map<std::string, std::vector<float>> key_commands = {
            {"w", {1.0f, 0.0f, 0.0f}},
            {"s", {-1.0f, 0.0f, 0.0f}},
            {"a", {0.0f, 1.0f, 0.0f}},
            {"d", {0.0f, -1.0f, 0.0f}},
            {"q", {0.0f, 0.0f, 1.0f}},
            {"e", {0.0f, 0.0f, -1.0f}}};
        std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
        if (key_commands.find(key) != key_commands.end())
        {
            // TODO: smooth and limit the velocity commands
            cmd = key_commands[key];
        }
        // std::cout << cmd[0] << " " << cmd[1] << " " << cmd[2] << std::endl;
        return cmd;
    }

    // TODO: need to add a new interface that can recieve vel command messages via DDS and then use that to inform the policy
    // use dds commands that are available in the G1

    // steps: 
    // 1: define new DDS topic and message type for vel commands
    // 2: add a DDS subscriber here to listen to the vel commands and send them
    // 3: create new function that can publish vel commands on that topic
    // 4: looks like there are some existing publishers, need to just find topics and subscribe to get robot state (do this in higher level control block)
    REGISTER_OBSERVATION(dds_commands)
    {
        static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

        const auto &vel_cmd = FSMState::vel_subscriber->getVelCmd();
        // std::cout << "Received vel command: " << vel_cmd[0] << " " << vel_cmd[1] << " " << vel_cmd[2] << std::endl;
        //std::vector<float> cmd = {0.0f, 0.0f, 1.0f};
        return vel_cmd;
    }
}


State_RLBase::State_RLBase(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
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
    auto action = env->action_manager->processed_actions();
    // std::cout << "RLBase action: ";
    // for (const auto &a : action)
    // {
    //     std::cout << a << " ";
    // }
    // std::cout << std::endl;
    for (int i(0); i < env->robot->data.joint_ids_map.size(); i++)
    {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}