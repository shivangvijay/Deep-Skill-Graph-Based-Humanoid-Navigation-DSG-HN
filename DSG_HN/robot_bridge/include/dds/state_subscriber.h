#pragma once
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <vector>
#include <robot_bridge.h>

using namespace unitree::common;
using namespace unitree::robot;

#define JOINT_STATE_TOPIC "rt/lowstate"
#define IMU_STATE_TOPIC "rt/secondary_imu"

#define SPORT_MODE_TOPIC "rt/sportmodestate" //high frequency
#define SPORT_MODE_TOPIC_LF "rt/lf/odommodestate" //low frequency

class StateSubscriber
{
public:
    explicit StateSubscriber() {}
    ~StateSubscriber() = default;
    void Init();
    RobotState GetRobotState();

private:
    ChannelSubscriberPtr<::unitree_hg::msg::dds_::LowState_> joint_state_subscriber;
    ChannelSubscriberPtr<::unitree_go::msg::dds_::SportModeState_> sport_mode_subscriber;
    ChannelSubscriberPtr<::unitree_hg::msg::dds_::IMUState_> imu_state_subscriber; // this IMU is just for the torst, sport state contains the aggregate vel

    RobotState current_state;
    void jointStateCallback(const void *msg);
    void sportModeCallback(const void *msg);
    void imuCallback(const void *msg);
};
