#pragma once
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/Twist_.hpp>
#include <vector>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_VEL_COMMANDS "base_velocity_commands"

class VelSubscriber
{
public:
    explicit VelSubscriber() {}
    ~VelSubscriber() = default;
    void Init();
    const std::vector<float>& getVelCmd() const { return vel_cmd; }
private:
    void velCommandCallback(const void *msg);
    ChannelSubscriberPtr<::geometry_msgs::msg::dds_::Twist_> vel_subscriber;
    std::vector<float> vel_cmd = {0.0f, 0.0f, 0.0f};
};
