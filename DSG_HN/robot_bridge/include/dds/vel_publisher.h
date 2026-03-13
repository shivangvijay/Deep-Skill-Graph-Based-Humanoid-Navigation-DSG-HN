#pragma once
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/ros2/Twist_.hpp>
#include <vector>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_VEL_COMMANDS "base_velocity_commands"
class VelPublisher
{
public:
    explicit VelPublisher() {}
    ~VelPublisher() = default;
    void Init();
    void publishVelCommand(const std::vector<float> &cmd);

private:
    ChannelPublisherPtr<::geometry_msgs::msg::dds_::Twist_> vel_publisher;
};

