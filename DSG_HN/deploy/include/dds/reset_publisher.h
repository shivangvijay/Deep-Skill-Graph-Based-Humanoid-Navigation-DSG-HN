#pragma once
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/ros2/Pose_.hpp>
#include <vector>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_RESET "reset_simulation"
class ResetPublisher
{
public:
    explicit ResetPublisher() {}
    ~ResetPublisher() = default;
    void Init();
    void publishResetCommand(const std::vector<float> &position, const std::vector<float> &orientation);

private:
    ChannelPublisherPtr<::geometry_msgs::msg::dds_::Pose_> reset_publisher;
};

