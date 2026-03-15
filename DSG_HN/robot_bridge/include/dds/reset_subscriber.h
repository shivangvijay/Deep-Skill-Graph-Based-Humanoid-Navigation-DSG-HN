#pragma once
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/Pose_.hpp>
#include <vector>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_RESET "reset_simulation"
class ResetSubscriber
{
public:
    explicit ResetSubscriber() {}
    ~ResetSubscriber() = default;
    void Init();

    bool reset = false;

private:
    ChannelSubscriberPtr<::geometry_msgs::msg::dds_::Pose_> reset_subscriber;
    std::array<float, 7> reset_pose; // x, y, z, qw, qx, qy, qz
    
    void reset_callback(const void *msg);
};

