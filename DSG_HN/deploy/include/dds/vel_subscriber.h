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

private:
    void velCommandCallback(const void *msg);
    ChannelSubscriberPtr<::geometry_msgs::msg::dds_::Twist_> vel_subscriber;

    std::vector<float> vel_cmd = {0.0f, 0.0f, 0.0f};
};

void VelSubscriber::velCommandCallback(const void *msg)
{
    const auto *twist_msg = static_cast<const ::geometry_msgs::msg::dds_::Twist_ *>(msg);
    vel_cmd[0] = twist_msg->linear().x();
    vel_cmd[1] = twist_msg->linear().y();
    vel_cmd[2] = twist_msg->angular().z();
    // std::cout << "Received vel command: " << vel_cmd[0] << " " << vel_cmd[1] << " " << vel_cmd[2] << std::endl;
}

void VelSubscriber::Init()
{
    std::cout << "Initializing VelSubscriber..." << std::endl;
    vel_subscriber.reset(new ChannelSubscriber<::geometry_msgs::msg::dds_::Twist_>(TOPIC_VEL_COMMANDS));
    vel_subscriber->InitChannel(std::bind(&VelSubscriber::velCommandCallback, this, std::placeholders::_1));
    std::cout << "VelSubscriber initialized and subscribed to topic: " << TOPIC_VEL_COMMANDS << std::endl;
}