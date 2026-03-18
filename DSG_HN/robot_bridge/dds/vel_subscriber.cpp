#include "dds/vel_subscriber.h"
void VelSubscriber::velCommandCallback(const void *msg)
{
    const auto *twist_msg = static_cast<const ::geometry_msgs::msg::dds_::Twist_ *>(msg);
    vel_cmd[0] = twist_msg->linear().x();
    vel_cmd[1] = twist_msg->linear().y();
    vel_cmd[2] = twist_msg->angular().z();
    //std::cout << "Received vel command: " << vel_cmd[0] << " " << vel_cmd[1] << " " << vel_cmd[2] << std::endl;
}

void VelSubscriber::Init()
{
    std::cout << "Initializing VelSubscriber..." << std::endl;
    vel_subscriber.reset(new ChannelSubscriber<::geometry_msgs::msg::dds_::Twist_>(TOPIC_VEL_COMMANDS));
    vel_subscriber->InitChannel(std::bind(&VelSubscriber::velCommandCallback, this, std::placeholders::_1));
    std::cout << "VelSubscriber initialized and subscribed to topic: " << TOPIC_VEL_COMMANDS << std::endl;
}