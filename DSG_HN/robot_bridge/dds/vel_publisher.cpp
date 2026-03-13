#include "dds/vel_publisher.h"
void VelPublisher::Init()
{
    std::cout << "Initializing VelPublisher..." << std::endl;
    vel_publisher.reset(new ChannelPublisher<::geometry_msgs::msg::dds_::Twist_>(TOPIC_VEL_COMMANDS));
    vel_publisher->InitChannel();
    std::cout << "VelPublisher initialized and ready to publish on topic: " << TOPIC_VEL_COMMANDS << std::endl;
}

void VelPublisher::publishVelCommand(const std::vector<float> &cmd)
{
    ::geometry_msgs::msg::dds_::Twist_ twist_msg;
    twist_msg.linear().x(cmd[0]);
    twist_msg.linear().y(cmd[1]);
    twist_msg.angular().z(cmd[2]);
    vel_publisher->Write(twist_msg);
}