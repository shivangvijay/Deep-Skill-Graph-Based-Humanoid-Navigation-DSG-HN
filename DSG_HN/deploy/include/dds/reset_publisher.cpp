#include "reset_publisher.h"

void ResetPublisher::Init()
{
    std::cout << "Initializing ResetPublisher..." << std::endl;
    reset_publisher.reset(new ChannelPublisher<::geometry_msgs::msg::dds_::Pose_>(TOPIC_RESET));
    reset_publisher->InitChannel();
    std::cout << "ResetPublisher initialized and ready to publish on topic: " << TOPIC_RESET << std::endl;
}

void ResetPublisher::publishResetCommand(const std::vector<float> &position, const std::vector<float> &orientation)
{
    // Inputs: position (x, y), orientation (quaternion)
    ::geometry_msgs::msg::dds_::Pose_ pose_msg;
    pose_msg.position().x(position[0]);
    pose_msg.position().y(position[1]);
    // Set orientation if needed
    pose_msg.orientation().w(orientation[0]);
    pose_msg.orientation().x(orientation[1]);
    pose_msg.orientation().y(orientation[2]);
    pose_msg.orientation().z(orientation[3]);

    reset_publisher->Write(pose_msg);
}