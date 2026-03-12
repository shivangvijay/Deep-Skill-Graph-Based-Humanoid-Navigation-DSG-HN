#include "reset_subscriber.h"

void ResetSubscriber::Init()
{
    std::cout << "Initializing ResetSubscriber..." << std::endl;
    reset_subscriber.reset(new ChannelSubscriber<::geometry_msgs::msg::dds_::Pose_>(TOPIC_RESET));
    reset_subscriber->InitChannel(std::bind(&ResetSubscriber::reset_callback, this, std::placeholders::_1));
    std::cout << "ResetSubscriber initialized and ready to subscribe to topic: " << TOPIC_RESET << std::endl;
}

void ResetSubscriber::reset_callback(const void *msg)
{
    const auto *pose_msg = static_cast<const ::geometry_msgs::msg::dds_::Pose_ *>(msg);
    reset_pose[0] = pose_msg->position().x();
    reset_pose[1] = pose_msg->position().y();
    reset_pose[2] = 0.9f; // Z_START_HEIGHT
    reset_pose[3] = pose_msg->orientation().w();
    reset_pose[4] = pose_msg->orientation().x();
    reset_pose[5] = pose_msg->orientation().y();
    reset_pose[6] = pose_msg->orientation().z();
    //std::cout << "Received reset message" << std::endl;
    reset = true;
}