#include "dds/state_subscriber.h"

void StateSubscriber::Init()
{
    std::cout << "Initializing StateSubscriber..." << std::endl;
    joint_state_subscriber.reset(new ChannelSubscriber<::unitree_hg::msg::dds_::LowState_>(JOINT_STATE_TOPIC));
    sport_mode_subscriber.reset(new ChannelSubscriber<::unitree_go::msg::dds_::SportModeState_>(SPORT_MODE_TOPIC));
    imu_state_subscriber.reset(new ChannelSubscriber<::unitree_hg::msg::dds_::IMUState_>(IMU_STATE_TOPIC));

    joint_state_subscriber->InitChannel(std::bind(&StateSubscriber::jointStateCallback, this, std::placeholders::_1));
    sport_mode_subscriber->InitChannel(std::bind(&StateSubscriber::sportModeCallback, this, std::placeholders::_1));
    imu_state_subscriber->InitChannel(std::bind(&StateSubscriber::imuCallback, this, std::placeholders::_1));
    std::cout << "StateSubscriber initialized and subscribed to topics: " << JOINT_STATE_TOPIC << ", " << SPORT_MODE_TOPIC << ", " << IMU_STATE_TOPIC << std::endl;
}

RobotState StateSubscriber::GetRobotState()
{
    return current_state;
}

void StateSubscriber::jointStateCallback(const void *msg)
{
    const auto *low_state_msg = static_cast<const ::unitree_hg::msg::dds_::LowState_ *>(msg);

    for (int i = 0; i < low_state_msg->motor_state().size(); ++i) {
        current_state.q[i] = low_state_msg->motor_state()[i].q(); // note: for now just keeping order the same as the motor state, but may want to reorder to match whatever is going on in RLBase.cpp, where things get reordered before being sent to the robot
        current_state.dq[i] = low_state_msg->motor_state()[i].dq();
    }
}

// This IMU is the IMU of just the torso, use it to get the oritentation of the robot
void StateSubscriber::imuCallback(const void *msg)
{
    const auto *imu_state_msg = static_cast<const ::unitree_hg::msg::dds_::IMUState_ *>(msg);
    current_state.accel = imu_state_msg->accelerometer();
    current_state.angular_velocity = imu_state_msg->gyroscope();
    current_state.orientation = imu_state_msg->quaternion();

    // std::cout << "Received IMUState message: quaternion = [" 
    //             << imu_state_msg->quaternion()[0] << ", " 
    //             << imu_state_msg->quaternion()[1] << ", " 
    //             << imu_state_msg->quaternion()[2] << ", " 
    //             << imu_state_msg->quaternion()[3] << "]" << std::endl;
    // std::cout << "Received IMUState message: gyroscope = [" 
    //             << imu_state_msg->gyroscope()[0] << ", " 
    //             << imu_state_msg->gyroscope()[1] << ", " 
    //             << imu_state_msg->gyroscope()[2] << "]" << std::endl;
    // std::cout << "Received IMUState message: accelerometer = [" 
    //             << imu_state_msg->accelerometer()[0] << ", " 
    //             << imu_state_msg->accelerometer()[1] << ", " 
    //             << imu_state_msg->accelerometer()[2] << "]" << std::endl;   
    // std::cout<< "Received IMUState message: rpy = [" 
    //             << imu_state_msg->rpy()[0] << ", " 
    //             << imu_state_msg->rpy()[1] << ", " 
    //             << imu_state_msg->rpy()[2] << "]" << std::endl;
}

// this sport mode contains the aggregate velocity and position of the robot
void StateSubscriber::sportModeCallback(const void *msg)
{
    const auto *sport_mode_msg = static_cast<const ::unitree_go::msg::dds_::SportModeState_ *>(msg);
    current_state.position = sport_mode_msg->position();
    current_state.velocity = sport_mode_msg->velocity();

    // std::cout << "Received SportModeState message: pos = ["
    //             << sport_mode_msg->position()[0] << ", " 
    //             << sport_mode_msg->position()[1] << ", " 
    //             << sport_mode_msg->position()[2] << "]" << std::endl;
    // std::cout << "Received SportModeState message: vel = ["
    //             << sport_mode_msg->velocity()[0] << ", " 
    //             << sport_mode_msg->velocity()[1] << ", " 
    //             << sport_mode_msg->velocity()[2] << "]" << std::endl;    
    
    // std::cout << "Received SportModeState message: quaternion = ["
    //             << sport_mode_msg->imu_state().quaternion()[0] << ", "
    //             << sport_mode_msg->imu_state().quaternion()[1] << ", "
    //             << sport_mode_msg->imu_state().quaternion()[2] << ", "
    //             << sport_mode_msg->imu_state().quaternion()[3] << "]" << std::endl;
    // std::cout << "Received SportModeState message: gyroscope = ["
    //             << sport_mode_msg->imu_state().gyroscope()[0] << ", "
    //             << sport_mode_msg->imu_state().gyroscope()[1] << ", "
    //             << sport_mode_msg->imu_state().gyroscope()[2] << "]" << std::endl;
    // std::cout << "Received SportModeState message: accelerometer = ["
    //             << sport_mode_msg->imu_state().accelerometer()[0] << ", "
    //             << sport_mode_msg->imu_state().accelerometer()[1] << ", "
    //             << sport_mode_msg->imu_state().accelerometer()[2] << "]" << std::endl;
}

