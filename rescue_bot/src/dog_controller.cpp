#include "dog_controller.h"

DogController::DogController(ros::NodeHandle nh)
{
    // set ROS node handle
    nh_ = nh;

    // Publishes to 'cmd_vel' topic
    velocityPublisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10, false);
}

void DogController::sendCommand(float x, float y, float th)
{
    geometry_msgs::Twist msg;
    msg.linear.x = x * moveSpeed_;
    msg.linear.y = y * moveSpeed_;
    msg.angular.z = th * turnSpeed_;

    // std::cout << "Sent commands - x: " << msg.linear.x << ", y: " << msg.linear.y << ", th: " << msg.angular.z << std::endl;
    velocityPublisher_.publish(msg);
}
