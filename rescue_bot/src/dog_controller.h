#ifndef DOGCONTROLLER_H
#define DOGCONTROLLER_H

#include "ros/ros.h"

// ROS message types
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

class DogController
{
    public:
        /**
         * @brief Default constructor for dog. Sets up internal variables,
         * subscribers, publishers and services.
         * 
         * @param[in] nh The ROS node handle, used to identify node instance
        */
        DogController(ros::NodeHandle nh);

        /**
         * @brief Sends a command to the robotics platform.
         * 
         * @param[in] x Positive moves forward, negative moves backward
         * @param[in] y Positive moves left, negative moves right
         * @param[in] th Positive rotates anti-clockwise, negative rotates clockwise
        */
        void sendCommand(float x, float y, float th);

    private:
        float moveSpeed_ = 0.5;                 // linear speed
        float turnSpeed_ = 1.0;                 // angular speed

        ros::NodeHandle nh_;
        
        ros::Publisher velocityPublisher_;      // Publishes to 'cmd_vel' rostopic
};

#endif // DOGCONTROLLER_H