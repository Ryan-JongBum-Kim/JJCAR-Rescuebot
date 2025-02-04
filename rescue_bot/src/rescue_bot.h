#ifndef RESCUEBOT_H
#define RESCUEBOT_H

#include "ros/ros.h"

#include <cmath>
#include <vector>
#include <mutex>
#include <thread>

#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>

// ROS message types
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"

#include "dog_controller.h"

class RescueBot
{
    public:
        /**
         * @brief Default constructor for dog. Sets up internal variables,
         * subscribers, publishers and services.
         * 
         * @param[in] nh The ROS node handle, used to identify node instance
        */
        RescueBot(ros::NodeHandle nh);

        /**
         * @brief Destroy the Rescue Bot object
         */
        ~RescueBot();

        /**
         * @brief Set the Goal object
         * 
         * @param[in] goal The chosen goal point 
         */
        void setGoal(geometry_msgs::Point goal);

        /**
         * @brief Reach the current Goal
         * 
         * Using the current position of the robot, the robot tries to
         * reach the current goal by sending movement commands to the
         * controller in order to reduce the error in its position up
         * to a certain tolerance.
         */
        void reachGoal();

        /**
         * @brief Gets the distance from the origin to the goal
         * 
         * @param[in] origin - The origin pose (stamped)
         * @param[in] goal - The goal pose (stamped)
         * @return The distance to the goal as a float
         */
        double distanceToGoal(geometry_msgs::PoseStamped origin,
                              geometry_msgs::PointStamped goal);

    private:
        void poseCallback(nav_msgs::Odometry pose);
        
        geometry_msgs::Point globalToLocal(geometry_msgs::Point point);

    private:
        ros::NodeHandle nh_;

        DogController* dogPtr_;

        ros::Subscriber poseSubscriber_;
        ros::Subscriber goalSubscriber_;

        std::thread controlThread_;

        geometry_msgs::Point currentGoal_;
        bool goalReached_;

        // geometry_msgs::PoseWithCovarianceStamped pose_;
        nav_msgs::Odometry pose_;
};

#endif // RESCUEBOT_H