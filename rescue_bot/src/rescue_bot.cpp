#include "rescue_bot.h"

#include <algorithm>

// HELPER
double normalizeAngle(double angle) {
    return atan2(sin(angle), cos(angle));
}   

RescueBot::RescueBot(ros::NodeHandle nh) :
    nh_(nh)
{
    poseSubscriber_ = nh_.subscribe("/odom/ground_truth", 100, &RescueBot::poseCallback, this);

    goalSubscriber_ = nh_.subscribe("/goal_point", 3, &RescueBot::setGoal, this);

    // create a dog controller object
    dogPtr_ = new DogController(nh_);

    pose_.pose.pose.orientation.w = 1;

    controlThread_ = std::thread(&RescueBot::reachGoal, this);
}

RescueBot::~RescueBot()
{
    // Join thread once quadcopter is about to be destroyed
    if (controlThread_.joinable())
    {
        controlThread_.join();
    }

    if (dogPtr_ != nullptr)
    {
        delete dogPtr_;
    }
}

void RescueBot::setGoal(geometry_msgs::Point goal)
{
    goalReached_ = false;
    currentGoal_ = goal;
}

void RescueBot::poseCallback(nav_msgs::Odometry pose)
{
    pose_ = pose;
    std::cout << pose << std::endl;
}

void RescueBot::reachGoal()
{
    ros::Rate rate_limiter(1);
    goalReached_ = false;

    while (ros::ok)
    {
        // currentGoal_.x = 7;
        // currentGoal_.y = 0;
        // currentGoal_.z = 0;

        std::cout << currentGoal_ << std::endl;

        geometry_msgs::Point localPoint = globalToLocal(currentGoal_);

        double dx = currentGoal_.x - pose_.pose.pose.position.x;
        double dy = currentGoal_.y - pose_.pose.pose.position.y;
        double dist = hypot(dx, dy);

        double d_th = atan2(localPoint.y, localPoint.x) - tf::getYaw(pose_.pose.pose.orientation);
        d_th = normalizeAngle(d_th);

        // if the angle is NaN, then don't send commands
        if (isnan(d_th))
        {
            // std::cout << pose_ << std::endl;
            continue;
        }

        // if the angle is greater than 90deg and the distance is greater than 1m
        // or if the angle is greather than 20deg and the distance is less than 1m
        // only spin the dog around without walking
        if ((abs(d_th) > M_PI_2 && dist > 1) || (abs(d_th) > (20 * (M_PI / 180)) && dist < 1))
        {
            dogPtr_->sendCommand(0, 0, d_th);
            continue;
        }

        if (abs(d_th) > 0.05 || dist > 0.5)
        {
            std::cout << "distance: " << std::min(dist, 1.0) << " | theta:" << d_th << std::endl;
            // std::cout << pose_ << std::endl;
            dogPtr_->sendCommand(std::min(dist, 1.0), 0, d_th);
            continue;
        }

        goalReached_ = true;
        dogPtr_->sendCommand(0, 0, 0);

        rate_limiter.sleep();
    }

    ros::shutdown();
}

double RescueBot::distanceToGoal(geometry_msgs::PoseStamped origin,
                                 geometry_msgs::PointStamped goal)
{
    double dx = goal.point.x - origin.pose.position.x;
    double dy = goal.point.y - origin.pose.position.y;

    return std::hypot(dx, dy);
}

geometry_msgs::Point RescueBot::globalToLocal(geometry_msgs::Point point)
{
    geometry_msgs::Point outPoint;

    outPoint.x = point.x - pose_.pose.pose.position.x;
    outPoint.y = point.y - pose_.pose.pose.position.y;
    outPoint.z = 0;

    return outPoint;
}