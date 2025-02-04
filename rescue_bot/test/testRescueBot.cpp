#include <gtest/gtest.h>
#include <climits>
#include <vector>

// #include <ros/ros.h>
// #include <ros/service_client.h>

#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/Point.h"
// #include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "rescue_bot.h"
#include "dog_controller.h"

void setRobotToOrigin(ros::NodeHandle n)
{
    // Call gazebo service to set robot to origin
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");

    gazebo_msgs::SetModelState service;
    
    service.request.model_state.model_name = "/";
    
    service.request.model_state.pose.position.x = 0;
    service.request.model_state.pose.position.y = 0;
    service.request.model_state.pose.position.z = 0.5;

    service.request.model_state.pose.orientation.x = 0;
    service.request.model_state.pose.orientation.x = 0;
    service.request.model_state.pose.orientation.x = 0;
    service.request.model_state.pose.orientation.w = 1;

    service.request.model_state.reference_frame = "";

    service.request.model_state.twist.angular.x = 0;
    service.request.model_state.twist.angular.y = 0;
    service.request.model_state.twist.angular.z = 0;

    service.request.model_state.twist.linear.x = 0;
    service.request.model_state.twist.linear.y = 0;
    service.request.model_state.twist.linear.z = 0;

    if (client.call(service))
    {
        ROS_INFO("SET MODEL_STATE TO ORIGIN");
    }
    else
    {
        ROS_ERROR("Failed to call service gazebo/set_model_state");
    }
}

TEST(RescueBotTest, LocalisationTest)
{
    ros::NodeHandle n;
    setRobotToOrigin(n);

    // walk towards goal
    // RescueBot* dog = new RescueBot(n);
    std::shared_ptr<RescueBot> rescueBot(new RescueBot(n));

    // geometry_msgs::Point goal;
    // goal.x = 7;
    // goal.y = 0;
    // goal.z = 0;
    // rescueBot->setGoal(goal);

    ASSERT_EQ(1, 0.5);
}

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testRescueBot");
    ros::spinOnce();

    return RUN_ALL_TESTS();
}
