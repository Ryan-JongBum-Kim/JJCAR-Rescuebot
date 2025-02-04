#include <thread>
#include <string>

#include "ros/ros.h"

#include "rescue_bot.h"

int main(int argc, char **argv)
{
    std::cout << "STARTED" << std::endl;

    ros::init(argc, argv, "rescue_bot");

    ros::NodeHandle nh;

    std::shared_ptr<RescueBot> rescueBot(new RescueBot(nh));

    ros::spin();

    ros::shutdown();

    return 0;
}