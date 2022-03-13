#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ttcRadar_pkg/ttcRadar_msg.h"

ttcRadar_pkg::ttcRadar_msg ttcRadar_output_msg;

void chatterCallback (const ttcRadar_pkg::ttcRadar_msg& ttcRadar_output_msg)
{
    ROS_INFO("I heard %u:   isObject = [%u]", ttcRadar_output_msg.msg_counter, ttcRadar_output_msg.isObject);
    ROS_INFO("              distance = [%f] \r\n", ttcRadar_output_msg.distance);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ttcListener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ttcRadar_Data", 1000, chatterCallback);
    ros::spin();
    return 0;
}