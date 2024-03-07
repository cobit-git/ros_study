#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

void myCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("rx: %f",msg->linear.x);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, myCallBack);
    ros::spin();
    return 0;
}