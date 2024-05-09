#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // Process the received Twist message here
  ROS_INFO_STREAM("Linear velocity: " << msg->linear.x << ", " << msg->linear.y << ", " << msg->linear.z);
  ROS_INFO_STREAM("Angular velocity: " << msg->angular.x << ", " << msg->angular.y << ", " << msg->angular.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "teleop_receiver");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback); // Subscribe to "/cmd_vel" with queue size 10

  ros::spin();

  return 0;
}
