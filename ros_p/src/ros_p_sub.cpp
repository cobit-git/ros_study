#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // Assuming teleop publishes Twist messages
#include <sensor_msgs/JointState.h>

void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // Process teleop data (linear and angular velocities)
  ROS_INFO("Received teleop message: linear vel (%.2f, %.2f), angular vel %.2f",
           msg->linear.x, msg->linear.y, msg->angular.z);
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  // Process joint state data (joint names and positions)
  ROS_INFO("Received joint state message:");
  for (size_t i = 0; i < msg->name.size(); i++) {
    ROS_INFO("  Joint: %s, Position: %.2f", msg->name[i].c_str(), msg->position[i]);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subscriber_node");
  ros::NodeHandle nh;

  // Subscribe to teleop topic (replace "/cmd_vel" with the actual topic name)
  ros::Subscriber teleopSubscriber = nh.subscribe("/cmd_vel", 10, teleopCallback);

  // Subscribe to joint state topic (replace "/joint_states" with the actual topic name)
  ros::Subscriber jointStateSubscriber = nh.subscribe("/joint_states", 10, jointStateCallback);

  ros::spin();

  return 0;
}
