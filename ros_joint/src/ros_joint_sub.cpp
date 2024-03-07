#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial/serial.h>
serial::Serial ser;
void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
    // Access joint states here (e.g., using msg->name and msg->position)
    //ROS_INFO("Received joint state update:");
    for (size_t i = 0; i < msg->name.size(); ++i) {
        //ROS_INFO("%s: %f", msg->name[i].c_str(), msg->position[i]);
        if(i == 0){
            int angle = (int)(msg->position[i]*100);
            if(angle > 0){
                ROS_INFO("%s: %d", msg->name[i].c_str(), angle);
                ser.write("a");
                ser.write(std::to_string(angle));
                ser.write("b\n");
            }
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_state_subscriber");
    ros::NodeHandle nh;

    
    ser.setPort("/dev/ttyACM1");  // Replace with your serial port name
    ser.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(timeout);
    ser.open();

    //if (!ser.isOpen()) {
    //    ROS_ERROR("Serial port could not be opened.");
    //    return -1;
    //}

    // Subscribe to the "/joint_states" topic
    ros::Subscriber sub = nh.subscribe("/joint_states", 10, jointStateCallback);

    ROS_INFO("Waiting for joint state messages...");

    ros::spin();

    return 0;
}
