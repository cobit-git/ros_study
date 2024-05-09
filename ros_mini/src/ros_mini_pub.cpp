#include "ros/ros.h"                            // ROS Default Header File
#include "ros_mini/MsgMini.h"    // MsgSerial Message File Header. The header file is automatically created when building the package.
#include <serial/serial.h>
#include <iostream>
#include "ros_mini/SrvMini.h"// Service1 Service File Header (Automatically created after build)

bool calculation(ros_mini::SrvMini::Request &req,
                 ros_mini::SrvMini::Response &res)
{
  // The service name is 'ros_srv' and it will call 'calculation' function upon the service request.
  res.result = req.a;

  // Displays 'a' and 'b' values used in the service request and
  // the 'result' value corresponding to the service response
  ROS_INFO("request: x=%ld", (long int)req.a);
  ROS_INFO("sending back response: %ld", (long int)res.result);

  return true;
}

int main(int argc, char **argv)                 // Node Main Function
{
  ros::init(argc, argv, "serial_topic_publisher");     // Initializes Node Name
  ros::NodeHandle nh;                           // Node handle declaration for communication with ROS system

  // Declare publisher, create publisher 'serial_topic_publisher' using the 'Msgserial'
  // message file from the 'ros_serial_topic' package. The topic name is
  // 'ros_serial_msg' and the size of the publisher queue is set to 100.
  ros::Publisher ros_serial_pub = nh.advertise<ros_mini::MsgMini>("ros_mini_msg", 100);

  ros::ServiceServer ros_service_server = nh.advertiseService("ros_s", calculation);
  ROS_INFO("ready srv server!");
  // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
  ros::Rate loop_rate(10);

  ros_mini::MsgMini msg;     // Declares message 'msg' in 'Msgserial' message file format

  serial::Serial ser;
  ser.setPort("/dev/ttyACM1");  // Replace with your serial port name
  ser.setBaudrate(115200);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  ser.setTimeout(timeout);
  ser.open();

  if (!ser.isOpen()) {
    ROS_ERROR("Serial port could not be opened.");
    return -1;
  }

  while (ros::ok())
  { 
    if (ser.available()>0){
      std::string data_str = ser.readline();
      //ROS_INFO("arduini send %s",data_str.c_str());
      
      if(data_str.length() >  0){ // Sometiems data_str is NULL, then it triggers segment fault 
        int data = std::stof(data_str);  // Convert string to integer
        msg.distance = (float)data;
        ROS_INFO("send msg = %f", msg.distance);        // Prints the 'data' message
        ros_serial_pub.publish(msg);          // Publishes 'msg' message
      }
      
    }
    loop_rate.sleep();                      // Goes to sleep according to the loop rate defined above.
    

  }

  return 0;
}