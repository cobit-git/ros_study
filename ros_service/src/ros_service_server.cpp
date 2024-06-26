#include "ros/ros.h"                          // ROS Default Header File
#include "ros_service/Service1.h"// Service1 Service File Header (Automatically created after build)

// The below process is performed when there is a service request
// The service request is declared as 'req', and the service response is declared as 'res'
bool calculation(ros_service::Service1::Request &req,
                 ros_service::Service1::Response &res)
{
  // The service name is 'ros_srv' and it will call 'calculation' function upon the service request.
  res.result = req.a + req.b;

  // Displays 'a' and 'b' values used in the service request and
  // the 'result' value corresponding to the service response
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: %ld", (long int)res.result);

  return true;
}

int main(int argc, char **argv)              // Node Main Function
{
  ros::init(argc, argv, "service_server");   // Initializes Node Name
  ros::NodeHandle nh;                        // Node handle declaration

  // Declare service server 'ros_service_server'
  // using the 'Service1' service file in the 'ros_service' package.
  // The service name is 'ros_srv' and it will call 'calculation' function
  // upon the service request.
  ros::ServiceServer ros_service_server = nh.advertiseService("ros_srv", calculation);

  ROS_INFO("ready srv server!");

  ros::spinOnce();    // Wait for the service request

  return 0;
}