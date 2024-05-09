cmake_minimum_required(VERSION 3.0.2)
project(ros_serial_topic)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  serial
  std_msgs
)

 add_message_files(
   FILES
   MsgSer.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_serial_topic_pub src/ros_serial_topic_pub.cpp)
add_dependencies(ros_serial_topic_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_serial_topic_pub
   ${catkin_LIBRARIES}
)

add_executable(ros_serial_topic_sub src/ros_serial_topic_sub.cpp)
add_dependencies(ros_serial_topic_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_serial_topic_sub
   ${catkin_LIBRARIES}
)