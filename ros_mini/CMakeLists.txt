cmake_minimum_required(VERSION 3.0.2)
project(ros_mini)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  serial
  std_msgs
)
add_message_files(
   FILES
   MsgMini.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SrvMini.srv
#   Service2.srv
 )
 generate_messages(
   DEPENDENCIES
   std_msgs
 )
catkin_package(
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(ros_mini_pub src/ros_mini_pub.cpp)
add_dependencies(ros_mini_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_mini_pub
   ${catkin_LIBRARIES}
 )
 add_executable(ros_mini_sub src/ros_mini_sub.cpp)
add_dependencies(ros_mini_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_mini_sub
   ${catkin_LIBRARIES}
 )
