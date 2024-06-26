cmake_minimum_required(VERSION 3.0.2)
project(ros_joint)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  serial
  std_msgs
  tf
)
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_joint
#  CATKIN_DEPENDS geometry_msgs message_generation roscpp serial std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(ros_joint_sub src/ros_joint_sub.cpp)
add_dependencies(ros_joint_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_joint_sub
   ${catkin_LIBRARIES}
)

add_executable(ros_joint_teleop_sub src/ros_joint_teleop_sub.cpp)
add_dependencies(ros_joint_teleop_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_joint_teleop_sub
   ${catkin_LIBRARIES}
)