cmake_minimum_required(VERSION 3.0.2)
project(dancers_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  AgentState.msg
  AgentStruct.msg
  AgentStructArray.msg
  Neighbor.msg
  NeighborArray.msg
  Target.msg
  VelocityHeading.msg
  VelocityHeadingArray.msg
)

add_service_files(
  FILES
  GetAgentVelocities.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
)
