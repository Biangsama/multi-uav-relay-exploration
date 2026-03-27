cmake_minimum_required(VERSION 3.0.2)
project(relay_racer_integration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  connector_core
  dancers_msgs
  geometry_msgs
  message_generation
  nav_msgs
  protobuf_msgs
  roscpp
  std_msgs
  topic_tools
)
find_package(Protobuf REQUIRED)

add_message_files(
  FILES
  RelayRoleCmd.msg
  RelayPeerState.msg
  RelayProposal.msg
  RelayTaskState.msg
  SwarmCommState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS connector_core dancers_msgs geometry_msgs message_runtime nav_msgs protobuf_msgs roscpp std_msgs topic_tools
  DEPENDS Protobuf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)

add_executable(racer_phy_connector_node
  src/racer_phy_connector_node.cpp
)
add_dependencies(racer_phy_connector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(racer_phy_connector_node
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

add_executable(racer_real_phy_bridge_node
  src/racer_real_phy_bridge_node.cpp
)
add_dependencies(racer_real_phy_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(racer_real_phy_bridge_node
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

add_executable(racer_msg_tx_bridge_node
  src/racer_msg_tx_bridge_node.cpp
)
add_dependencies(racer_msg_tx_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(racer_msg_tx_bridge_node
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

add_executable(racer_msg_rx_bridge_node
  src/racer_msg_rx_bridge_node.cpp
)
add_dependencies(racer_msg_rx_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(racer_msg_rx_bridge_node
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

add_executable(racer_msg_relay_forwarder_node
  src/racer_msg_relay_forwarder_node.cpp
)
add_dependencies(racer_msg_relay_forwarder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(racer_msg_relay_forwarder_node
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

add_executable(racer_comm_controller_node
  src/racer_comm_controller_node.cpp
)
add_dependencies(racer_comm_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(racer_comm_controller_node
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

add_executable(racer_local_relay_controller_node
  src/racer_local_relay_controller_node.cpp
)
add_dependencies(racer_local_relay_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(racer_local_relay_controller_node
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

add_executable(racer_topic_router_node
  src/racer_topic_router_node.cpp
)
add_dependencies(racer_topic_router_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(racer_topic_router_node
  ${catkin_LIBRARIES}
)

add_executable(relay_closed_loop_test_node
  src/relay_closed_loop_test_node.cpp
)
add_dependencies(relay_closed_loop_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(relay_closed_loop_test_node
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

install(TARGETS
  racer_phy_connector_node
  racer_msg_tx_bridge_node
  racer_msg_rx_bridge_node
  racer_msg_relay_forwarder_node
  racer_real_phy_bridge_node
  racer_comm_controller_node
  racer_local_relay_controller_node
  racer_topic_router_node
  relay_closed_loop_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/relay_validation_monitor.py
  scripts/relay_rviz_overlay.py
  scripts/planner_sensor_gate.py
  scripts/racer_real_odom_bridge.py
  scripts/stagec_transport_probe.py
  scripts/relay_log_console.py
  scripts/relay_efficiency_eval.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
