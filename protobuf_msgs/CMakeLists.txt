cmake_minimum_required(VERSION 3.0.2)
project(protobuf_msgs)

find_package(catkin REQUIRED)
find_package(Protobuf REQUIRED)

set(PROTO_FILES
  protobuf/dancers_update.proto
  protobuf/pose_vector.proto
  protobuf/ordered_neighbors.proto
  protobuf/velocity_heading_vector.proto
  protobuf/state_payload.proto
  protobuf/net_rx_events.proto
  protobuf/agent_state_batch.proto
  protobuf/racer_swarm_msg.proto
)

include_directories(
  ${Protobuf_INCLUDE_DIRS}
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

add_library(${PROJECT_NAME}_proto ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME}_proto ${Protobuf_LIBRARIES})

set(PROTO_DEVEL_INCLUDE_DIR
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
)
file(MAKE_DIRECTORY ${PROTO_DEVEL_INCLUDE_DIR})

foreach(PROTO_HDR ${PROTO_HDRS})
  get_filename_component(PROTO_HDR_NAME ${PROTO_HDR} NAME)
  add_custom_command(
    TARGET ${PROJECT_NAME}_proto
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${PROTO_HDR}
            ${PROTO_DEVEL_INCLUDE_DIR}/${PROTO_HDR_NAME}
    VERBATIM
  )
endforeach()

catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  LIBRARIES ${PROJECT_NAME}_proto
  DEPENDS Protobuf
)

install(
  TARGETS ${PROJECT_NAME}_proto
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES ${PROTO_HDRS}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
)

install(
  FILES ${PROTO_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/protobuf
)
