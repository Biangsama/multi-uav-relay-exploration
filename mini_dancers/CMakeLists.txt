cmake_minimum_required(VERSION 3.0.2)
project(mini_dancers)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  connector_core
  dancers_msgs
  geometry_msgs
  protobuf_msgs
  roscpp
  std_msgs
  visualization_msgs
)
find_package(Eigen3 REQUIRED)

set(MINI_DANCERS_SIM_INCLUDE_DIR "" CACHE PATH "Directory containing mini_dancers simulator headers such as uav_system.hpp")
set(_mini_dancers_sim_include_hints
  "${CMAKE_SOURCE_DIR}/../multi-uav-comm-control-cosimulation/src/physics_connectors/mini_dancers/include/mini_dancers"
  "$ENV{HOME}/communcation_multi_uav_platform_ws/multi-uav-comm-control-cosimulation/src/physics_connectors/mini_dancers/include/mini_dancers"
  "/home/zby/communcation_multi_uav_platform_ws/multi-uav-comm-control-cosimulation/src/physics_connectors/mini_dancers/include/mini_dancers"
)

if(NOT MINI_DANCERS_SIM_INCLUDE_DIR)
  foreach(candidate_dir ${_mini_dancers_sim_include_hints})
    if(candidate_dir AND EXISTS "${candidate_dir}/uav_system.hpp")
      set(MINI_DANCERS_SIM_INCLUDE_DIR "${candidate_dir}")
      break()
    endif()
  endforeach()
endif()

if(NOT MINI_DANCERS_SIM_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find mini_dancers simulator headers (uav_system.hpp). Set MINI_DANCERS_SIM_INCLUDE_DIR.")
endif()

catkin_package(
  CATKIN_DEPENDS connector_core dancers_msgs geometry_msgs protobuf_msgs roscpp std_msgs visualization_msgs
)

add_executable(mini_dancers_velocity
  src/mini_dancers_velocity.cpp
  src/phy_comm_module.cpp
)
target_compile_features(mini_dancers_velocity PRIVATE cxx_std_17)
target_include_directories(mini_dancers_velocity PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${MINI_DANCERS_SIM_INCLUDE_DIR}
)
target_link_libraries(mini_dancers_velocity
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

install(
  TARGETS mini_dancers_velocity
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
