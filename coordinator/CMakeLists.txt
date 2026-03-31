cmake_minimum_required(VERSION 3.0.2)
project(coordinator)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  connector_core
  nav_msgs
  protobuf_msgs
  roscpp
  rosgraph_msgs
)
find_package(Boost REQUIRED COMPONENTS fiber iostreams)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS connector_core nav_msgs protobuf_msgs roscpp rosgraph_msgs
)

add_executable(coordinator src/coordinator.cpp)
target_compile_features(coordinator PRIVATE cxx_std_17)
target_include_directories(coordinator PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(coordinator
  ${catkin_LIBRARIES}
  yaml-cpp::yaml-cpp
  Boost::fiber
  Boost::iostreams
)

install(
  TARGETS coordinator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
