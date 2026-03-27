cmake_minimum_required(VERSION 3.0.2)
project(connector_core)

find_package(catkin REQUIRED COMPONENTS
  protobuf_msgs
  roscpp
)
find_package(Boost REQUIRED COMPONENTS iostreams system)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES connector_core
  CATKIN_DEPENDS protobuf_msgs roscpp
  DEPENDS Boost
)

add_library(${PROJECT_NAME} src/connector_core_link.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp::yaml-cpp
  Boost::iostreams
  Boost::system
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
