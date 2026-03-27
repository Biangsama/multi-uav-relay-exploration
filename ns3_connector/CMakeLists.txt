cmake_minimum_required(VERSION 3.0.2)
project(ns3_connector)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  connector_core
  dancers_msgs
  protobuf_msgs
  roscpp
  std_msgs
  visualization_msgs
)
find_package(Boost REQUIRED COMPONENTS iostreams)

set(NS3_LOCAL_CMAKE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../.deps/ns3.42-install/lib/cmake/ns3")
set(NS3_FALLBACK_CMAKE_DIR "$ENV{HOME}/ns3-work/ns-allinone-3.38/ns-3.38/cmake-cache")

if(NOT DEFINED ns3_DIR)
  foreach(candidate_dir
      "${NS3_LOCAL_CMAKE_DIR}"
      "$ENV{NS3_DIR}"
      "$ENV{ns3_DIR}"
      "${NS3_FALLBACK_CMAKE_DIR}")
    if(candidate_dir AND EXISTS "${candidate_dir}/ns3Config.cmake")
      get_filename_component(ns3_DIR "${candidate_dir}" ABSOLUTE)
      break()
    endif()
  endforeach()
endif()

find_package(ns3 QUIET)

catkin_package(
  CATKIN_DEPENDS connector_core dancers_msgs protobuf_msgs roscpp std_msgs visualization_msgs
  INCLUDE_DIRS include
)

if(ns3_FOUND)
  add_executable(basic_wifi_adhoc
    src/basic_wifi_adhoc.cpp
    include/ns3_connector/applications/waypoint_broadcast.cpp
    include/ns3_connector/applications/waypoint_broadcast.h
  )
  target_compile_features(basic_wifi_adhoc PRIVATE cxx_std_20)
  target_include_directories(basic_wifi_adhoc PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ns3_connector
  )
  target_link_libraries(basic_wifi_adhoc
    ${catkin_LIBRARIES}
    Boost::iostreams
    ns3::libapplications
    ns3::libbuildings
    ns3::libcore
    ns3::libinternet
    ns3::libmobility
    ns3::libnetwork
    ns3::libpropagation
    ns3::libstats
    ns3::libwifi
  )

  add_executable(basic_wifi_adhoc_with_controller
    src/basic_wifi_adhoc_with_controller.cpp
    include/ns3_connector/applications/flocking-application.cpp
    include/ns3_connector/applications/flocking-application.h
    include/ns3_connector/applications/swarm_message_application.cpp
    include/ns3_connector/applications/swarm_message_application.h
  )
  target_compile_features(basic_wifi_adhoc_with_controller PRIVATE cxx_std_20)
  target_include_directories(basic_wifi_adhoc_with_controller PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ns3_connector
  )
  target_link_libraries(basic_wifi_adhoc_with_controller
    ${catkin_LIBRARIES}
    Boost::iostreams
    ns3::libapplications
    ns3::libbuildings
    ns3::libcore
    ns3::libinternet
    ns3::libmobility
    ns3::libnetwork
    ns3::libpropagation
    ns3::libspectrum
    ns3::libwifi
  )

  install(
    TARGETS basic_wifi_adhoc basic_wifi_adhoc_with_controller
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message(WARNING "ns3 CMake package not found; skipping build of ns3_connector targets.")
endif()

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
