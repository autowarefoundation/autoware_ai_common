cmake_minimum_required(VERSION 2.8.3)
project(emergency_handler)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  autoware_msgs
  autoware_system_msgs
  autoware_health_checker
  roscpp
  std_msgs
  ros_observer
  roslint
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

catkin_package(
  LIBRARIES
  system_status_filter
  CATKIN_DEPENDS
  autoware_msgs
  autoware_system_msgs
  autoware_health_checker
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(system_status_filter lib/libsystem_status_filter.cpp lib/libvital_monitor.cpp)

target_link_libraries(system_status_filter
  ${catkin_LIBRARIES}
)

add_dependencies(system_status_filter ${catkin_EXPORTED_TARGETS})

set(EMERGENCY_HANDLER_SRC
  src/emergency_handler.cpp
  src/emergency_stop_planner.cpp
  src/system_status_filter.cpp
)

add_executable(
  emergency_handler
  src/emergency_handler_node.cpp
  ${EMERGENCY_HANDLER_SRC}
)

target_link_libraries(
  emergency_handler
  system_status_filter
  ${catkin_LIBRARIES}
)

add_dependencies(emergency_handler ${catkin_EXPORTED_TARGETS})

install(TARGETS
        emergency_handler
        system_status_filter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#Install launch
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

# Install config
install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

if (CATKIN_ENABLE_TESTING)
  roslint_add_test()
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_emergency_handler
  test/test_emergency_handler.test
  test/src/test_emergency_handler.cpp
  ${EMERGENCY_HANDLER_SRC})
  target_link_libraries(test_emergency_handler
  system_status_filter
  ${catkin_LIBRARIES})
endif()
