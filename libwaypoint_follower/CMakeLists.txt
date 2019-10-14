cmake_minimum_required(VERSION 2.8.3)
project(libwaypoint_follower)

find_package(autoware_build_flags)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        autoware_msgs
        rosunit
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES libwaypoint_follower
        CATKIN_DEPENDS
        std_msgs
        geometry_msgs
        autoware_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(libwaypoint_follower
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(libwaypoint_follower src/libwaypoint_follower.cpp)
add_dependencies(libwaypoint_follower ${catkin_EXPORTED_TARGETS})
target_link_libraries(libwaypoint_follower ${catkin_LIBRARIES})


## Install executables and/or libraries
install(TARGETS libwaypoint_follower
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test-libwaypoint_follower
      test/test_libwaypoint_follower.test
      test/src/test_libwaypoint_follower.cpp
      src/libwaypoint_follower.cpp
    )
    add_dependencies(test-libwaypoint_follower ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test-libwaypoint_follower
    ${catkin_LIBRARIES})
endif ()
