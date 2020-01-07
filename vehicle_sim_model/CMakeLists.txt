cmake_minimum_required(VERSION 2.8.3)
project(vehicle_sim_model)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roslint
)
find_package(Eigen3 QUIET)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES vehicle_sim_model
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
add_compile_options(-std=c++14)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(vehicle_sim_model_SRC
  src/vehicle_model_interface.cpp
  src/vehicle_model_ideal.cpp
  src/vehicle_model_constant_acceleration.cpp
  src/vehicle_model_time_delay.cpp
)

add_library(vehicle_sim_model ${vehicle_sim_model_SRC})
add_dependencies(vehicle_sim_model ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_sim_model ${catkin_LIBRARIES})

install(TARGETS vehicle_sim_model
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/fitParamDelayInputModel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test-vehicle_sim_model
    test/test_vehicle_sim_model.test
    test/src/test_vehicle_sim_model.cpp
  )
  add_dependencies(test-vehicle_sim_model ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test-vehicle_sim_model
    vehicle_sim_model
    ${catkin_LIBRARIES}
  )
  roslint_add_test()
endif ()
