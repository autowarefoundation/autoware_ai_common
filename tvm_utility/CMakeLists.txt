cmake_minimum_required(VERSION 2.8.3)
project(tvm_utility)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(autoware_build_flags REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# set dependencies for the tests
set(CATKIN_TEST_DEPENDS pcl_ros autoware_msgs)

find_package(
  catkin REQUIRED
  COMPONENTS tvm_vendor roslint ${CATKIN_TEST_DEPENDS}
)

# declare dependency packages to catkin
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_TEST_DEPENDS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${tvm_vendor_INCLUDE_DIRS}/tvm_vendor
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  roslint_add_test()

  # compile each folder inside test/ as a test case
  file(GLOB test_cases test/*)
  foreach(test_folder ${test_cases})
    if(EXISTS "${test_folder}/inference_engine_tvm_config.hpp"
       AND EXISTS "${test_folder}/deploy_lib.so"
       AND EXISTS "${test_folder}/deploy_graph.json"
       AND EXISTS "${test_folder}/deploy_param.params"
    )
      # the folder name becomes the test case name
      file(RELATIVE_PATH test_case_name ${CMAKE_CURRENT_LIST_DIR}/test ${test_folder})

      # add all cpp files in the folder to the target
      file(GLOB test_case_sources ${test_folder}/*.cpp)
      catkin_add_gtest(${test_case_name} ${test_case_sources})

      # set extra properties of the target
      set_property(TARGET ${test_case_name} PROPERTY CXX_STANDARD 14)
      target_link_libraries(${test_case_name}
        ${catkin_LIBRARIES}
        boost_system
        pthread
        tvm_runtime
        ${OpenCV_LIBS}
      )

      # copy compiled model files
      if(EXISTS "${test_folder}/inference_engine_tvm_config.hpp")
        file(COPY ${test_folder}/inference_engine_tvm_config.hpp DESTINATION ${CMAKE_BINARY_DIR})
      endif()
      if(EXISTS "${test_folder}/deploy_lib.so")
        file(COPY ${test_folder}/deploy_lib.so DESTINATION ${CMAKE_BINARY_DIR})
      endif()
      if(EXISTS "${test_folder}/deploy_graph.json")
        file(COPY ${test_folder}/deploy_graph.json DESTINATION ${CMAKE_BINARY_DIR})
      endif()
      if(EXISTS "${test_folder}/deploy_param.params")
        file(COPY ${test_folder}/deploy_param.params DESTINATION ${CMAKE_BINARY_DIR})
      endif()

      # copy yolo_v2_tiny specific files
      if(EXISTS "${test_folder}/test_image_0.jpg")
        file(COPY ${test_folder}/test_image_0.jpg DESTINATION ${CMAKE_BINARY_DIR})
      endif()
      file(COPY ${test_folder}/anchors.csv DESTINATION ${CMAKE_BINARY_DIR})
      file(COPY ${test_folder}/labels.txt DESTINATION ${CMAKE_BINARY_DIR})
    else()
      message(WARNING "No model is generated for ${test_folder}, skipping test")
    endif()

  endforeach(test_folder)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
