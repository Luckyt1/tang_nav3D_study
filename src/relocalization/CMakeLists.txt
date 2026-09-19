cmake_minimum_required(VERSION 3.16)
project(relocalization)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(map REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree filters registration)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Candidate.msg" "msg/CandidateArray.msg" "msg/FineResult.msg" "msg/TrackingStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)

add_library(relocalization_processing STATIC third_party/localization/src/relocalization.cpp)
target_compile_features(relocalization_processing PUBLIC cxx_std_17)
target_include_directories(relocalization_processing PUBLIC third_party/localization/include)
target_include_directories(relocalization_processing SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(relocalization_processing PUBLIC
  map::freedom_processing map::submap_processing ${OpenCV_LIBS})
target_compile_options(relocalization_processing PRIVATE -Wall -Wextra -Wpedantic)

add_library(fine_registration STATIC third_party/localization/src/fine_registration.cpp)
target_compile_features(fine_registration PUBLIC cxx_std_17)
target_include_directories(fine_registration PUBLIC third_party/localization/include)
target_include_directories(fine_registration SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(fine_registration PUBLIC relocalization_processing ${PCL_LIBRARIES})
target_compile_options(fine_registration PRIVATE -Wall -Wextra -Wpedantic)

add_library(pose_tracking STATIC third_party/localization/src/pose_tracking.cpp)
target_compile_features(pose_tracking PUBLIC cxx_std_17)
target_include_directories(pose_tracking PUBLIC third_party/localization/include)
target_link_libraries(pose_tracking PUBLIC Eigen3::Eigen)
target_compile_options(pose_tracking PRIVATE -Wall -Wextra -Wpedantic)

add_executable(relocalization_node src/relocalization_node.cpp)
ament_target_dependencies(relocalization_node
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros pcl_conversions)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(relocalization_node fine_registration pose_tracking "${cpp_typesupport_target}")
target_compile_options(relocalization_node PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS relocalization_node DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_relocalization test/test_relocalization.cpp)
  target_link_libraries(test_relocalization relocalization_processing)
  ament_add_gtest(test_fine_registration test/test_fine_registration.cpp)
  target_link_libraries(test_fine_registration fine_registration)
  ament_add_gtest(test_pose_tracking test/test_pose_tracking.cpp)
  target_link_libraries(test_pose_tracking pose_tracking)
  add_executable(relocalization_test_fixture test/create_fixture.cpp)
  target_link_libraries(relocalization_test_fixture relocalization_processing)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME relocalization_node_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_node.py
      $<TARGET_FILE:relocalization_node> $<TARGET_FILE:relocalization_test_fixture>)
  set_tests_properties(relocalization_node_integration PROPERTIES TIMEOUT 90)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
