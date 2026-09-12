cmake_minimum_required(VERSION 3.16)
project(scan_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation sample_consensus)

add_executable(scan_planner_odom src/main.cpp src/cloud.cpp src/cloud_cut.cpp)
target_compile_features(scan_planner_odom PRIVATE cxx_std_17)
ament_target_dependencies(scan_planner_odom
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
)
target_include_directories(scan_planner_odom PRIVATE include)
target_include_directories(scan_planner_odom SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(scan_planner_odom ${PCL_LIBRARIES})

install(TARGETS scan_planner_odom
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
