cmake_minimum_required(VERSION 3.16)
project(planner LANGUAGES C CXX)

if(POLICY CMP0144)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation sample_consensus)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  "msg/DataDisp.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)
rosidl_get_typesupport_target(planner_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

set(planner_ros_dependencies
  ament_index_cpp
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs)

set(planner_algorithm_include_dirs
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/plan_env/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/path_searching/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bspline_opt/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/traj_utils/include)

add_library(plan_env_internal STATIC
  third_party/plan_env/src/grid_map.cpp
  third_party/plan_env/src/raycast.cpp)
target_include_directories(plan_env_internal PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/plan_env/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(plan_env_internal
  cv_bridge geometry_msgs message_filters nav_msgs pcl_conversions rclcpp
  sensor_msgs tf2 tf2_ros visualization_msgs)
target_link_libraries(plan_env_internal Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_library(path_searching_internal STATIC third_party/path_searching/src/dyn_a_star.cpp)
target_include_directories(path_searching_internal PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/path_searching/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/plan_env/include)
ament_target_dependencies(path_searching_internal rclcpp)
target_link_libraries(path_searching_internal Eigen3::Eigen plan_env_internal)

add_library(bspline_opt_internal STATIC
  third_party/bspline_opt/src/uniform_bspline.cpp
  third_party/bspline_opt/src/bspline_optimizer.cpp
  third_party/bspline_opt/src/gradient_descent_optimizer.cpp)
target_include_directories(bspline_opt_internal PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bspline_opt/include
  ${planner_algorithm_include_dirs})
ament_target_dependencies(bspline_opt_internal rclcpp)
target_link_libraries(bspline_opt_internal Eigen3::Eigen path_searching_internal plan_env_internal)

add_library(traj_utils_internal STATIC
  third_party/traj_utils/src/planning_visualization.cpp
  third_party/traj_utils/src/polynomial_traj.cpp)
target_include_directories(traj_utils_internal PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/traj_utils/include
  ${planner_algorithm_include_dirs})
ament_target_dependencies(traj_utils_internal rclcpp std_msgs visualization_msgs)
target_link_libraries(traj_utils_internal Eigen3::Eigen bspline_opt_internal)

set(OCTO_PLANNER3D_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/OctoPlanner3D-ROS2"
  CACHE PATH "Path to the OctoPlanner3D-ROS2 third_party tree")
if(NOT EXISTS "${OCTO_PLANNER3D_ROOT}/planner/include/global_planner.h")
  message(FATAL_ERROR "OCTO_PLANNER3D_ROOT does not point to OctoPlanner3D-ROS2: ${OCTO_PLANNER3D_ROOT}")
endif()

set(PORTABLE_OCTOMAP_LIBRARY "${CMAKE_CURRENT_BINARY_DIR}/portable_lib/liboctomap.so.1.10")
set(PORTABLE_OCTOMATH_LIBRARY "${CMAKE_CURRENT_BINARY_DIR}/portable_lib/liboctomath.so.1.10")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/portable_lib")
configure_file("${OCTO_PLANNER3D_ROOT}/lib/liboctomap.so.1.10" "${PORTABLE_OCTOMAP_LIBRARY}" COPYONLY)
configure_file("${OCTO_PLANNER3D_ROOT}/lib/liboctomath.so.1.10" "${PORTABLE_OCTOMATH_LIBRARY}" COPYONLY)
file(RPATH_REMOVE FILE "${PORTABLE_OCTOMAP_LIBRARY}")
file(RPATH_REMOVE FILE "${PORTABLE_OCTOMATH_LIBRARY}")

add_library(octomap SHARED IMPORTED GLOBAL)
set_target_properties(octomap PROPERTIES
  IMPORTED_LOCATION "${PORTABLE_OCTOMAP_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${OCTO_PLANNER3D_ROOT}/octomap/include")
add_library(octomath SHARED IMPORTED GLOBAL)
set_target_properties(octomath PROPERTIES
  IMPORTED_LOCATION "${PORTABLE_OCTOMATH_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${OCTO_PLANNER3D_ROOT}/octomap/include")

add_library(octo_planner3d_core STATIC
  "${OCTO_PLANNER3D_ROOT}/octomap/src/pcd2octomap_converter.cpp"
  "${OCTO_PLANNER3D_ROOT}/planner/src/global_planner.cpp")
target_include_directories(octo_planner3d_core PUBLIC
  "${OCTO_PLANNER3D_ROOT}/octomap/include"
  "${OCTO_PLANNER3D_ROOT}/planner/include"
  ${PCL_INCLUDE_DIRS})
target_link_libraries(octo_planner3d_core octomap octomath ${PCL_LIBRARIES})
target_compile_definitions(octo_planner3d_core PUBLIC ${PCL_DEFINITIONS})

set(planner_runtime_dir "${CMAKE_CURRENT_SOURCE_DIR}/third_party/planner_runtime")

function(configure_scan_target target)
  target_include_directories(${target} PRIVATE
    include
    "${planner_runtime_dir}/include"
    ${planner_algorithm_include_dirs}
    "${OCTO_PLANNER3D_ROOT}/octomap/include"
    "${OCTO_PLANNER3D_ROOT}/planner/include")
  ament_target_dependencies(${target} ${planner_ros_dependencies})
  target_link_libraries(${target}
    Eigen3::Eigen
    plan_env_internal
    path_searching_internal
    bspline_opt_internal
    traj_utils_internal
    "${planner_typesupport_target}")
endfunction()

add_library(local_planner_runtime STATIC
  "${planner_runtime_dir}/src/scan_planner_node.cpp"
  "${planner_runtime_dir}/src/scan_replan_fsm.cpp"
  "${planner_runtime_dir}/src/planner_manager.cpp")
configure_scan_target(local_planner_runtime)

add_library(controller_runtime STATIC
  "${planner_runtime_dir}/src/closed_loop_controller.cpp")
configure_scan_target(controller_runtime)

add_library(global_planner_runtime STATIC
  "${planner_runtime_dir}/src/octo_global_planner_node.cpp")
target_include_directories(global_planner_runtime PRIVATE
  include
  "${OCTO_PLANNER3D_ROOT}/octomap/include"
  "${OCTO_PLANNER3D_ROOT}/planner/include")
ament_target_dependencies(global_planner_runtime
  rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs)
target_link_libraries(global_planner_runtime octo_planner3d_core)

add_library(cloud_preprocessor_runtime STATIC
  "${planner_runtime_dir}/src/cloud_preprocessor_node.cpp"
  third_party/cloud_processing/src/cloud.cpp
  third_party/cloud_processing/src/cloud_cut.cpp)
target_compile_features(cloud_preprocessor_runtime PRIVATE cxx_std_17)
target_include_directories(cloud_preprocessor_runtime PRIVATE include third_party/cloud_processing/include)
target_include_directories(cloud_preprocessor_runtime SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(cloud_preprocessor_runtime
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros pcl_conversions)
target_link_libraries(cloud_preprocessor_runtime ${PCL_LIBRARIES})

# One entry source, with each executable linked to its own node implementation.
function(add_planner_executable target factory runtime_library)
  add_executable(${target} src/main.cpp)
  target_include_directories(${target} PRIVATE include)
  target_compile_definitions(${target} PRIVATE PLANNER_NODE_FACTORY=planner::${factory})
  ament_target_dependencies(${target} rclcpp)
  target_link_libraries(${target} ${runtime_library})
endfunction()

add_planner_executable(scan_planner_node createLocalPlannerNode local_planner_runtime)
add_planner_executable(closed_loop_controller createControllerNode controller_runtime)
add_planner_executable(octo_global_planner_node createGlobalPlannerNode global_planner_runtime)
add_planner_executable(scan_planner_odom createCloudPreprocessorNode cloud_preprocessor_runtime)
set_target_properties(octo_global_planner_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}/portable_lib"
  INSTALL_RPATH "$ORIGIN"
  INSTALL_RPATH_USE_LINK_PATH FALSE)

install(TARGETS
  scan_planner_node
  closed_loop_controller
  octo_global_planner_node
  scan_planner_odom
  DESTINATION lib/${PROJECT_NAME})
install(FILES "${PORTABLE_OCTOMAP_LIBRARY}" "${PORTABLE_OCTOMATH_LIBRARY}" DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY third_party/planner_runtime/include/ DESTINATION include)
install(FILES third_party/planner_runtime/NOTICE.md
  DESTINATION share/${PROJECT_NAME}/third_party/planner_runtime)
install(DIRECTORY third_party/plan_env/include/ DESTINATION include)
install(DIRECTORY third_party/path_searching/include/ DESTINATION include)
install(DIRECTORY third_party/bspline_opt/include/ DESTINATION include)
install(DIRECTORY third_party/traj_utils/include/ DESTINATION include)
install(DIRECTORY third_party/cloud_processing/include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(FILES README.md SOURCE_NOTICE.md DESTINATION share/${PROJECT_NAME})
install(FILES "${OCTO_PLANNER3D_ROOT}/LICENSE"
  DESTINATION share/${PROJECT_NAME}/third_party/OctoPlanner3D-ROS2)
install(PROGRAMS
  scripts/base_link_odometry.py
  scripts/keypoint_recorder.py
  scripts/relocalization_bridge.py
  DESTINATION lib/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_grid_map_dynamic_obstacle_timeout
    third_party/plan_env/test/test_grid_map_dynamic_obstacle_timeout.cpp)
  target_link_libraries(test_grid_map_dynamic_obstacle_timeout plan_env_internal)

  ament_add_gtest(test_grid_map_sliding_inflation
    third_party/plan_env/test/test_grid_map_sliding_inflation.cpp)
  target_link_libraries(test_grid_map_sliding_inflation plan_env_internal)

  ament_add_gtest(test_dyn_a_star_open_set
    third_party/path_searching/test/test_dyn_a_star_open_set.cpp)
  target_include_directories(test_dyn_a_star_open_set PRIVATE ${planner_algorithm_include_dirs})
  target_link_libraries(test_dyn_a_star_open_set Eigen3::Eigen path_searching_internal plan_env_internal)
  ament_target_dependencies(test_dyn_a_star_open_set rclcpp)

  ament_add_gtest(test_uniform_bspline
    third_party/bspline_opt/test/test_uniform_bspline.cpp)
  target_include_directories(test_uniform_bspline PRIVATE ${planner_algorithm_include_dirs})
  target_link_libraries(test_uniform_bspline bspline_opt_internal Eigen3::Eigen)

  ament_add_gtest(test_polynomial_traj
    third_party/traj_utils/test/test_polynomial_traj.cpp)
  target_include_directories(test_polynomial_traj PRIVATE ${planner_algorithm_include_dirs})
  target_link_libraries(test_polynomial_traj traj_utils_internal Eigen3::Eigen)

  ament_add_gtest(test_planning_safety test/test_planning_safety.cpp
    "${planner_runtime_dir}/src/scan_replan_fsm.cpp"
    "${planner_runtime_dir}/src/planner_manager.cpp")
  configure_scan_target(test_planning_safety)

  add_executable(test_octo_global_planner_replan_gate
    test/test_octo_global_planner_replan_gate.cpp)
  target_include_directories(test_octo_global_planner_replan_gate PRIVATE
    include
    "${OCTO_PLANNER3D_ROOT}/octomap/include"
    "${OCTO_PLANNER3D_ROOT}/planner/include")
  target_link_libraries(test_octo_global_planner_replan_gate octo_planner3d_core)
  ament_target_dependencies(test_octo_global_planner_replan_gate
    rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs)
  set_target_properties(test_octo_global_planner_replan_gate PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}/portable_lib"
    INSTALL_RPATH "$ORIGIN"
    INSTALL_RPATH_USE_LINK_PATH FALSE)
  add_test(NAME octo_global_planner_replan_gate COMMAND test_octo_global_planner_replan_gate)

  add_executable(test_global_planner_preblocked_semantics
    test/test_global_planner_preblocked_semantics.cpp)
  target_include_directories(test_global_planner_preblocked_semantics PRIVATE
    "${OCTO_PLANNER3D_ROOT}/octomap/include"
    "${OCTO_PLANNER3D_ROOT}/planner/include")
  target_link_libraries(test_global_planner_preblocked_semantics octo_planner3d_core)
  set_target_properties(test_global_planner_preblocked_semantics PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}/portable_lib"
    INSTALL_RPATH "$ORIGIN"
    INSTALL_RPATH_USE_LINK_PATH FALSE)
  add_test(NAME global_planner_preblocked_semantics COMMAND test_global_planner_preblocked_semantics)

  add_executable(test_trusted_ground_corridor test/test_trusted_ground_corridor.cpp)
  target_include_directories(test_trusted_ground_corridor PRIVATE
    "${OCTO_PLANNER3D_ROOT}/octomap/include"
    "${OCTO_PLANNER3D_ROOT}/planner/include")
  target_link_libraries(test_trusted_ground_corridor octo_planner3d_core)
  set_target_properties(test_trusted_ground_corridor PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}/portable_lib"
    INSTALL_RPATH "$ORIGIN"
    INSTALL_RPATH_USE_LINK_PATH FALSE)
  add_test(NAME trusted_ground_corridor COMMAND test_trusted_ground_corridor)

  ament_add_pytest_test(test_base_link_odometry test/test_base_link_odometry.py)
  ament_add_pytest_test(test_relocalization_bridge test/test_relocalization_bridge.py)
  ament_add_pytest_test(test_navigation_bridge test/test_navigation_bridge.py TIMEOUT 45)
  ament_add_pytest_test(test_navigation_launch test/test_navigation_launch.py)
  ament_add_pytest_test(test_navigation_pipeline test/test_navigation_pipeline.py TIMEOUT 75)
  add_test(
    NAME test_cloud_frame
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cloud_frame.py $<TARGET_FILE:scan_planner_odom>)
  set_tests_properties(test_cloud_frame PROPERTIES TIMEOUT 45)
  add_launch_test(test/test_controller_freshness.py TIMEOUT 45)
  add_launch_test(test/test_planner_startup.py TIMEOUT 30)
  add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30)
  add_launch_test(test/test_controller_lateral_velocity.py TIMEOUT 40)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
