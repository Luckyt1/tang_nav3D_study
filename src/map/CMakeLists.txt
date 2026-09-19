cmake_minimum_required(VERSION 3.16)
project(map)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs photo)
find_package(Threads REQUIRED)

add_library(freedom_core STATIC
  third_party/freedom/src/freedom.cpp
  third_party/freedom/src/map.cpp
  third_party/freedom/src/scanmap.cpp
  third_party/freedom/src/mrmap.cpp
  third_party/freedom/src/raycast.cpp
  third_party/freedom/src/depth_image.cpp
)
target_compile_features(freedom_core PUBLIC cxx_std_17)
target_include_directories(freedom_core SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/freedom/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(freedom_core PUBLIC ${PCL_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen Threads::Threads)

add_library(freedom_processing STATIC third_party/mapping/src/freedom.cpp)
target_include_directories(freedom_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/mapping/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(freedom_processing PUBLIC freedom_core)
target_compile_options(freedom_processing PRIVATE -Wall -Wextra -Wpedantic)

add_library(btc_core STATIC third_party/btc/src/btc.cpp)
target_compile_features(btc_core PUBLIC cxx_std_17)
target_include_directories(btc_core SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/btc/include>
  $<INSTALL_INTERFACE:include> ${PCL_INCLUDE_DIRS})
target_link_libraries(btc_core PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen)

add_library(submap_processing STATIC third_party/mapping/src/submap.cpp)
target_compile_features(submap_processing PUBLIC cxx_std_17)
target_include_directories(submap_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/mapping/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(submap_processing SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(submap_processing PUBLIC btc_core ${PCL_LIBRARIES} Eigen3::Eigen)
target_compile_options(submap_processing PRIVATE -Wall -Wextra -Wpedantic)

add_executable(map_node src/map_node.cpp)
ament_target_dependencies(map_node rclcpp sensor_msgs geometry_msgs std_srvs tf2 tf2_ros pcl_conversions)
target_link_libraries(map_node freedom_processing submap_processing)
target_compile_options(map_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS map_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS freedom_core freedom_processing btc_core submap_processing
  EXPORT map_processing_targets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY third_party/mapping/include/
  third_party/freedom/include/ third_party/btc/include/
  DESTINATION include)
ament_export_targets(map_processing_targets HAS_LIBRARY_TARGET)
# Load Eigen before PCL/VTK extend CMAKE_MODULE_PATH with legacy find modules.
ament_export_dependencies(Eigen3 PCL OpenCV Threads)
install(FILES README.md third_party/freedom/NOTICE.md third_party/freedom/LICENSE
  DESTINATION share/${PROJECT_NAME})
install(FILES third_party/btc/NOTICE.md DESTINATION share/${PROJECT_NAME}/btc)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_freedom test/test_freedom.cpp)
  target_link_libraries(test_freedom freedom_processing)
  ament_add_gtest(test_submap test/test_submap.cpp)
  target_link_libraries(test_submap submap_processing)
  ament_add_gtest(test_btc test/test_btc.cpp)
  target_link_libraries(test_btc submap_processing)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME map_node_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_map_node.py $<TARGET_FILE:map_node>)
  set_tests_properties(map_node_integration PROPERTIES TIMEOUT 90)
endif()

ament_package()
