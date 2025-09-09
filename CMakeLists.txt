cmake_minimum_required(VERSION 3.5)
project(eterry_navigation_custom_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# --- Build IsFirstRun BT node ---
add_library(is_first_run_condition_bt_node SHARED src/is_first_run_condition.cpp)
ament_target_dependencies(is_first_run_condition_bt_node
  behaviortree_cpp_v3
)
target_compile_definitions(is_first_run_condition_bt_node PRIVATE "BT_PLUGIN_EXPORT")

# --- Build Simple Path Generator BT node ---
add_library(simple_path_generator SHARED src/simple_path_generator.cpp)
ament_target_dependencies(simple_path_generator
  rclcpp
  geometry_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_compile_definitions(simple_path_generator PRIVATE "BT_PLUGIN_EXPORT")

# --- Build CoveragePathTrimmer BT node ---
add_library(coverage_path_trimmer SHARED src/coverage_path_trimmer.cpp)
ament_target_dependencies(coverage_path_trimmer
  rclcpp
  geometry_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_compile_definitions(coverage_path_trimmer PRIVATE "BT_PLUGIN_EXPORT")

# --- Build PauseCondition BT node ---
add_library(pause_condition_bt_node SHARED src/pause_condition.cpp)
ament_target_dependencies(pause_condition_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
)
target_compile_definitions(pause_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)

# --- Install ---
install(TARGETS 
  is_first_run_condition_bt_node  # <-- Ajoutez cette ligne
  simple_path_generator 
  pause_condition_bt_node 
  coverage_path_trimmer
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()