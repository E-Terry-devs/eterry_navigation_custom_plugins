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
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(eterry_navigation_custom_interfaces REQUIRED)


# --- Find your eterry_vs package ---
find_package(eterry_vs REQUIRED)



# --- Build vision_navigation_action_bt_node BT node ---
add_library(vision_navigation_action_bt_node SHARED src/vision_navigation_action.cpp)
ament_target_dependencies(vision_navigation_action_bt_node
    behaviortree_cpp_v3
    rclcpp
    geometry_msgs
    std_msgs
    OpenCV
    eterry_vs  # Votre package eterry_vs
)

# Inclure les répertoires
target_include_directories(vision_navigation_action_bt_node PRIVATE
    ${eterry_vs_INCLUDE_DIRS}
)

# Définition pour l'export du plugin
target_compile_definitions(vision_navigation_action_bt_node PRIVATE "BT_PLUGIN_EXPORT")




# --- Build is_vision_available_bt_node BT node ---
add_library(is_vision_available_bt_node SHARED src/is_vision_available.cpp)
ament_target_dependencies(is_vision_available_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  nav_msgs
  nav2_msgs
)
target_compile_definitions(is_vision_available_bt_node PRIVATE "BT_PLUGIN_EXPORT")




# --- Build in_row_condition_bt_node BT node ---
add_library(in_row_condition_bt_node SHARED src/in_row_condition.cpp)
ament_target_dependencies(in_row_condition_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  eterry_navigation_custom_interfaces
  nav_msgs
  nav2_msgs
)
target_compile_definitions(in_row_condition_bt_node PRIVATE "BT_PLUGIN_EXPORT")




# --- Build wait_for_button_release BT node ---
add_library(wait_for_button_release_bt_node SHARED src/wait_for_button_release.cpp)
ament_target_dependencies(wait_for_button_release_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
)
target_compile_definitions(wait_for_button_release_bt_node PRIVATE "BT_PLUGIN_EXPORT")

# --- Build stop_vision_controller BT node ---
add_library(stop_vision_controller_bt_node SHARED src/stop_vision_controller.cpp)
ament_target_dependencies(stop_vision_controller_bt_node
  rclcpp
  geometry_msgs
  nav2_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
)
target_include_directories(stop_vision_controller_bt_node PUBLIC
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
  ${eterry_vs_INCLUDE_DIRS}
)
target_compile_definitions(stop_vision_controller_bt_node PRIVATE "BT_PLUGIN_EXPORT")

# --- Build stop_gps_controller BT node ---
add_library(stop_gps_controller_bt_node SHARED src/stop_gps_controller.cpp)
ament_target_dependencies(stop_gps_controller_bt_node
  rclcpp
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
)
target_include_directories(stop_gps_controller_bt_node PUBLIC
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)
target_compile_definitions(stop_gps_controller_bt_node PRIVATE "BT_PLUGIN_EXPORT")

# --- Build go_to_loading_station BT node ---
add_library(go_to_loading_station_bt_node SHARED src/go_to_loading_station.cpp)
ament_target_dependencies(go_to_loading_station_bt_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  tinyxml2
  nav2_msgs
)
target_include_directories(go_to_loading_station_bt_node PUBLIC
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)
target_link_libraries(go_to_loading_station_bt_node
  ${tinyxml2_LIBRARIES}
)
target_compile_definitions(go_to_loading_station_bt_node PRIVATE "BT_PLUGIN_EXPORT")

# --- Build IsFirstRun BT node ---
add_library(is_first_run_condition_bt_node SHARED src/is_first_run_condition.cpp)
ament_target_dependencies(is_first_run_condition_bt_node
  behaviortree_cpp_v3
)
target_include_directories(is_first_run_condition_bt_node PUBLIC
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
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
target_include_directories(coverage_path_trimmer PUBLIC
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)
target_compile_definitions(coverage_path_trimmer PRIVATE "BT_PLUGIN_EXPORT")

# --- Build PauseCondition BT node ---
add_library(pause_condition_bt_node SHARED src/pause_condition.cpp)
ament_target_dependencies(pause_condition_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
)
target_include_directories(pause_condition_bt_node PUBLIC
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)
target_compile_definitions(pause_condition_bt_node PRIVATE "BT_PLUGIN_EXPORT")

# --- Build unloading_condition BT node ---
add_library(unloading_condition_bt_node SHARED src/unloading_condition.cpp)
ament_target_dependencies(unloading_condition_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)
target_include_directories(unloading_condition_bt_node PUBLIC
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)
target_compile_definitions(unloading_condition_bt_node PRIVATE "BT_PLUGIN_EXPORT")

# --- Build obstacle_condition BT node ---
add_library(obstacle_condition_bt_node SHARED src/obstacle_condition.cpp)
ament_target_dependencies(obstacle_condition_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)
target_include_directories(obstacle_condition_bt_node PUBLIC
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)
target_compile_definitions(obstacle_condition_bt_node PRIVATE "BT_PLUGIN_EXPORT")

# --- Build unloading_button_trigger BT node ---
add_library(unloading_button_trigger_bt_node SHARED src/unloading_button_trigger.cpp)
ament_target_dependencies(unloading_button_trigger_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)
target_include_directories(unloading_button_trigger_bt_node PUBLIC
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)
target_compile_definitions(unloading_button_trigger_bt_node PRIVATE "BT_PLUGIN_EXPORT")

# --- Install ---
install(TARGETS 
  wait_for_button_release_bt_node
  go_to_loading_station_bt_node
  unloading_condition_bt_node
  is_first_run_condition_bt_node
  in_row_condition_bt_node
  simple_path_generator 
  pause_condition_bt_node
  obstacle_condition_bt_node
  unloading_button_trigger_bt_node 
  stop_gps_controller_bt_node
  stop_vision_controller_bt_node
  
  coverage_path_trimmer
  is_vision_available_bt_node
  vision_navigation_action_bt_node
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()