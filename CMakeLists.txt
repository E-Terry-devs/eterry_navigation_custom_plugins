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
find_package(tinyxml2 REQUIRED)  # AJOUT IMPORTANT


# --- Build go_to_loading_station BT node ---
add_library(go_to_loading_station_bt_node SHARED src/go_to_loading_station.cpp)

ament_target_dependencies(go_to_loading_station_bt_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  tinyxml2  # AJOUT
)

target_link_libraries(go_to_loading_station_bt_node
  ${tinyxml2_LIBRARIES}  # AJOUT
)

target_compile_definitions(go_to_loading_station_bt_node PRIVATE "BT_PLUGIN_EXPORT")

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

# --- Build unloading_condition BT node ---
add_library(unloading_condition_bt_node SHARED src/unloading_condition.cpp)
ament_target_dependencies(unloading_condition_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs  # Ajoutez cette ligne
  nav_msgs       
)
target_compile_definitions(unloading_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)


# --- Build Unloading_condition BT node ---
add_library(obstacle_condition_bt_node SHARED src/obstacle_condition.cpp)
ament_target_dependencies(obstacle_condition_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs  # Ajoutez cette ligne
  nav_msgs       
)
target_compile_definitions(obstacle_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)



# --- Build Unloading_condition BT node ---
add_library(unloading_button_trigger_bt_node SHARED src/unloading_button_trigger.cpp)
ament_target_dependencies(unloading_button_trigger_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs  # Ajoutez cette ligne
  nav_msgs       
)
target_compile_definitions(unloading_button_trigger_bt_node PRIVATE BT_PLUGIN_EXPORT)


# --- Build wait_for_button_release_bt_node BT node ---
add_library(wait_for_button_release_bt_node SHARED src/wait_for_button_release.cpp)
ament_target_dependencies(wait_for_button_release_bt_node
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs  # Ajoutez cette ligne
  nav_msgs       
)
target_compile_definitions(wait_for_button_release_bt_node PRIVATE BT_PLUGIN_EXPORT)


# --- Install ---
install(TARGETS 
  wait_for_button_release_bt_node
  go_to_loading_station_bt_node
  unloading_condition_bt_node
  is_first_run_condition_bt_node
  simple_path_generator 
  pause_condition_bt_node
  unloading_button_trigger_bt_node 
  coverage_path_trimmer
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()