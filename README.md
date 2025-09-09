# eterry_navigation_custom_plugins
A ROS 2 package providing Behavior Tree nodes for autonomous coverage path planning and navigation in agricultural environments.
##🌟 Features
Coverage Path Planning: Complete field coverage algorithms for agricultural robots

Behavior Tree Nodes: Custom BT nodes for ROS 2 Navigation2 framework

Path Generation: Smooth path generation with curvature control

Pause/Resume: Intelligent navigation interruption and resumption

First-Run Handling: Smart initialization that only goes to start position once

###📦 Included Nodes
🎯 Core Navigation Nodes
SimplePathGenerator: Generates smooth paths with configurable curvature

CoveragePathTrimmer: Trims already-covered paths for efficient resumption

PauseCondition: Handles navigation pause/resume functionality

IsFirstRun: Condition node for one-time initialization sequences

####🔧 Technical Specifications
ROS 2 Humble compatible

BehaviorTree.CPP v3 integration

10Hz navigation rate for smooth operation

Configurable path parameters: points per meter, curvature, min/max points

#####🏗️ Architecture
<img src="https://github.com/hediiimohamed/eterry_navigation_custom_plugins/blob/main/img/eterry_navigation_Bt.png" width="1000" alt="E-terry Navigation BT">


Behavior Tree Structure
The package implements a sophisticated BT that:

First run: Navigates to optimal start position

Subsequent runs: Resumes coverage from last position

Pause/Resume: Handles interruptions gracefully

Efficient coverage: Minimizes redundant path traversal

🎯 Use Cases
Agricultural robotics: Field coverage and crop monitoring

Autonomous mowing: Lawn and turf management

Industrial cleaning: Large area coverage algorithms

Search patterns: Systematic area exploration


###### Build Instructions :
 Clone the repository
 
```bash
git clone https://github.com/your-username/bt_simple_path_generator.git
```

 -Build with colcon
```bash
colcon build --packages-select bt_simple_path_generator
source install/setup.bash
```
####### YAML Configuration Example :
bt_loop_duration: 50
default_server_timeout: 20
navigators: ['navigate_complete_coverage']

navigate_complete_coverage:
  plugin: "opennav_coverage_navigator/CoverageNavigator"

plugin_lib_names:
  - nav2_follow_path_action_bt_node
  - nav2_rate_controller_bt_node
  - simple_path_generator
  - coverage_path_trimmer
  - pause_condition_bt_node
  - is_first_run_condition_bt_node
  
######## Behavior Tree Structure
The package implements a sophisticated BT that:

1/First run: Navigates to optimal start position

2/Subsequent runs: Resumes coverage from last position

3/Pause/Resume: Handles interruptions gracefully

4/Efficient coverage: Minimizes redundant path


