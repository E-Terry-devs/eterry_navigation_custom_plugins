#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <vector>

class ReversePathTrimmer : public BT::SyncActionNode
{
public:
  ReversePathTrimmer(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("reverse_path_trimmer_node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Publisher pour visualisation
    reverse_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/reverse_path", 10);
    original_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/original_path", 10);
    
    RCLCPP_INFO(node_->get_logger(), "ReversePathTrimmer initialized");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("nav_path"),
      BT::OutputPort<nav_msgs::msg::Path>("trimmed_path")
    };
  }
  
  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(node_->get_logger(), "=== ReversePathTrimmer ACTIVE ===");
    
    nav_msgs::msg::Path input_path;
    if (!getInput("nav_path", input_path)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing nav_path input");
      return BT::NodeStatus::FAILURE;
    }
    
    if (input_path.poses.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Input path is empty");
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Original path has %zu points", input_path.poses.size());
    
    // Récupérer la position actuelle du robot
    geometry_msgs::msg::PoseStamped current_pose;
    if (!getCurrentRobotPose(current_pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Cannot get current robot pose");
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Current robot pose: (%.2f, %.2f)", 
                current_pose.pose.position.x, current_pose.pose.position.y);
    
    // Publier chemin original
    original_path_pub_->publish(input_path);
    
    // Créer le chemin de retour (marche arrière)
    nav_msgs::msg::Path return_path;
    return_path.header = input_path.header;
    return_path.header.frame_id = "map";
    return_path.header.stamp = node_->now();
    
    // 1. POINT CRITIQUE : Ajouter la position actuelle du robot en premier
    return_path.poses.push_back(current_pose);
    
    // 2. Ajouter tous les points PARCOURUS (de la position du robot jusqu'au début)
    // Pour cela, il faut trouver où est le robot dans le chemin original
    size_t robot_index = findNearestPoseIndex(current_pose, input_path);
    RCLCPP_INFO(node_->get_logger(), "Robot is at index %zu in original path", robot_index);
    
    // 3. Ajouter les points du chemin original de robot_index jusqu'à 0 (en ordre inverse)
    for (int i = robot_index; i >= 0; i--) {
      geometry_msgs::msg::PoseStamped pose = input_path.poses[i];
      pose.header.frame_id = "map";
      pose.header.stamp = node_->now();
      // NE PAS CHANGER L'ORIENTATION - le robot recule naturellement
      return_path.poses.push_back(pose);
    }
    
    RCLCPP_INFO(node_->get_logger(), "Return path has %zu points (from robot to start)", 
                return_path.poses.size());
    
    // Afficher les premiers points pour debug
    if (return_path.poses.size() > 1) {
      RCLCPP_INFO(node_->get_logger(), "First point (robot): (%.2f, %.2f)",
                  return_path.poses[0].pose.position.x,
                  return_path.poses[0].pose.position.y);
      RCLCPP_INFO(node_->get_logger(), "Second point: (%.2f, %.2f)",
                  return_path.poses[1].pose.position.x,
                  return_path.poses[1].pose.position.y);
    }
    
    // Publier chemin de retour pour visualisation
    reverse_path_pub_->publish(return_path);
    
    setOutput("trimmed_path", return_path);
    
    return BT::NodeStatus::SUCCESS;
  }
  
private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reverse_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr original_path_pub_;
  
  bool getCurrentRobotPose(geometry_msgs::msg::PoseStamped& pose) {
    try {
      auto transform = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));
      
      pose.header.frame_id = "map";
      pose.header.stamp = node_->now();
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return true;
    } catch (tf2::TransformException& e) {
      RCLCPP_ERROR(node_->get_logger(), "TF error: %s", e.what());
      return false;
    }
  }
  
  size_t findNearestPoseIndex(const geometry_msgs::msg::PoseStamped& pose,
                               const nav_msgs::msg::Path& path) {
    size_t best_idx = 0;
    double best_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
      double dx = pose.pose.position.x - path.poses[i].pose.position.x;
      double dy = pose.pose.position.y - path.poses[i].pose.position.y;
      double dist = std::sqrt(dx*dx + dy*dy);
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = i;
      }
      if (dist < 0.2) break;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Nearest index %zu, distance %.2f", best_idx, best_dist);
    return best_idx;
  }
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ReversePathTrimmer>("ReversePathTrimmer");
}