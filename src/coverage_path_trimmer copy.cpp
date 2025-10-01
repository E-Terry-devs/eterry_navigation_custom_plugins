#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <mutex>

class CoveragePathTrimmer : public BT::SyncActionNode
{
public:
  CoveragePathTrimmer(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("coverage_path_trimmer_node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
    // Get input path
    nav_msgs::msg::Path input_path;
    if (!getInput("nav_path", input_path)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing required input [nav_path]");
      return BT::NodeStatus::FAILURE;
    }

    if (input_path.poses.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Input path is empty");
      return BT::NodeStatus::FAILURE;
    }

    // Get robot pose
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      RCLCPP_DEBUG(node_->get_logger(), "Failed to get robot pose, using full path");
      setOutput("trimmed_path", input_path);
      return BT::NodeStatus::SUCCESS;
    }

    // Find next waypoint
    size_t next_idx = findNextWaypoint(robot_pose, input_path);
    
    if (next_idx >= input_path.poses.size()) {
      RCLCPP_INFO(node_->get_logger(), "Reached end of path");
      return BT::NodeStatus::FAILURE;
    }

    // Trim path from the next waypoint
    nav_msgs::msg::Path trimmed_path;
    trimmed_path.header = input_path.header;
    trimmed_path.poses.assign(input_path.poses.begin() + next_idx, input_path.poses.end());

    setOutput("trimmed_path", trimmed_path);

    RCLCPP_DEBUG(node_->get_logger(), "Trimmed path: %zu -> %zu waypoints", 
                input_path.poses.size(), trimmed_path.poses.size());

    return BT::NodeStatus::SUCCESS;
  }

private:
  bool getRobotPose(geometry_msgs::msg::PoseStamped& pose) const
  {
    try {
      // Use non-blocking check first
      if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero, 
                                  tf2::durationFromSec(0.1))) {
        return false;
      }

      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero);

      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG(node_->get_logger(), "TF exception: %s", ex.what());
      return false;
    }
  }

  size_t findNextWaypoint(const geometry_msgs::msg::PoseStamped& robot_pose, 
                         const nav_msgs::msg::Path& path) const
  {
    size_t nearest_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
      double distance = pointDistance(robot_pose.pose.position, path.poses[i].pose.position);
      
      if (distance < min_distance) {
        min_distance = distance;
        nearest_idx = i;
      }
      
      if (min_distance < 0.1) {
        break;
      }
    }

    // Start from the next waypoint if close enough
    if (min_distance < 0.5 && nearest_idx + 1 < path.poses.size()) {
      return nearest_idx + 1;
    }
    
    return nearest_idx;
  }

  double pointDistance(const geometry_msgs::msg::Point& p1, 
                     const geometry_msgs::msg::Point& p2) const
  {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx*dx + dy*dy);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CoveragePathTrimmer>("CoveragePathTrimmer");
}