#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class SimplePathGenerator : public BT::AsyncActionNode
{
public:
  SimplePathGenerator(const std::string& name, const BT::NodeConfiguration& config)
  : BT::AsyncActionNode(name, config),
    node_(rclcpp::Node::make_shared("simple_path_generator")),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_pose"),
      BT::InputPort<int>("min_points", 10, "Minimum number of points"),  // Augmenté
      BT::InputPort<int>("max_points", 100, "Maximum number of points"), // Augmenté
      BT::InputPort<double>("points_per_meter", 5.0, "Points per meter of distance"), // Plus dense
      BT::InputPort<double>("curvature_distance", 1.5, "Distance for initial/final curvature"), // Augmenté pour 1.8m robot
      BT::InputPort<double>("min_turning_radius", 2.0, "Minimum turning radius (meters)"), // Nouveau paramètre
      BT::OutputPort<nav_msgs::msg::Path>("generated_path")
    };
  }

  BT::NodeStatus tick() override
  {
    // [Récupération des poses comme avant...]
    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getInput("input_pose", goal_pose)) {
      RCLCPP_ERROR(node_->get_logger(), "No input_pose provided");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_.lookupTransform(
          "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Could not get robot pose: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "map";
    start_pose.header.stamp = node_->now();
    start_pose.pose.position.x = transformStamped.transform.translation.x;
    start_pose.pose.position.y = transformStamped.transform.translation.y;
    start_pose.pose.position.z = transformStamped.transform.translation.z;
    start_pose.pose.orientation = transformStamped.transform.rotation;

    // Calculs de distance et d'angle
    double dx = goal_pose.pose.position.x - start_pose.pose.position.x;
    double dy = goal_pose.pose.position.y - start_pose.pose.position.y;
    double distance = std::hypot(dx, dy);
    double target_yaw = std::atan2(dy, dx);

    // Paramètres de configuration
    int min_points, max_points;
    double points_per_meter, curvature_dist, min_turning_radius;
    getInput("min_points", min_points);
    getInput("max_points", max_points);
    getInput("points_per_meter", points_per_meter);
    getInput("curvature_distance", curvature_dist);
    getInput("min_turning_radius", min_turning_radius);

    // Ajustement automatique de la courbure basé sur le rayon minimal
    double actual_curvature_dist = std::max(curvature_dist, min_turning_radius * 0.75);
    actual_curvature_dist = std::min(actual_curvature_dist, distance/2.0);

    // Calcul du nombre de points
    int num_points = static_cast<int>(min_points + distance * points_per_meter);
    num_points = std::clamp(num_points, min_points, max_points);

    // Conversion des orientations
    tf2::Quaternion start_quat, goal_quat;
    tf2::fromMsg(start_pose.pose.orientation, start_quat);
    tf2::fromMsg(goal_pose.pose.orientation, goal_quat);
    
    double roll, pitch, start_yaw, goal_yaw;
    tf2::Matrix3x3(start_quat).getRPY(roll, pitch, start_yaw);
    tf2::Matrix3x3(goal_quat).getRPY(roll, pitch, goal_yaw);

    // Normalisation des angles
    double yaw_diff = target_yaw - start_yaw;
    while (yaw_diff > M_PI) yaw_diff -= 2*M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2*M_PI;

    // Création du chemin
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = node_->now();

    for (int i = 0; i <= num_points; ++i)
    {
      double t = static_cast<double>(i) / num_points;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;

      // Position - interpolation linéaire
      pose.pose.position.x = start_pose.pose.position.x + t * dx;
      pose.pose.position.y = start_pose.pose.position.y + t * dy;
      pose.pose.position.z = start_pose.pose.position.z + t * (goal_pose.pose.position.z - start_pose.pose.position.z);

      // Orientation - avec courbure adaptée
      double current_dist = t * distance;
      tf2::Quaternion interp_quat;
      
      if (current_dist < actual_curvature_dist) {
        // Courbure initiale (clothoïde simplifiée)
        double curve_t = current_dist / actual_curvature_dist;
        double eased_t = 0.5 - 0.5 * std::cos(curve_t * M_PI); // Easing pour fluidité
        double interp_yaw = start_yaw + eased_t * yaw_diff;
        interp_quat.setRPY(0, 0, interp_yaw);
      } 
      else if (current_dist > distance - actual_curvature_dist) {
        // Courbure finale
        double curve_t = (current_dist - (distance - actual_curvature_dist)) / actual_curvature_dist;
        double eased_t = 0.5 - 0.5 * std::cos(curve_t * M_PI);
        double interp_yaw = target_yaw + eased_t * (goal_yaw - target_yaw);
        interp_quat.setRPY(0, 0, interp_yaw);
      }
      else {
        // Section droite
        interp_quat.setRPY(0, 0, target_yaw);
      }

      pose.pose.orientation = tf2::toMsg(interp_quat);
      path_msg.poses.push_back(pose);
    }

    setOutput("generated_path", path_msg);
    RCLCPP_INFO(node_->get_logger(), "Generated smooth path for large robot (%.1fm) with %d points", 
                min_turning_radius, num_points);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SimplePathGenerator>("SimplePathGenerator");
}