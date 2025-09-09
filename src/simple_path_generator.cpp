#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

class SimplePathGenerator : public BT::AsyncActionNode
{
public:
  SimplePathGenerator(const std::string &name, const BT::NodeConfiguration &config)
  : BT::AsyncActionNode(name, config),
    node_(rclcpp::Node::make_shared("simple_path_generator")),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("generated_path", 5);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "input_pose", "Target pose to reach"),
      BT::InputPort<int>(
        "min_points", 20, "Minimum number of points in generated path"),
      BT::InputPort<int>(
        "max_points", 150, "Maximum number of points in generated path"),
      BT::InputPort<double>(
        "points_per_meter", 5.0, "Number of generated points per meter"),
      BT::InputPort<double>(
        "curvature_distance", 1.5, "Distance for curvature control (m)"),
      BT::InputPort<double>(
        "curvature_intensity", 0.5, "Curvature intensity factor (0.0-1.0)"),
      BT::OutputPort<nav_msgs::msg::Path>(
        "generated_path", "Output generated path")
    };
  }

  BT::NodeStatus tick() override
  {
    // Récupération de la pose cible
    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getInput("input_pose", goal_pose))
    {
      RCLCPP_ERROR(node_->get_logger(), "SimplePathGenerator: missing input_pose");
      return BT::NodeStatus::FAILURE;
    }

    // Paramètres avec valeurs par défaut
    int min_points = 20;
    int max_points = 150;
    double points_per_meter = 5.0;
    double curvature_dist = 1.5;
    double curvature_intensity = 0.5;

    getInput("min_points", min_points);
    getInput("max_points", max_points);
    getInput("points_per_meter", points_per_meter);
    getInput("curvature_distance", curvature_dist);
    getInput("curvature_intensity", curvature_intensity);

    // Obtenir la pose actuelle du robot
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      transformStamped = tf_buffer_.lookupTransform(
          "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.5));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "SimplePathGenerator: Could not get robot pose: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "map";
    start_pose.header.stamp = node_->now();
    start_pose.pose.position.x = transformStamped.transform.translation.x;
    start_pose.pose.position.y = transformStamped.transform.translation.y;
    start_pose.pose.position.z = transformStamped.transform.translation.z;
    start_pose.pose.orientation = transformStamped.transform.rotation;

    // Transformer la pose cible si nécessaire
    if (goal_pose.header.frame_id != "map" && !goal_pose.header.frame_id.empty())
    {
      try
      {
        geometry_msgs::msg::TransformStamped goal_transform = tf_buffer_.lookupTransform(
            "map", goal_pose.header.frame_id, goal_pose.header.stamp, tf2::durationFromSec(0.5));
        
        tf2::Transform transform;
        tf2::fromMsg(goal_transform.transform, transform);
        
        tf2::Vector3 position(goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
        tf2::Quaternion orientation;
        tf2::fromMsg(goal_pose.pose.orientation, orientation);
        
        tf2::Vector3 transformed_pos = transform * position;
        tf2::Quaternion transformed_orient = transform * orientation;
        
        goal_pose.pose.position.x = transformed_pos.x();
        goal_pose.pose.position.y = transformed_pos.y();
        goal_pose.pose.position.z = transformed_pos.z();
        goal_pose.pose.orientation = tf2::toMsg(transformed_orient);
        goal_pose.header.frame_id = "map";
        
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(node_->get_logger(), "SimplePathGenerator: Goal pose transform failed: %s", ex.what());
        return BT::NodeStatus::FAILURE;
      }
    }

    // Calculer la distance et la direction
    double dx = goal_pose.pose.position.x - start_pose.pose.position.x;
    double dy = goal_pose.pose.position.y - start_pose.pose.position.y;
    double dz = goal_pose.pose.position.z - start_pose.pose.position.z;
    double distance = std::hypot(dx, dy);

    // Extraire les orientations
    tf2::Quaternion start_quat, goal_quat;
    tf2::fromMsg(start_pose.pose.orientation, start_quat);
    tf2::fromMsg(goal_pose.pose.orientation, goal_quat);
    
    double roll, pitch, start_yaw, goal_yaw;
    tf2::Matrix3x3(start_quat).getRPY(roll, pitch, start_yaw);
    tf2::Matrix3x3(goal_quat).getRPY(roll, pitch, goal_yaw);

    // Calculer le nombre de points
    int num_points = static_cast<int>(min_points + distance * points_per_meter);
    num_points = std::clamp(num_points, min_points, max_points);

    // Calcul de la direction cible à partir du vecteur
    double target_yaw = std::atan2(dy, dx);
    double yaw_diff = normalizeAngle(target_yaw - start_yaw);

    // Calcul du point de contrôle Bézier
    double ux = dx / distance;
    double uy = dy / distance;
    
    // Vecteur perpendiculaire pour la courbure
    double perp_x = -uy;
    double perp_y = ux;
    
    // Intensité de la courbure basée sur l'angle et la distance
    double curvature_factor = curvature_intensity * std::sin(yaw_diff) * distance;
    
    // Point de contrôle Bézier
    double ctrl_x = start_pose.pose.position.x + curvature_dist * ux + curvature_factor * perp_x;
    double ctrl_y = start_pose.pose.position.y + curvature_dist * uy + curvature_factor * perp_y;

    // Génération du chemin Bézier
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = node_->now();
    path_msg.poses.reserve(num_points + 1);

    for (int i = 0; i <= num_points; ++i)
    {
      double t = static_cast<double>(i) / num_points;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;

      // Courbe de Bézier quadratique
      double omt = 1.0 - t;
      pose.pose.position.x = omt * omt * start_pose.pose.position.x +
                           2 * omt * t * ctrl_x +
                           t * t * goal_pose.pose.position.x;
      
      pose.pose.position.y = omt * omt * start_pose.pose.position.y +
                           2 * omt * t * ctrl_y +
                           t * t * goal_pose.pose.position.y;
      
      pose.pose.position.z = start_pose.pose.position.z + t * dz;

      // Interpolation d'orientation (smooth)
      double interp_yaw;
      if (distance < 0.5) {
        // Pour les courtes distances, interpolation directe
        interp_yaw = start_yaw + t * normalizeAngle(goal_yaw - start_yaw);
      } else {
        // Pour les longues distances, suivre la direction du chemin
        if (t < 0.3) {
          interp_yaw = start_yaw + (t/0.3) * yaw_diff;
        } else if (t > 0.7) {
          interp_yaw = target_yaw + ((t-0.7)/0.3) * normalizeAngle(goal_yaw - target_yaw);
        } else {
          interp_yaw = target_yaw;
        }
      }

      tf2::Quaternion orientation;
      orientation.setRPY(0, 0, interp_yaw);
      pose.pose.orientation = tf2::toMsg(orientation);

      path_msg.poses.push_back(pose);
    }

    // Sortie et publication
    setOutput("generated_path", path_msg);
    path_pub_->publish(path_msg);

    RCLCPP_INFO(node_->get_logger(), 
                "SimplePathGenerator: Generated Bézier path with %d points, distance: %.2fm", 
                num_points, distance);
    
    return BT::NodeStatus::SUCCESS;
  }

private:
  double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SimplePathGenerator>("SimplePathGenerator");
}