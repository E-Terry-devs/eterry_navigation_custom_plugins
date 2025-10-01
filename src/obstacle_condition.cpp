#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ObstacleCondition : public BT::ConditionNode {
public:
    ObstacleCondition(const std::string& name, const BT::NodeConfiguration& config) 
        : BT::ConditionNode(name, config), 
          obstacle_detected_(false),
          position_saved_(false)
    {
        node_ = rclcpp::Node::make_shared("obstacle_condition_bt_node");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS qos(10);
        qos.transient_local();

        obstacle_detected_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/obstacle_detected", qos,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                bool previous_state = obstacle_detected_;
                obstacle_detected_ = msg->data;
                
                RCLCPP_INFO(node_->get_logger(), "Obstacle detected: %s", 
                           obstacle_detected_ ? "TRUE" : "FALSE");
                
                // Sauvegarder la position quand l'obstacle apparaît
                if (obstacle_detected_ && !previous_state) {
                    saveRobotPosition();
                }
            });
    }

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("last_pose_before_obstacle"),
            BT::OutputPort<bool>("obstacle_triggered")
        };
    }

    BT::NodeStatus tick() override {
        rclcpp::spin_some(node_);
        
        if (obstacle_detected_) {
            RCLCPP_WARN(node_->get_logger(), "Obstacle detected! Interrupting navigation.");
            
            // Mettre à jour les ports de sortie
            if (position_saved_) {
                setOutput("last_pose_before_obstacle", last_pose_);
                setOutput("obstacle_triggered", true);
            }
            
            return BT::NodeStatus::SUCCESS;
        } else {
            // Reset le flag quand l'obstacle disparaît
            setOutput("obstacle_triggered", false);
        }
        
        return BT::NodeStatus::FAILURE;
    }

private:
    void saveRobotPosition() {
        try {
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            
            last_pose_.header.stamp = node_->now();
            last_pose_.header.frame_id = "map";
            last_pose_.pose.position.x = transform.transform.translation.x;
            last_pose_.pose.position.y = transform.transform.translation.y;
            last_pose_.pose.position.z = transform.transform.translation.z;
            last_pose_.pose.orientation = transform.transform.rotation;
            
            position_saved_ = true;
            
            RCLCPP_INFO(node_->get_logger(), 
                       "Position saved before obstacle: (%.2f, %.2f, %.2f)",
                       last_pose_.pose.position.x,
                       last_pose_.pose.position.y,
                       last_pose_.pose.position.z);
                       
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to save robot position: %s", ex.what());
            position_saved_ = false;
        }
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_detected_sub_;
    
    bool obstacle_detected_;
    bool position_saved_;
    geometry_msgs::msg::PoseStamped last_pose_;
};

// Register node
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<ObstacleCondition>("ObstacleCondition");
}