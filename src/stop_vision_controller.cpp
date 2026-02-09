#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

// ADD THESE BEHAVIOR TREE INCLUDES
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <memory>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// FIX: Use correct namespace for BT::NodeConfiguration
class StopVisionController : public BT::SyncActionNode
{
public:
    // FIX: Changed BT::NodeConfig to BT::NodeConfiguration
    StopVisionController(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        node_ = std::make_shared<rclcpp::Node>("stop_vision_controller_bt");
        stop_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/vision/stop", 10);
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        vision_active_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/vision/active", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                vision_active_ = msg->data;
            });
    }

    // FIX: Changed return type from BT::PortsList to BT::PortsList
    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(), "Stopping vision controller...");
        
        try {
            auto stop_msg = std_msgs::msg::Empty();
            stop_pub_->publish(stop_msg);
            
            auto zero_vel = geometry_msgs::msg::Twist();
            zero_vel.linear.x = 0.0;
            zero_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(zero_vel);
            
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(100ms);
            rclcpp::spin_some(node_);
            
            if (vision_active_) {
                RCLCPP_WARN(node_->get_logger(), "Vision still active, sending second stop");
                stop_pub_->publish(stop_msg);
                std::this_thread::sleep_for(50ms);
                rclcpp::spin_some(node_);
            }
            
            RCLCPP_INFO(node_->get_logger(), "Vision controller stopped successfully");
            return BT::NodeStatus::SUCCESS;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Error stopping vision controller: %s", e.what());
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_active_sub_;
    bool vision_active_ = false;
};

// Register the node
// No need to include bt_factory.h again since we included it at the top
// Register node
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<StopVisionController>("StopVisionController");
}