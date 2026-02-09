#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class StopGPSController : public BT::SyncActionNode
{
public:
    StopGPSController(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        node_ = std::make_shared<rclcpp::Node>("stop_gps_controller_bt");
        follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
            node_, "follow_path");
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(), "Stopping GPS controller...");
        
        try {
            bool success = true;
            
            if (follow_path_client_->action_server_is_ready()) {
                RCLCPP_INFO(node_->get_logger(), "Cancelling FollowPath action...");
                auto future_cancel = follow_path_client_->async_cancel_all_goals();
                
                if (rclcpp::spin_until_future_complete(node_, future_cancel, 500ms) !=
                    rclcpp::FutureReturnCode::SUCCESS) {
                    RCLCPP_WARN(node_->get_logger(), "Failed to cancel FollowPath action");
                    success = false;
                }
            }
            
            auto zero_vel = geometry_msgs::msg::Twist();
            zero_vel.linear.x = 0.0;
            zero_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(zero_vel);
            
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(100ms);
            rclcpp::spin_some(node_);
            
            cmd_vel_pub_->publish(zero_vel);
            std::this_thread::sleep_for(50ms);
            rclcpp::spin_some(node_);
            
            if (success) {
                RCLCPP_INFO(node_->get_logger(), "GPS controller stopped successfully");
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_WARN(node_->get_logger(), "GPS controller stopped with warnings");
                return BT::NodeStatus::SUCCESS;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Error stopping GPS controller: %s", e.what());
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<StopGPSController>("StopGPSController");
}