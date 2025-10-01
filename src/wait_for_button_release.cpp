#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/action_node.h"

class WaitForButtonRelease : public BT::StatefulActionNode {
public:
    WaitForButtonRelease(const std::string& name, const BT::NodeConfiguration& config) 
        : BT::StatefulActionNode(name, config) {
        node_ = rclcpp::Node::make_shared("wait_for_button_release_bt_node");
        
        // Topic : /button_unreleased
        button_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/button_unreleased", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                button_unreleased_ = msg->data;
                RCLCPP_DEBUG(node_->get_logger(), "Received /button_unreleased: %s", 
                            button_unreleased_ ? "true" : "false");
            });
    }

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<int>("wait_timeout", 0, "Timeout in seconds (0 = no timeout)")
        };
    }

    BT::NodeStatus onStart() override {
        // Récupérer le timeout (optionnel)
        wait_timeout_ = 0;
        getInput("wait_timeout", wait_timeout_);
        
        if (wait_timeout_ > 0) {
            timeout_start_ = node_->now();
            RCLCPP_INFO(node_->get_logger(), "Waiting for /button_unreleased == true (timeout: %d seconds)...", wait_timeout_);
        } else {
            RCLCPP_INFO(node_->get_logger(), "Waiting for /button_unreleased == true (no timeout)...");
        }
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        rclcpp::spin_some(node_);
        
        // Vérifier le timeout
        if (wait_timeout_ > 0) {
            auto elapsed = node_->now() - timeout_start_;
            if (elapsed.seconds() >= wait_timeout_) {
                RCLCPP_WARN(node_->get_logger(), "Timeout reached after %d seconds, continuing anyway...", wait_timeout_);
                return BT::NodeStatus::SUCCESS;
            }
        }
        
        // SUCCESS quand /button_unreleased == true
        if (button_unreleased_) {
            RCLCPP_INFO(node_->get_logger(), "Button is released (/button_unreleased = true), continuing...");
            return BT::NodeStatus::SUCCESS;
        }
        
        // RUNNING tant que /button_unreleased est false
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        RCLCPP_INFO(node_->get_logger(), "Waiting for /button_unreleased interrupted");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_;
    bool button_unreleased_ = false;
    int wait_timeout_ = 0;
    rclcpp::Time timeout_start_;
};

// Register node
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<WaitForButtonRelease>("WaitForButtonRelease");
}