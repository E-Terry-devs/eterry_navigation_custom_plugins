#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/action_node.h"

class WaitForButtonRelease : public BT::StatefulActionNode {
public:
    WaitForButtonRelease(const std::string& name, const BT::NodeConfiguration& config) 
        : BT::StatefulActionNode(name, config),
          button_pressed_(false),
          button_was_pressed_(false) {
        node_ = rclcpp::Node::make_shared("wait_for_button_release_bt_node");
        
        button_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/unloading_button", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                button_pressed_ = msg->data;
                RCLCPP_DEBUG(node_->get_logger(), "Button state: %s", 
                            button_pressed_ ? "PRESSED" : "RELEASED");
            });
    }

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<int>("wait_timeout", 10, "Timeout in seconds"),
            BT::InputPort<bool>("wait_for_false", false, "Wait for button=false instead of timeout")
        };
    }

    BT::NodeStatus onStart() override {
        getInput("wait_timeout", wait_timeout_);
        getInput("wait_for_false", wait_for_false_);
        
        timeout_start_ = node_->now();
        button_was_pressed_ = false; // Reset à chaque début
        
        if (wait_for_false_) {
            RCLCPP_INFO(node_->get_logger(), "Waiting for button to be released (press and release required)...");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Waiting at loading station for %d seconds...", wait_timeout_);
        }
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        rclcpp::spin_some(node_);
        
        if (wait_for_false_) {
            // Mode: Attendre que l'utilisateur relâche ACTIVEMENT le bouton
            if (button_pressed_) {
                button_was_pressed_ = true; // L'utilisateur a pressé le bouton
                RCLCPP_DEBUG(node_->get_logger(), "Button pressed, waiting for release...");
            }
            
            // SUCCESS seulement si le bouton était pressé et est maintenant relâché
            if (!button_pressed_ && button_was_pressed_) {
                RCLCPP_INFO(node_->get_logger(), "Button released, resuming navigation");
                return BT::NodeStatus::SUCCESS;
            }
        } else {
            // Mode: Attendre le timeout
            auto elapsed = node_->now() - timeout_start_;
            if (elapsed.seconds() >= wait_timeout_) {
                RCLCPP_INFO(node_->get_logger(), "Wait completed after %d seconds", wait_timeout_);
                return BT::NodeStatus::SUCCESS;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        RCLCPP_INFO(node_->get_logger(), "Waiting interrupted");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_;
    bool button_pressed_ = false;
    bool button_was_pressed_ = false; // Nouvelle variable pour suivre l'état précédent
    int wait_timeout_ = 10;
    bool wait_for_false_ = false;
    rclcpp::Time timeout_start_;
};

// Register node
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<WaitForButtonRelease>("WaitForButtonRelease");
}