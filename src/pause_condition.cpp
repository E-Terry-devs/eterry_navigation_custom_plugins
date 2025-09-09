#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

class PauseCondition : public BT::ConditionNode
{
public:
    PauseCondition(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), paused_(false)
    {
        node_ = rclcpp::Node::make_shared("pause_condition_bt_node");
        
        rclcpp::QoS qos(10);
        qos.transient_local();

        pause_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/pause", qos,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                paused_ = msg->data;
            });
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        // Process any pending callbacks
        rclcpp::spin_some(node_);
        
        return paused_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
    bool paused_;

    // Add these declarations:
    static rclcpp::Node::SharedPtr static_node;
    static rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr static_pause_sub;
    static std::thread static_spin_thread;
    static bool static_paused;
    static std::mutex static_paused_mutex;
};
// Initialize static members
rclcpp::Node::SharedPtr PauseCondition::static_node = nullptr;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr PauseCondition::static_pause_sub = nullptr;
std::thread PauseCondition::static_spin_thread;
bool PauseCondition::static_paused = false;
std::mutex PauseCondition::static_paused_mutex;

// Register node
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<PauseCondition>("PauseCondition");
}