#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

class IsVisionAvailable : public BT::ConditionNode
{
public:
    IsVisionAvailable(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), vision_available_(false)
    {
        static std::atomic<int> counter{0};
        node_name_ = "is_vision_available_" + std::to_string(counter.fetch_add(1));
        
        node_ = rclcpp::Node::make_shared(node_name_);
        
        // QoS compatible
        rclcpp::QoS qos(10);
        qos.reliable();  // Gardez reliable si le publisher l'utilise
        
        vision_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/vision_available", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                vision_available_.store(msg->data);
                RCLCPP_INFO(node_->get_logger(), 
                           "👁️ Vision status: %s", 
                           msg->data ? "AVAILABLE" : "UNAVAILABLE");
            });
            
        RCLCPP_INFO(node_->get_logger(), 
                   "IsVisionAvailable initialized");
    }

    static BT::PortsList providedPorts()
    {
        return {};  // Aucun port nécessaire pour cette condition
    }

    BT::NodeStatus tick() override
    {
        rclcpp::spin_some(node_);
        return vision_available_.load() ? 
               BT::NodeStatus::SUCCESS : 
               BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_sub_;
    std::string node_name_;
    std::atomic<bool> vision_available_;
};

// Plugin registration
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<IsVisionAvailable>("IsVisionAvailable");
}