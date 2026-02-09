#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;

class IsVisionAvailable : public BT::ConditionNode
{
public:
    IsVisionAvailable(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config),
          vision_available_(false),
          last_msg_time_(builtin_interfaces::msg::Time()),
          max_age_duration_(builtin_interfaces::msg::Duration())
    {
        // Initialize ROS node if not already done
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // Create a ROS node
        node_ = std::make_shared<rclcpp::Node>("is_vision_available_bt_node");
        
        // Get parameters
        double max_age_seconds = 0.5; // default
        if (auto max_age = getInput<double>("max_age")) {
            max_age_seconds = max_age.value();
        }
        max_age_duration_.sec = static_cast<int32_t>(max_age_seconds);
        max_age_duration_.nanosec = static_cast<uint32_t>((max_age_seconds - max_age_duration_.sec) * 1e9);
    }

    ~IsVisionAvailable() {
        subscription_.reset();
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("max_age", 0.5, "Maximum age of vision data in seconds") };
    }

    BT::NodeStatus tick() override
    {
        // Initialize subscription on first tick
        if (!subscription_) {
            subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
                "/vision_available",
                10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    vision_available_ = msg->data;
                    last_msg_time_ = node_->now();
                });
        }
        
        // Process any pending messages
        rclcpp::spin_some(node_);
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Check if we have received any message
        if (last_msg_time_.sec == 0 && last_msg_time_.nanosec == 0) {
            // No message received yet
            return BT::NodeStatus::FAILURE;
        }
        
        // Check if the data is too old
        auto now = node_->now();
        
        // Calculate age manually
        int64_t now_nsec = static_cast<int64_t>(now.seconds()) * 1000000000LL + now.nanoseconds();
        int64_t msg_nsec = static_cast<int64_t>(last_msg_time_.sec) * 1000000000LL + last_msg_time_.nanosec;
        int64_t max_age_nsec = static_cast<int64_t>(max_age_duration_.sec) * 1000000000LL + max_age_duration_.nanosec;
        
        if ((now_nsec - msg_nsec) > max_age_nsec) {
            // Data is too old
            return BT::NodeStatus::FAILURE;
        }
        
        // Return based on vision availability
        return vision_available_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    std::mutex mutex_;
    bool vision_available_;
    builtin_interfaces::msg::Time last_msg_time_;
    builtin_interfaces::msg::Duration max_age_duration_;
};

// Plugin registration
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

// Enregistrement du plugin
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<IsVisionAvailable>("IsVisionAvailable");
}