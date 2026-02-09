#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "eterry_navigation_custom_interfaces/msg/coverage_navigation_status.hpp"
#include <memory>
#include <mutex>

using namespace BT;

class InRowConditionSimple : public ConditionNode
{
public:
    InRowConditionSimple(const std::string& name, const NodeConfiguration& config)
        : ConditionNode(name, config)
    {
        // SOLUTION: Créer son propre node ROS au lieu de demander node_handle
        // Chaque instance a un nom unique
        static int instance_counter = 0;
        std::string node_name = std::string("in_row_condition_") + std::to_string(instance_counter++);
        
        // Vérifier si ROS est initialisé
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // Créer le node avec options
        rclcpp::NodeOptions options;
        options.allow_undeclared_parameters(true);
        options.automatically_declare_parameters_from_overrides(true);
        
        ros_node_ = std::make_shared<rclcpp::Node>(node_name, options);
        
        RCLCPP_INFO(ros_node_->get_logger(), 
                   " InRowConditionSimple créé: %s", name.c_str());
        
        // Créer le subscriber
        status_sub_ = ros_node_->create_subscription<
            eterry_navigation_custom_interfaces::msg::CoverageNavigationStatus>(
            "ET01_C_N/coverage_navigation_status", 
            10,
            [this](const eterry_navigation_custom_interfaces::msg::CoverageNavigationStatus::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                current_row_ = msg->current_row;
                navigation_state_ = msg->navigation_state;
                
                // Debug occasionnel
                static int msg_count = 0;
                if (msg_count++ % 10 == 0) {
                    RCLCPP_DEBUG(ros_node_->get_logger(), 
                               " Row: %d, State: %s", 
                               current_row_, navigation_state_.c_str());
                }
            });
        
        RCLCPP_INFO(ros_node_->get_logger(), 
                   " Subscribed to: ET01_C_N/coverage_navigation_status");
        
        // Lancer l'exécuteur dans un thread séparé
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(ros_node_);
        
        spin_thread_ = std::thread([this]() {
            try {
                executor_->spin();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(ros_node_->get_logger(), 
                           "Executor exception: %s", e.what());
            }
        });
    }
    
    ~InRowConditionSimple()
    {
        RCLCPP_INFO(ros_node_->get_logger(), 
                   "  Nettoyage InRowConditionSimple");
        
        if (executor_) {
            executor_->cancel();
        }
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }
    
    static PortsList providedPorts()
    {
        return { 
            InputPort<bool>("invert", false, "Invert condition (true = check for turning)"),
            OutputPort<int>("current_row", "Current row number"),
            OutputPort<std::string>("navigation_state", "Current navigation state")
        };
    }
    
    NodeStatus tick() override
    {
        if (!ros_node_) {
            return NodeStatus::FAILURE;
        }
        
        bool invert = false;
        getInput("invert", invert);
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Set outputs
        setOutput("current_row", current_row_);
        setOutput("navigation_state", navigation_state_);
        
        bool is_in_row = (current_row_ > 0);
        
        // Apply inversion if requested
        bool result = invert ? !is_in_row : is_in_row;
        
        // Log occasionnel
        static int tick_count = 0;
        if (tick_count++ % 20 == 0) {
            RCLCPP_INFO(ros_node_->get_logger(),
                       "17: InRowCondition: Row=%d, Invert=%s, Result=%s",
                       current_row_, 
                       invert ? "true" : "false",
                       result ? "SUCCESS" : "FAILURE");
        }
        
        return result ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<eterry_navigation_custom_interfaces::msg::CoverageNavigationStatus>::SharedPtr status_sub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
    
    int current_row_ = 0;
    std::string navigation_state_ = "";
    std::mutex mutex_;
};
// Register node
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<InRowConditionSimple>("InRowCondition");
}