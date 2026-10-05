#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <cmath>

// Shifts every pose of the incoming path sideways by a small amount, so the
// robot can be nudged left/right to re-align with a physical crop row while
// FollowPath keeps running (this node is re-ticked every cycle from a
// ReactiveSequence, so the shift is re-applied continuously without ever
// stopping/canceling the FollowPath goal).
//
// Sign convention matches SwathFactory::generateOffsetSwaths in
// row_swath_generator.cpp: perpendicular = (-dy, dx)/len, so a positive
// offset shifts LEFT of the direction of travel, negative shifts RIGHT.
class LateralOffsetCorrector : public BT::SyncActionNode
{
public:
  LateralOffsetCorrector(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config), offset_(0.0)
  {
    node_ = rclcpp::Node::make_shared("lateral_offset_corrector_node");

    rclcpp::QoS qos(10);
    qos.transient_local();

    offset_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      "/row_alignment/lateral_offset", qos,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        offset_ = msg->data;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("nav_path"),
      BT::OutputPort<nav_msgs::msg::Path>("offset_path")
    };
  }

  BT::NodeStatus tick() override
  {
    // Process any pending offset updates published since the last tick
    rclcpp::spin_some(node_);

    nav_msgs::msg::Path input_path;
    if (!getInput("nav_path", input_path)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing required input [nav_path]");
      return BT::NodeStatus::FAILURE;
    }

    if (std::fabs(offset_) < 1e-4 || input_path.poses.size() < 2) {
      setOutput("offset_path", input_path);
      return BT::NodeStatus::SUCCESS;
    }

    nav_msgs::msg::Path offset_path = input_path;
    const size_t n = input_path.poses.size();

    for (size_t i = 0; i < n; ++i) {
      // Central difference for the local tangent, forward/backward at the ends
      size_t prev = (i == 0) ? 0 : i - 1;
      size_t next = (i == n - 1) ? n - 1 : i + 1;

      double dx = input_path.poses[next].pose.position.x - input_path.poses[prev].pose.position.x;
      double dy = input_path.poses[next].pose.position.y - input_path.poses[prev].pose.position.y;
      double len = std::sqrt(dx * dx + dy * dy);

      if (len < 1e-6) {
        continue;
      }

      double perp_x = -dy / len;
      double perp_y = dx / len;

      offset_path.poses[i].pose.position.x = input_path.poses[i].pose.position.x + perp_x * offset_;
      offset_path.poses[i].pose.position.y = input_path.poses[i].pose.position.y + perp_y * offset_;
    }

    setOutput("offset_path", offset_path);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr offset_sub_;
  double offset_;
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<LateralOffsetCorrector>("LateralOffsetCorrector");
}
