#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// ULTRA MINIMAL DEBUG VERSION
class CoveragePathTrimmer : public BT::SyncActionNode
{
public:
  CoveragePathTrimmer(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    // ABSOLUTELY NOTHING in constructor
  }

  static BT::PortsList providedPorts()
  {
    // KEEP EXACTLY what your BT XML expects
    return {
      BT::InputPort<nav_msgs::msg::Path>("nav_path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("interrupted_pose"),
      BT::OutputPort<nav_msgs::msg::Path>("trimmed_path")
    };
  }
  
  BT::NodeStatus tick() override
  {
    // Just return SUCCESS with empty path
    nav_msgs::msg::Path empty_path;
    setOutput("trimmed_path", empty_path);
    
    return BT::NodeStatus::SUCCESS;
  }
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CoveragePathTrimmer>("CoveragePathTrimmer");
}