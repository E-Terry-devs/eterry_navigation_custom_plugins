#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"

class IsPathEmpty : public BT::ConditionNode
{
public:
  IsPathEmpty(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<nav_msgs::msg::Path>("path") };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    if (!getInput("path", path)) {
      // Si on ne peut pas lire le chemin, on considère qu'il est vide par sécurité
      return BT::NodeStatus::SUCCESS;
    }
    
    // Si le chemin a moins de 2 poses, on le considère comme vide/inutilisable
    if (path.poses.size() < 2) {
      return BT::NodeStatus::SUCCESS;  // Chemin vide ou trop court
    }
    
    return BT::NodeStatus::FAILURE;     // Chemin non vide
  }
};






// Plugin registration
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<IsPathEmpty>("IsPathEmpty");
}

