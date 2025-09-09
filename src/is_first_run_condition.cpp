#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"

class IsFirstRun : public BT::ConditionNode
{
public:
  IsFirstRun(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), first_run_(true)
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if (first_run_) {
      first_run_ = false;
      return BT::NodeStatus::SUCCESS; 
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  bool first_run_;
};

// Enregistrement
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<IsFirstRun>("IsFirstRun");
}