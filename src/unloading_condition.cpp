#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"

// Variable globale pour Nav2
static bool g_unloading_flag = false;

// Node pour SET le flag unloading (quand on va à la station)
class IsUnloading : public BT::ConditionNode
{
public:
  IsUnloading(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    if (!g_unloading_flag) {
      g_unloading_flag = true;
      return BT::NodeStatus::SUCCESS; 
    }
    return BT::NodeStatus::FAILURE;
  }
};

// Node pour CHECK si on vient de retourner de unloading
class IsUnloadingReturn : public BT::ConditionNode
{
public:
  IsUnloadingReturn(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    return g_unloading_flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// Node pour RESET le flag (après avoir géré le retour)
class ResetUnloading : public BT::ConditionNode
{
public:
  ResetUnloading(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    if (g_unloading_flag) {
      g_unloading_flag = false;
      return BT::NodeStatus::SUCCESS; 
    }
    return BT::NodeStatus::FAILURE;
  }
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<IsUnloading>("IsUnloading");
  factory.registerNodeType<IsUnloadingReturn>("IsUnloadingReturn");
  factory.registerNodeType<ResetUnloading>("ResetUnloading");
}