#include "behaviortree_cpp/behavior_tree.h"
#include <behaviortree_ros/bt_action_node.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <control_msgs/GripperCommandResult.h>
#include <control_msgs/GripperCommandFeedback.h>

using namespace BT;

class GripperAction : public RosActionNode<control_msgs::GripperCommandAction>
{
public:
  GripperAction(ros::NodeHandle &handle, const std::string &name, const NodeConfiguration &conf)
    : RosActionNode<control_msgs::GripperCommandAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return {
        InputPort<double>("position"),
        InputPort<double>("max_effort")};
  }

  bool sendGoal(GoalType &goal) override
  {
    if (!getInput<double>("position", goal.command.position))
    {
      ROS_ERROR("Missing required input [position]");
      return false;
    }

    if (!getInput<double>("max_effort", goal.command.max_effort))
    {
      ROS_ERROR("Missing required input [max_effort]");
      return false;
    }

    return true;
  }

  NodeStatus onResult(const ResultType &res) override
  {
    return NodeStatus::SUCCESS;
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("GripperAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }
};
