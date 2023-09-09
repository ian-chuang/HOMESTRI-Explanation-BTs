#include <ros/ros.h>
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
        InputPort<double>("max_effort"),
        OutputPort<double>("position"),
        OutputPort<double>("effort"),
        OutputPort<bool>("stalled"),
        OutputPort<bool>("reached_goal"),
    };
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
    setOutput<double>("position", res.position);
    setOutput<double>("effort", res.effort);
    setOutput<bool>("stalled", res.stalled);
    setOutput<bool>("reached_goal", res.reached_goal);

    if (res.reached_goal || res.stalled)
    {
      return NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("GripperAction action server failure: %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }
};
