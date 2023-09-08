#include "behaviortree_cpp/behavior_tree.h"
#include <behaviortree_ros/bt_action_node.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace BT;

class JointTrajectoryAction : public RosActionNode<control_msgs::FollowJointTrajectoryAction>
{
public:
  JointTrajectoryAction(ros::NodeHandle &handle, const std::string &name, const NodeConfiguration &conf)
    : RosActionNode<control_msgs::FollowJointTrajectoryAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return {
        InputPort<trajectory_msgs::JointTrajectory>("joint_trajectory"),
    };
  }

  bool sendGoal(GoalType &goal) override
  {
    if (!getInput<trajectory_msgs::JointTrajectory>("joint_trajectory", goal.trajectory)) {
        ROS_ERROR("Missing required input [joint_trajectory]");
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
    ROS_ERROR("JointTrajectoryAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if (status() == NodeStatus::RUNNING)
    {
      ROS_WARN("JointTrajectoryAction halted");
      BaseClass::halt();
    }
  }
};
