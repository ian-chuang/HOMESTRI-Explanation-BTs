#include "behaviortree_cpp/behavior_tree.h"
#include <behaviortree_ros/bt_action_node.h>
#include <compliant_trajectory_control/FollowCompliantTrajectoryAction.h>
#include <compliant_trajectory_control/FollowCompliantTrajectoryGoal.h>
#include <compliant_trajectory_control/FollowCompliantTrajectoryResult.h>
#include <compliant_trajectory_control/FollowCompliantTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace BT;

class CompliantTrajectoryAction : public RosActionNode<compliant_trajectory_control::FollowCompliantTrajectoryAction>
{
public:
  CompliantTrajectoryAction(ros::NodeHandle &handle, const std::string &name, const NodeConfiguration &conf)
    : RosActionNode<compliant_trajectory_control::FollowCompliantTrajectoryAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return {
        InputPort<trajectory_msgs::JointTrajectory>("joint_trajectory"),
        InputPort<geometry_msgs::Wrench>("wrench"),
        InputPort<std::string>("wrench_frame_id"),
    };
  }

  bool sendGoal(GoalType &goal) override
  {
    if (!getInput<trajectory_msgs::JointTrajectory>("joint_trajectory", goal.joint_trajectory)) {
      ROS_ERROR("Missing required input [joint_trajectory]");
      return false;
    }

    if (!getInput<geometry_msgs::Wrench>("wrench", goal.wrench.wrench)) {
      ROS_ERROR("Missing required input [wrench]");
      return false;
    }

    if (!getInput<std::string>("wrench_frame_id", goal.wrench.header.frame_id)) {
      ROS_ERROR("Missing required input [wrench_frame_id]");
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
};
