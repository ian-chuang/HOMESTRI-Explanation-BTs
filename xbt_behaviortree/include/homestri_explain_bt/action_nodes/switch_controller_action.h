#include "behaviortree_cpp/behavior_tree.h"
#include <behaviortree_ros/bt_service_node.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>

using namespace BT;

class SwitchControllerAction : public RosServiceNode<controller_manager_msgs::SwitchController>
{
private:
  const static inline std::vector<std::string> ConflictingControllers = {
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
    "joint_group_vel_controller",
    "twist_controller",
    "cartesian_motion_controller",
    "cartesian_force_controller",
    "cartesian_compliance_controller"
  };

public:
  SwitchControllerAction(ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration& conf)
    : RosServiceNode<controller_manager_msgs::SwitchController>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return {
      InputPort<std::string>("controller")
    };
  }

  void sendRequest(RequestType& request) override
  {
    std::string controller;
    if (!getInput<std::string>("controller", controller))
    {
      ROS_ERROR("Missing required input [controller]");
      return;
    }

    std::vector<std::string> other_controllers = SwitchControllerAction::ConflictingControllers;
    other_controllers.erase(std::remove(other_controllers.begin(), other_controllers.end(), controller), other_controllers.end());

    request.start_controllers = {controller};
    request.stop_controllers = other_controllers;
    request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    if (rep.ok)
    {
      return NodeStatus::SUCCESS;
    }
    else
    {
      ROS_ERROR("SwitchControllerAction: Failed to switch controllers");
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("SwitchControllerAction: Request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }
};
