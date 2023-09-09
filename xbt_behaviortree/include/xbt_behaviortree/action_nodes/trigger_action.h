#include <ros/ros.h>
#include <behaviortree_ros/bt_service_node.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>

using namespace BT;

class TriggerAction: public RosServiceNode<std_srvs::Trigger>
{
public:
  TriggerAction( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<std_srvs::Trigger>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return  {};
  }

  bool sendRequest(RequestType& request) override
  {
    return true;
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    if( rep.success == true)
    {
      return NodeStatus::SUCCESS;
    }
    else{
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("TriggerService service failure: %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }
};