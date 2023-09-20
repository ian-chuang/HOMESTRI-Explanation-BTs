#include <ros/ros.h>
#include "behaviortree_ros/bt_async_behavior.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>

using namespace BT;

class DetectFrameAction : public AsyncBehaviorBase
{
public:
  // Define a constant default timeout value.
  static constexpr double DefaultTimeout = 1.0;
  static constexpr double DefaultWaitDuration = 0.0;

  DetectFrameAction(const std::string &name, const NodeConfiguration &conf)
      : AsyncBehaviorBase(name, conf)
  {
  }

  static PortsList providedPorts()
  {
    return {
        InputPort<std::string>("target_frame"),
        InputPort<std::string>("source_frame"),
        OutputPort<geometry_msgs::Pose>("output_pose"),
        InputPort<unsigned>("wait_duration", DetectFrameAction::DefaultWaitDuration, "wait for given duration before getting frame (milliseconds)"),
        InputPort<unsigned>("timeout", DetectFrameAction::DefaultTimeout, "timeout to lookup transform (milliseconds)")};
  }

  BT::NodeStatus doWork() override
  {
    std::string target_frame;
    if (!getInput<std::string>("target_frame", target_frame))
    {
      ROS_ERROR("Missing required input [target_frame]");
      return BT::NodeStatus::FAILURE;
    }

    std::string source_frame;
    if (!getInput<std::string>("source_frame", source_frame))
    {
      ROS_ERROR("Missing required input [source_frame]");
      return BT::NodeStatus::FAILURE;
    }

    unsigned msec;
    if (!getInput<unsigned>("timeout", msec))
    {
      ROS_ERROR("Missing required input [timeout]");
      return NodeStatus::FAILURE;
    }
    ros::Duration timeout(static_cast<double>(msec) * 1e-3);

    if (!getInput<unsigned>("wait_duration", msec))
    {
      ROS_ERROR("Missing required input [wait_duration]");
      return NodeStatus::FAILURE;
    }
    ros::Duration wait_duration(static_cast<double>(msec) * 1e-3);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // wait for given duration before getting frame
    ros::Duration(wait_duration).sleep();

    try
    {
      auto transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), timeout);
      geometry_msgs::Pose pose;
      pose.position.x = transformStamped.transform.translation.x;
      pose.position.y = transformStamped.transform.translation.y;
      pose.position.z = transformStamped.transform.translation.z;
      pose.orientation.x = transformStamped.transform.rotation.x;
      pose.orientation.y = transformStamped.transform.rotation.y;
      pose.orientation.z = transformStamped.transform.rotation.z;
      pose.orientation.w = transformStamped.transform.rotation.w;

      setOutput("output_pose", pose);

      return NodeStatus::SUCCESS;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("DetectFrameAction: %s", ex.what());
      return NodeStatus::FAILURE;
    }
  }
};
