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
        InputPort<double>("x"), 
        InputPort<double>("y"), 
        InputPort<double>("z"), 
        InputPort<double>("qx"),
        InputPort<double>("qy"),
        InputPort<double>("qz"),
        InputPort<double>("qw"),
        OutputPort<double>("output_x"),
        OutputPort<double>("output_y"),
        OutputPort<double>("output_z"),
        OutputPort<double>("output_qx"),
        OutputPort<double>("output_qy"),
        OutputPort<double>("output_qz"),
        OutputPort<double>("output_qw"),
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

    geometry_msgs::Pose output_pose;
    try
    {
      auto transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), timeout);
      output_pose.position.x = transformStamped.transform.translation.x;
      output_pose.position.y = transformStamped.transform.translation.y;
      output_pose.position.z = transformStamped.transform.translation.z;
      output_pose.orientation.x = transformStamped.transform.rotation.x;
      output_pose.orientation.y = transformStamped.transform.rotation.y;
      output_pose.orientation.z = transformStamped.transform.rotation.z;
      output_pose.orientation.w = transformStamped.transform.rotation.w;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("DetectFrameAction: %s", ex.what());
      return NodeStatus::FAILURE;
    }   

    // Overwrite individual components if optional inputs are provided
    double x;
    if (getInput("x", x))
    {
      output_pose.position.x = x;
    }

    double y;
    if (getInput("y", y))
    {
      output_pose.position.y = y;
    }

    double z;
    if (getInput("z", z))
    {
      output_pose.position.z = z;
    }

    double qx;
    if (getInput("qx", qx))
    {
      output_pose.orientation.x = qx;
    }

    double qy;
    if (getInput("qy", qy))
    {
      output_pose.orientation.y = qy;
    }

    double qz;
    if (getInput("qz", qz))
    {
      output_pose.orientation.z = qz;
    }

    double qw;
    if (getInput("qw", qw))
    {
      output_pose.orientation.w = qw;
    }

    setOutput("output_x", output_pose.position.x);
    setOutput("output_y", output_pose.position.y);
    setOutput("output_z", output_pose.position.z);
    setOutput("output_qx", output_pose.orientation.x);
    setOutput("output_qy", output_pose.orientation.y);
    setOutput("output_qz", output_pose.orientation.z);
    setOutput("output_qw", output_pose.orientation.w);
    setOutput("output_pose", output_pose);

    return NodeStatus::SUCCESS;

  }
};
