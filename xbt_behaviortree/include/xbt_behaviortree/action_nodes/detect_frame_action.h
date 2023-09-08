#include "behaviortree_cpp/behavior_tree.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>

using namespace BT;

class DetectFrameAction : public StatefulActionNode
{
public:
  // Define a constant default timeout value.
  static constexpr double DEFAULT_TIMEOUT = 0.02;

  DetectFrameAction(const std::string &name, const NodeConfiguration &conf)
      : StatefulActionNode(name, conf)
  {
  }

  static PortsList providedPorts()
  {
    return {
        InputPort<std::string>("target_frame"),
        InputPort<std::string>("source_frame"),
        OutputPort<geometry_msgs::Pose>("output_pose"),
        InputPort<int>("max_tries"),
        InputPort<double>("timeout")
    };
  }

  NodeStatus onStart() override
  {
    // Retrieve target_frame and source_frame only once.
    if (!getInput<std::string>("target_frame", target_frame_))
    {
      throw RuntimeError("missing port [target_frame]");
    }

    if (!getInput<std::string>("source_frame", source_frame_))
    {
      throw RuntimeError("missing port [source_frame]");
    }

    if (!getInput<int>("max_tries", max_tries_))
    {
      max_tries_ = 1; // Default to 1 try if max_tries is not set.
    }

    num_tries_ = 0;

    // Try to get a custom timeout; otherwise, use the default.
    if (getInput<double>("timeout", timeout_))
    {
      if (timeout_ <= 0)
      {
        throw RuntimeError("Invalid timeout value. It must be greater than zero.");
      }
    }
    else
    {
      timeout_ = DEFAULT_TIMEOUT; // Use the default timeout.
    }

    // Create tfBuffer and tfListener here to reuse them in onRunning.
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>();
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    return NodeStatus::RUNNING;
  }

  NodeStatus onRunning() override
  {
    if (num_tries_ < max_tries_)
    {
      try
      {
        transformStamped_ = tfBuffer_->lookupTransform(target_frame_, source_frame_, ros::Time(0), ros::Duration(timeout_));
        geometry_msgs::Pose pose;
        pose.position.x = transformStamped_.transform.translation.x;
        pose.position.y = transformStamped_.transform.translation.y;
        pose.position.z = transformStamped_.transform.translation.z;
        pose.orientation.x = transformStamped_.transform.rotation.x;
        pose.orientation.y = transformStamped_.transform.rotation.y;
        pose.orientation.z = transformStamped_.transform.rotation.z;
        pose.orientation.w = transformStamped_.transform.rotation.w;

        setOutput("output_pose", pose);

        return NodeStatus::SUCCESS;
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        num_tries_++; // Increment the number of tries.
        return NodeStatus::RUNNING;
      }
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

  void onHalted() override
  {
    // Clean up if necessary when the node is interrupted.
    // For example, you can cancel ongoing transformations here.
  }

private:
  int num_tries_;
  int max_tries_;
  double timeout_;
  std::string target_frame_;
  std::string source_frame_;
  geometry_msgs::TransformStamped transformStamped_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
};
