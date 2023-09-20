#include <ros/ros.h>
#include "behaviortree_ros/bt_async_behavior.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//TODO

using namespace BT;

class TransformPoseAction : public AsyncBehaviorBase
{
public:
  TransformPoseAction(const std::string& name, const NodeConfig& config)
    : AsyncBehaviorBase(name, config)
  { }

  static PortsList providedPorts()
  {
    return {
        InputPort<geometry_msgs::Pose>("input_pose"),
        InputPort<geometry_msgs::Vector3>("translation_xyz"),
        InputPort<geometry_msgs::Quaternion>("quaternion_xyzw"),
        OutputPort<geometry_msgs::Pose>("output_pose")
    };
  }

  BT::NodeStatus doWork() override
  {
    // Check if input_pose is available
    geometry_msgs::Pose input_pose;  
    if (!getInput("input_pose", input_pose))
    {
      ROS_ERROR("Missing required input [input_pose]");
      return NodeStatus::FAILURE;
    }

    // Set default values for translation and quaternion if not provided
    geometry_msgs::Vector3 translation_xyz;
    if (!getInput("translation_xyz", translation_xyz))
    {
      translation_xyz.x = 0.0;
      translation_xyz.y = 0.0;
      translation_xyz.z = 0.0;
    }

    geometry_msgs::Quaternion quaternion_xyzw;
    if (!getInput("quaternion_xyzw", quaternion_xyzw))
    {
      quaternion_xyzw.x = 0.0;
      quaternion_xyzw.y = 0.0;
      quaternion_xyzw.z = 0.0;
      quaternion_xyzw.w = 1.0;
    }

    // Apply quaternion rotation
    tf2::Transform tf_transform;
    tf_transform.setRotation(tf2::Quaternion(
        quaternion_xyzw.x,
        quaternion_xyzw.y,
        quaternion_xyzw.z,
        quaternion_xyzw.w
    ));

    // Apply translation
    tf_transform.setOrigin(tf2::Vector3(
        translation_xyz.x,
        translation_xyz.y,
        translation_xyz.z
    ));

    tf2::Transform input_tf_transform;
    tf2::fromMsg(input_pose, input_tf_transform);

    tf2::Transform output_tf_transform = input_tf_transform * tf_transform;

    // Convert the resulting transform back to a ROS message
    geometry_msgs::Pose output_pose;
    tf2::toMsg(output_tf_transform, output_pose);

    // Publish the transformed pose on the output port
    setOutput("output_pose", output_pose);

    return NodeStatus::SUCCESS;
  }
};
