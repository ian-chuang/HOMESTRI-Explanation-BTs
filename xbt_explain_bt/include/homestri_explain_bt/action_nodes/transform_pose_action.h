#include "behaviortree_cpp/behavior_tree.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h> // Include tf2::Transform
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace BT;

class TransformPoseAction : public SyncActionNode
{
public:
  TransformPoseAction(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config)
  { }

  static PortsList providedPorts()
  {
    return {
        InputPort<geometry_msgs::PoseStamped>("input_pose"),
        InputPort<geometry_msgs::Vector3>("translation_xyz"),
        InputPort<geometry_msgs::Quaternion>("quaternion_xyzw"),
        OutputPort<geometry_msgs::PoseStamped>("output_pose")
    };
  }

  NodeStatus tick() override
  {
    // Check if input_pose is available
    if (!getInput("input_pose", input_pose_))
    {
      throw RuntimeError("Missing required port [input_pose]");
    }

    // Set default values for translation and quaternion if not provided
    if (!getInput("translation_xyz", translation_xyz_))
    {
      translation_xyz_.x = 0.0;
      translation_xyz_.y = 0.0;
      translation_xyz_.z = 0.0;
    }

    if (!getInput("quaternion_xyzw", quaternion_xyzw_))
    {
      quaternion_xyzw_.x = 0.0;
      quaternion_xyzw_.y = 0.0;
      quaternion_xyzw_.z = 0.0;
      quaternion_xyzw_.w = 1.0;
    }

    // Perform the transformation
    geometry_msgs::PoseStamped output_pose;
    output_pose.header = input_pose_.header; // Copy the header from input_pose

    // Apply quaternion rotation
    tf2::Transform tf_transform;
    tf_transform.setRotation(tf2::Quaternion(
        quaternion_xyzw_.x,
        quaternion_xyzw_.y,
        quaternion_xyzw_.z,
        quaternion_xyzw_.w
    ));

    // Apply translation
    tf_transform.setOrigin(tf2::Vector3(
        translation_xyz_.x,
        translation_xyz_.y,
        translation_xyz_.z
    ));

    tf2::Transform input_tf_transform;
    tf2::fromMsg(input_pose_.pose, input_tf_transform);

    tf2::Transform output_tf_transform = input_tf_transform * tf_transform;

    // Convert the resulting transform back to a ROS message
    tf2::toMsg(output_tf_transform, output_pose.pose);

    // Publish the transformed pose on the output port
    setOutput("output_pose", output_pose);

    return NodeStatus::SUCCESS;
  }

private:
  geometry_msgs::PoseStamped input_pose_;
  geometry_msgs::Vector3 translation_xyz_;
  geometry_msgs::Quaternion quaternion_xyzw_;
};
