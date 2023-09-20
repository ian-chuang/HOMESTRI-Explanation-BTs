#include <ros/ros.h>
#include "behaviortree_ros/bt_async_behavior.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace BT;

class SetTargetWrenchAction : public AsyncBehaviorBase
{
public:
  // Define a constant default timeout value.
  const static inline std::string EndEffectorFrame = "tcp_link";

  SetTargetWrenchAction(const std::string &name, const NodeConfiguration &conf)
      : AsyncBehaviorBase(name, conf)
  {
  }

  static PortsList providedPorts()
  {
    return{ 
      BT::InputPort<int>("msec"),
      InputPort<geometry_msgs::Wrench>("wrench"),
      InputPort<std::string>("wrench_frame_id"),
    };
  }

  BT::NodeStatus doWork() override
  {
    geometry_msgs::Wrench wrench;
    if (!getInput<geometry_msgs::Wrench>("wrench", wrench)) {
      ROS_ERROR("Missing required input [wrench]");
      return NodeStatus::FAILURE;
    }

    std::string wrench_frame_id;
    if (!getInput<std::string>("wrench_frame_id", wrench_frame_id)) {
      ROS_ERROR("Missing required input [wrench_frame_id]");
      return NodeStatus::FAILURE;
    }

    unsigned msec;
    if (!getInput<unsigned>("msec", msec))
    {
      ROS_ERROR("Missing required input [msec]");
      return NodeStatus::FAILURE;
    }
    ros::Duration wait_duration(static_cast<double>(msec) * 1e-3);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Publisher wrench_stamped_pub_ = ros::NodeHandle().advertise<geometry_msgs::WrenchStamped>("/target_wrench", 1);

    // start time 
    ros::Time start_time = ros::Time::now();
    // keep publishing wrench until past wait_duration
    while( ros::Time::now() - start_time < wait_duration ) {
      // Create the input wrench stamped message
      geometry_msgs::WrenchStamped input_wrench_stamped;
      input_wrench_stamped.header.frame_id = wrench_frame_id;
      input_wrench_stamped.header.stamp = ros::Time::now();
      input_wrench_stamped.wrench = wrench;

      // Create the transformed wrench stamped message
      geometry_msgs::WrenchStamped transformed_wrench_stamped;

      try {
        // Use tfBufferPtr_ to perform the transformation
        transformed_wrench_stamped = tfBuffer.transform(input_wrench_stamped, EndEffectorFrame, ros::Duration(1.0));
      } catch (tf2::TransformException& ex) {
        continue;
      }

      // Publish the transformed wrench stamped message
      wrench_stamped_pub_.publish(transformed_wrench_stamped);
    }

    return NodeStatus::SUCCESS;

  }
};