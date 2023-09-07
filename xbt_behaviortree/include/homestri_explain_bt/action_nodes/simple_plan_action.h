#include "behaviortree_cpp/behavior_tree.h"
#include <behaviortree_ros/bt_service_node.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <homestri_planning_interface/SimplePlan.h>
#include <homestri_planning_interface/SimplePlanRequest.h>
#include <homestri_planning_interface/SimplePlanResponse.h>

using namespace BT;

class SimplePlanAction : public RosServiceNode<homestri_planning_interface::SimplePlan>
{
private:
    const static inline double DefaultVelScaling = 0.1;
    const static inline double DefaultAccScaling = 0.1;

public:
    SimplePlanAction(ros::NodeHandle &handle, const std::string &node_name, const NodeConfiguration &conf)
        : RosServiceNode<homestri_planning_interface::SimplePlan>(handle, node_name, conf)
    {
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("mode"),
            InputPort<std::string>("target"),
            InputPort<geometry_msgs::PoseStamped>("pose"),
            InputPort<double>("vel_scaling"),
            InputPort<double>("acc_scaling"),
            OutputPort<trajectory_msgs::JointTrajectory>("joint_trajectory"),
        };
    }

    void sendRequest(RequestType &request) override
    {
        std::string mode_str;
        if (!getInput<std::string>("mode", mode_str))
        {
            ROS_ERROR("Missing required input [mode]");
            return;
        }
        uint8_t mode;
        if (mode_str == "POSE")
        {
            mode = homestri_planning_interface::SimplePlanRequest::POSE;
            if (!getInput<geometry_msgs::PoseStamped>("pose", request.pose))
            {
                ROS_ERROR("Missing required input [pose]");
                return;
            }
        }
        else if (mode_str == "POSE_LINE")
        {
            mode = homestri_planning_interface::SimplePlanRequest::POSE_LINE;
            if (!getInput<geometry_msgs::PoseStamped>("pose", request.pose))
            {
                ROS_ERROR("Missing required input [pose]");
                return;
            }
        }
        else if (mode_str == "TARGET")
        {
            mode = homestri_planning_interface::SimplePlanRequest::TARGET;
            if (!getInput<std::string>("target", request.target))
            {
                ROS_ERROR("Missing required input [target]");
                return;
            }
        }
        else
        {
            ROS_ERROR("Invalid mode input: %s", mode_str.c_str());
            return;
        }

        if (!getInput<double>("vel_scaling", request.vel_scaling))
        {
            request.vel_scaling = SimplePlanAction::DefaultVelScaling;
        }

        if (!getInput<double>("acc_scaling", request.acc_scaling))
        {
            request.acc_scaling = SimplePlanAction::DefaultAccScaling;
        }
    }

    NodeStatus onResponse(const ResponseType &rep) override
    {
        if (rep.success)
        {
            setOutput("joint_trajectory", rep.joint_trajectory);
            return NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("SimplePlanAction: plan failed");
            return NodeStatus::FAILURE;
        }
    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
        ROS_ERROR("SimplePlanAction: request failed %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }
};
