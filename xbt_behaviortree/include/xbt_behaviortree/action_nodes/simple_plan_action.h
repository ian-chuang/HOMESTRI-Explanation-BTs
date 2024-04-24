#include <ros/ros.h>
#include <behaviortree_ros/bt_service_node.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <xbt_planning_interface/SimplePlan.h>
#include <xbt_planning_interface/SimplePlanRequest.h>
#include <xbt_planning_interface/SimplePlanResponse.h>

using namespace BT;

class SimplePlanAction : public RosServiceNode<xbt_planning_interface::SimplePlan>
{
private:
    const static inline double DefaultVelScaling = 0.1;
    const static inline double DefaultAccScaling = 0.1;

public:
    SimplePlanAction(ros::NodeHandle &handle, const std::string &node_name, const NodeConfiguration &conf)
        : RosServiceNode<xbt_planning_interface::SimplePlan>(handle, node_name, conf)
    {
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("mode"),
            InputPort<std::string>("target"),
            InputPort<std::vector<double>>("joints"),
            InputPort<geometry_msgs::Pose>("pose"),
            InputPort<std::string>("pose_frame_id"),
            InputPort<geometry_msgs::Point>("center"),
            InputPort<std::string>("center_frame_id"),
            InputPort<double>("vel_scaling", SimplePlanAction::DefaultVelScaling, "Velocity scaling factor"),
            InputPort<double>("acc_scaling", SimplePlanAction::DefaultAccScaling, "Acceleration scaling factor"),
            OutputPort<trajectory_msgs::JointTrajectory>("joint_trajectory"),
        };
    }

    bool sendRequest(RequestType &request) override
    {
        std::string mode_str;
        if (!getInput<std::string>("mode", mode_str))
        {
            ROS_ERROR("Missing required input [mode]");
            return false;
        }
        if (mode_str == "POSE")
        {
            request.mode = xbt_planning_interface::SimplePlanRequest::POSE;
            if (!getInput<geometry_msgs::Pose>("pose", request.pose.pose))
            {
                ROS_ERROR("Missing required input [pose]");
                return false; 
            }
            if (!getInput<std::string>("pose_frame_id", request.pose.header.frame_id))
            {
                ROS_ERROR("Missing required input [pose_frame_id]");
                return false; 
            }
        }
        else if (mode_str == "POSE_LINE")
        {
            request.mode = xbt_planning_interface::SimplePlanRequest::POSE_LINE;
            if (!getInput<geometry_msgs::Pose>("pose", request.pose.pose))
            {
                ROS_ERROR("Missing required input [pose]");
                return false;
            }
            if (!getInput<std::string>("pose_frame_id", request.pose.header.frame_id))
            {
                ROS_ERROR("Missing required input [pose_frame_id]");
                return false; 
            }
        }
        else if (mode_str == "TARGET")
        {
            request.mode = xbt_planning_interface::SimplePlanRequest::TARGET;
            if (!getInput<std::string>("target", request.target))
            {
                ROS_ERROR("Missing required input [target]");
                return false;
            }
        }
        else if (mode_str == "JOINT")
        {
            request.mode = xbt_planning_interface::SimplePlanRequest::JOINT;
            if (!getInput<std::vector<double>>("joints", request.joints))
            {
                ROS_ERROR("Missing required input [joints]");
                return false;
            }
        }
        else if (mode_str == "CIRCLE")
        {
            request.mode = xbt_planning_interface::SimplePlanRequest::CIRCLE;
            if (!getInput<geometry_msgs::Pose>("pose", request.pose.pose))
            {
                ROS_ERROR("Missing required input [pose]");
                return false;
            }
            if (!getInput<std::string>("pose_frame_id", request.pose.header.frame_id))
            {
                ROS_ERROR("Missing required input [pose_frame_id]");
                return false; 
            }
            if (!getInput<geometry_msgs::Point>("center", request.center.point))
            {
                ROS_ERROR("Missing required input [center]");
                return false;
            }
            if (!getInput<std::string>("center_frame_id", request.center.header.frame_id))
            {
                ROS_ERROR("Missing required input [center_frame_id]");
                return false;
            }
        }
        else
        {
            ROS_ERROR("Invalid mode input: %s", mode_str.c_str());
            return false;
        }

        if (!getInput<double>("vel_scaling", request.vel_scaling))
        {
            ROS_ERROR("Missing required input [vel_scaling]");
            return false;
        }

        if (!getInput<double>("acc_scaling", request.acc_scaling))
        {
            ROS_ERROR("Missing required input [acc_scaling]");
            return false;
        }

        return true;
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
            return NodeStatus::FAILURE;
        }
    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
        ROS_ERROR("SimplePlanAction service failure: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }
};
