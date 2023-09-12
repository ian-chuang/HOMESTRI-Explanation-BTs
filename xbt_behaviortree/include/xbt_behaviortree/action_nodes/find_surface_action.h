#include <ros/ros.h>
#include <behaviortree_ros/bt_action_node.h>
#include <xbt_planning_interface/FindSurfaceAction.h>
#include <xbt_planning_interface/FindSurfaceGoal.h>
#include <xbt_planning_interface/FindSurfaceResult.h>
#include <xbt_planning_interface/FindSurfaceFeedback.h>

using namespace BT;

class FindSurfaceAction : public RosActionNode<xbt_planning_interface::FindSurfaceAction>
{
private:
    inline const static std::string WrenchFrameID = "tcp_link";

public:
    FindSurfaceAction(ros::NodeHandle &handle, const std::string &name, const NodeConfiguration &conf)
        : RosActionNode<xbt_planning_interface::FindSurfaceAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("force_target"),
            InputPort<double>("force_threshold"),
            InputPort<double>("maximum_distance"),
            InputPort<double>("maximum_duration")};
    }

    bool sendGoal(GoalType &goal) override
    {
        double force_target;
        if (!getInput("force_target", force_target))
        {
            ROS_ERROR("Missing required input [force_target]");
            return false;
        }

        // set goal wrench values
        goal.wrench.header.frame_id = FindSurfaceAction::WrenchFrameID;
        goal.wrench.wrench.force.x = force_target;
        goal.wrench.wrench.force.y = 0.0;
        goal.wrench.wrench.force.z = 0.0;
        goal.wrench.wrench.torque.x = 0.0;
        goal.wrench.wrench.torque.y = 0.0;
        goal.wrench.wrench.torque.z = 0.0;

        if (!getInput("force_threshold", goal.force_threshold))
        {
            ROS_ERROR("Missing required input [force_threshold]");
            return false;
        }

        if (!getInput("maximum_distance", goal.maximum_distance))
        {
            ROS_ERROR("Missing required input [maximum_distance]");
            return false;
        }

        double timeout;
        if (!getInput("maximum_duration", timeout))
        {
            ROS_ERROR("Missing required input [maximum_duration]");
            return false;
        }
        goal.timeout = ros::Duration(timeout);
        
        return true;
    }

    NodeStatus onResult(const ResultType &res) override
    {
        if (res.error_code == xbt_planning_interface::FindSurfaceResult::FORCE_THRESHOLD_EXCEEDED)
        {
            return NodeStatus::SUCCESS;
        }
        else
        {
            return NodeStatus::FAILURE;
        }
    }

    virtual NodeStatus onFailedRequest(FailureCause failure) override
    {
        ROS_ERROR("FindSurfaceAction action server failure: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }
};