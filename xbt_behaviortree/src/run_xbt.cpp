#include <ros/ros.h>
#include <iostream>
#include <filesystem>
#include <mutex>
#include <behaviortree_cpp/bt_factory.h>
#include <xbt_behaviortree/action_nodes/sleep_action.h>
#include <xbt_behaviortree/action_nodes/detect_frame_action.h>
#include <xbt_behaviortree/action_nodes/transform_pose_action.h>
#include <xbt_behaviortree/action_nodes/gripper_action.h>
#include <xbt_behaviortree/action_nodes/switch_controller_action.h>
#include <xbt_behaviortree/action_nodes/simple_plan_action.h>
#include <xbt_behaviortree/action_nodes/joint_trajectory_action.h>
#include <xbt_behaviortree/action_nodes/compliant_trajectory_action.h>
#include <xbt_behaviortree/condition_nodes/grasped_condition.h>
#include <xbt_behaviortree/node_input_conversions.h>
#include <explain_bt/ExplainableBT.h>
#include <explain_bt/Explain.h>
#include <std_srvs/Empty.h>

using namespace BT;

class BehaviorTreeController
{
public:
  BehaviorTreeController(ros::NodeHandle &nh) : nh_(nh), running_flag_(false), start_flag_(false)
  {
    // Create BT factory and register actions
    BehaviorTreeFactory factory;
    RegisterRosAction<GripperAction>(factory, "GripperAction", nh);
    RegisterRosAction<JointTrajectoryAction>(factory, "JointTrajectoryAction", nh);
    RegisterRosAction<CompliantTrajectoryAction>(factory, "CompliantTrajectoryAction", nh);
    RegisterRosService<SwitchControllerAction>(factory, "SwitchControllerAction", nh);
    RegisterRosService<SimplePlanAction>(factory, "SimplePlanAction", nh);
    RegisterRosSubscriber<GraspedCondition>(factory, "GraspedCondition", nh);
    factory.registerNodeType<SleepAction>("SleepAction");
    factory.registerNodeType<TransformPoseAction>("TransformPoseAction");
    factory.registerNodeType<DetectFrameAction>("DetectFrameAction");

    // Get the bt_directory parameter
    std::string bt_directory;
    if (!nh.getParam("bt_directory", bt_directory)) {
        ROS_ERROR("Failed to retrieve bt_directory parameter.");
        return;
    }
    ROS_INFO("Loading BT XML from directory: %s", bt_directory.c_str());

    // Create tree from XML files
    using std::filesystem::directory_iterator;
    for (auto const &entry : directory_iterator(bt_directory))
    {
      if (entry.path().extension() == ".xml")
      {
        factory.registerBehaviorTreeFromFile(entry.path().string());
      }
    }

    bt_ = factory.createTree("MainTree");
    xbt_ = std::make_shared<XBT::ExplainableBT>(bt_);

    // Create ROS services for starting, stopping, and resetting the behavior tree
    start_tree_service_ = nh.advertiseService("start_tree", &BehaviorTreeController::startTreeCallback, this);
    stop_tree_service_ = nh.advertiseService("stop_tree", &BehaviorTreeController::stopTreeCallback, this);
    reset_tree_service_ = nh.advertiseService("reset_tree", &BehaviorTreeController::resetTreeCallback, this);

    // Create ROS service for explaining the behavior tree
    explain_service_ = nh.advertiseService("explain_tree", &BehaviorTreeController::explainCallback, this);
  }

  void run()
  {
    while (ros::ok()) {
      state_mutex_.lock();
      bool running = running_flag_;
      bool start = start_flag_;
      state_mutex_.unlock();

      if (running)
      {
        xbt_mutex_.lock();
        auto status = xbt_->status();
        xbt_mutex_.unlock();

        if (status == NodeStatus::RUNNING || start)
        {
          xbt_mutex_.lock();
          xbt_->tick();
          xbt_mutex_.unlock();

          state_mutex_.lock();
          start_flag_ = false;
          state_mutex_.unlock();
        }
      }
      ros::Duration(0.01).sleep();
    }
  }

private:
  ros::NodeHandle &nh_;
  BT::Tree bt_;
  std::shared_ptr<XBT::ExplainableBT> xbt_;
  bool running_flag_;
  bool start_flag_;
  std::mutex state_mutex_;
  std::mutex xbt_mutex_;
  ros::ServiceServer start_tree_service_;
  ros::ServiceServer stop_tree_service_;
  ros::ServiceServer reset_tree_service_;
  ros::ServiceServer explain_service_;

  bool startTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    state_mutex_.lock();
    running_flag_ = true;
    start_flag_ = true;
    state_mutex_.unlock();
    return true;
  }

  bool stopTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    state_mutex_.lock();
    running_flag_ = false;
    state_mutex_.unlock();
    return true;
  }

  bool resetTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    xbt_mutex_.lock();
    xbt_->halt();
    xbt_mutex_.unlock();
    return true;
  }

  bool explainCallback(explain_bt::Explain::Request &req, explain_bt::Explain::Response &res)
  {
    xbt_mutex_.lock();
    bool success = xbt_->explain_callback(req, res);
    xbt_mutex_.unlock();
    return success;
  }
};

int main(int argc, char **argv)
{

  // setup ros
  ros::init(argc, argv, "run_explain_bt");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0); // Number of threads
  spinner.start();

  // Create an instance of the BehaviorTreeController class
  BehaviorTreeController controller(nh);

  // Run the behavior tree controller
  controller.run();

  spinner.stop();

  return 0;
}