#include <ros/ros.h>
#include <behaviortree_cpp/bt_factory.h>
#include <xbt_behaviortree/action_nodes/sleep_action.h>
#include <xbt_behaviortree/action_nodes/detect_frame_action.h>
#include <xbt_behaviortree/action_nodes/transform_pose_action.h>
#include <xbt_behaviortree/action_nodes/set_target_wrench_action.h>
#include <xbt_behaviortree/action_nodes/gripper_action.h>
#include <xbt_behaviortree/action_nodes/switch_controller_action.h>
#include <xbt_behaviortree/action_nodes/simple_plan_action.h>
#include <xbt_behaviortree/action_nodes/trigger_action.h>
#include <xbt_behaviortree/action_nodes/joint_trajectory_action.h>
#include <xbt_behaviortree/action_nodes/compliant_trajectory_action.h>
#include <xbt_behaviortree/action_nodes/find_surface_action.h>
#include <xbt_behaviortree/condition_nodes/grasped_condition.h>
#include <xbt_behaviortree/node_input_conversions.h>
#include <explain_bt/ExplainableBTController.h>

int main(int argc, char **argv)
{
  // Initialize ROS.
  ros::init(argc, argv, "run_xbt");
  ros::NodeHandle nh;

  // Create a Behavior Tree Factory.
  BehaviorTreeFactory factory;
  RegisterRosAction<GripperAction>(factory, "GripperAction", nh);
  RegisterRosAction<JointTrajectoryAction>(factory, "JointTrajectoryAction", nh);
  RegisterRosAction<CompliantTrajectoryAction>(factory, "CompliantTrajectoryAction", nh);
  RegisterRosAction<FindSurfaceAction>(factory, "FindSurfaceAction", nh);
  RegisterRosService<SwitchControllerAction>(factory, "SwitchControllerAction", nh);
  RegisterRosService<SimplePlanAction>(factory, "SimplePlanAction", nh);
  RegisterRosService<TriggerAction>(factory, "TriggerAction", nh);
  RegisterRosSubscriber<GraspedCondition>(factory, "GraspedCondition", nh);
  factory.registerNodeType<SetTargetWrenchAction>("SetTargetWrenchAction");
  factory.registerNodeType<SleepAction>("SleepAction");
  factory.registerNodeType<TransformPoseAction>("TransformPoseAction");
  factory.registerNodeType<DetectFrameAction>("DetectFrameAction");

  enum Status {OPEN=1, CLOSED=0};
  factory.registerScriptingEnums<Status>();


  // Get the bt_directory parameter
  std::string bt_directory;
  if (!nh.getParam("bt_directory", bt_directory))
  {
    ROS_ERROR("Failed to retrieve bt_directory parameter.");
    return 0;
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

  auto tree = factory.createTree("MainTree");

  // Create an Explainable Behavior Tree Controller.
  XBT::ExplainableBTController controller(tree, nh);
  controller.run();

  return 0;
}