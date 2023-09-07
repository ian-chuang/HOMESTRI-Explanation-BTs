#include <ros/ros.h>
#include <iostream>
#include <filesystem>

#include <behaviortree_cpp/bt_factory.h>
#include <xbt_behaviortree/action_nodes/sleep_action.h>
#include <xbt_behaviortree/action_nodes/detect_frame_action.h>
#include <xbt_behaviortree/action_nodes/transform_pose_action.h>
#include <xbt_behaviortree/action_nodes/gripper_action.h>
#include <xbt_behaviortree/action_nodes/switch_controller_action.h>
#include <xbt_behaviortree/action_nodes/simple_plan_action.h>
#include <xbt_behaviortree/action_nodes/joint_trajectory_action.h>
#include <xbt_behaviortree/node_input_conversions.h>
#include <explain_bt/ExplainableBT.h>

using namespace BT;

std::string get_xml_filename(ros::NodeHandle& nh, std::string param) {
  std::string xml_filename;
  nh.getParam(param, xml_filename);
  std::cout << xml_filename << "********" << std::endl;
  ROS_INFO("Loading XML : %s", xml_filename.c_str());
  return xml_filename;
}

int main(int argc, char **argv) {

  // setup ros
  ros::init(argc, argv, "run_explain_bt");
  ros::NodeHandle nh;

  // create bt factory
  BehaviorTreeFactory factory;
  // RegisterRosAction<ManipulationAction>(factory, "ManipulationAction", nh);
  // RegisterRosAction<ComplianceControlAction>(factory, "ComplianceControlAction", nh);
  // RegisterRosAction<CartesianControlAction>(factory, "CartesianControlAction", nh);
  // RegisterRosAction<ForceControlAction>(factory, "ForceControlAction", nh);
  RegisterRosAction<GripperAction>(factory, "GripperAction", nh);
  RegisterRosAction<JointTrajectoryAction>(factory, "JointTrajectoryAction", nh);
  RegisterRosService<SwitchControllerAction>(factory, "SwitchControllerAction", nh);
  RegisterRosService<SimplePlanAction>(factory, "SimplePlanAction", nh);

  factory.registerNodeType<SleepAction>("SleepAction");
  factory.registerNodeType<TransformPoseAction>("TransformPoseAction");
  factory.registerNodeType<DetectFrameAction>("DetectFrameAction");
  // factory.registerNodeType<DetectFrame>("DetectFrame");
  // factory.registerNodeType<UpdatePose>("UpdatePose");

  // create tree from xml files
  std::string bt_directory;
  nh.getParam("bt_directory", bt_directory);
  using std::filesystem::directory_iterator;
  for (auto const& entry : directory_iterator(bt_directory)) 
  {
    if( entry.path().extension() == ".xml")
    {
      factory.registerBehaviorTreeFromFile(entry.path().string());
    }
  }

  auto tree = factory.createTree("MainTree");
  XBT::ExplainableBT explainable_tree(tree);
  
  ros::ServiceServer service = nh.advertiseService("explainable_bt", &XBT::ExplainableBT::explain_callback, &explainable_tree);

  ros::AsyncSpinner spinner(0); // Number of threads
  spinner.start();

  NodeStatus status = explainable_tree.getStatus();
  while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING /*|| status == NodeStatus::FAILURE*/))
  {
    status = explainable_tree.execute();
    ros::Duration(0.01).sleep();  
  }

  spinner.stop();

  return 0;
}