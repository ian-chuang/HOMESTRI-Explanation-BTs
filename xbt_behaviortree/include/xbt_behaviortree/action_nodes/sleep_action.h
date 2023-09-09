#include <ros/ros.h>
#include "behaviortree_cpp/action_node.h"

using namespace BT;
using namespace std::chrono;

// Example of Asynchronous node that uses StatefulActionNode as base class
class SleepAction : public BT::StatefulActionNode
{
  public:
    SleepAction(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
      // amount of milliseconds that we want to sleep
      return{ BT::InputPort<int>("msec") };
    }

    NodeStatus onStart() override
    {
      int msec = 0;
      getInput("msec", msec);

      if( msec <= 0 ) {
        // No need to go into the RUNNING state
        return NodeStatus::SUCCESS;
      }
      else {
        // once the deadline is reached, we will return SUCCESS.
        deadline_ = system_clock::now() + milliseconds(msec);
        return NodeStatus::RUNNING;
      }
    }

    /// method invoked by an action in the RUNNING state.
    NodeStatus onRunning() override
    {
      if ( system_clock::now() >= deadline_ ) {
        return NodeStatus::SUCCESS;
      }
      else {
        return NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      // nothing to do here...
    }

  private:
    system_clock::time_point deadline_;
    BT::NodeStatus return_status_;
};