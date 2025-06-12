#include "nav2_drone_bt_navigator/action/read_goal.h"

namespace NavigationNodes
{

BT::NodeStatus NavDroneReadGoalAction::tick()
{
  setOutput("pose", goal_ );
  return BT::NodeStatus::SUCCESS;

}

}  // namespace
