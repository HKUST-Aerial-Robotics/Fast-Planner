#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// #include <dyn_planner/obj_predictor.h>
// #include <dyn_planner/planning_visualization.h>
// #include <dyn_planner/edt_environment.h>
#include <plan_manage/planning_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward
{
backward::SignalHandling sh;
}

using namespace dyn_planner;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dyn_planner_node");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  PlanningFSM fsm;
  fsm.init(nh);

  ros::spin();

  return 0;
}
