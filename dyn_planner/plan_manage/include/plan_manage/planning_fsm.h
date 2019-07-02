#ifndef _PLANNING_FSM_H_
#define _PLANNING_FSM_H_

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>

#include <traj_utils/planning_visualization.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <path_searching/kinodynamic_astar.h>
#include <bspline_opt/bspline_optimizer.h>
#include <plan_manage/dyn_planner_manager.h>
#include "plan_manage/Bspline.h"

using std::vector;

namespace dyn_planner
{
class PlanningFSM
{
private:
  /* ---------- flag ---------- */
  bool trigger_, have_goal_;
  enum EXEC_STATE
  {
    INIT,
    WAIT_GOAL,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ
  };
  EXEC_STATE exec_state_;

  enum FLIGHT_TYPE
  {
    MANUAL_GOAL = 1,
    PRESET_GOAL = 2
  };

  void changeExecState(EXEC_STATE new_state, string pos_call);
  void printExecState();

  /* ---------- planning utils ---------- */
  SDFMap::Ptr sdf_map_;

  EDTEnvironment::Ptr edt_env_;

  Astar::Ptr path_finder0_;
  KinodynamicAstar::Ptr path_finder_;
  BsplineOptimizer::Ptr bspline_optimizer_;

  DynPlannerManager::Ptr planner_manager_;

  PlanningVisualization::Ptr visualization_;

  /* ---------- parameter ---------- */
  int flight_type_;  // 1 mannual select, 2 hard code
  double thresh_no_replan_, thresh_replan_;
  double waypoints_[10][3];
  int wp_num_;

  /* ---------- planning api ---------- */
  Eigen::Vector3d start_pt_, start_vel_, start_acc_, end_pt_, end_vel_;
  int current_wp_;

  bool planSearchOpt();  // front-end and back-end method

  /* ---------- sub and pub ---------- */
  ros::NodeHandle node_;

  ros::Timer exec_timer_, safety_timer_;
  ros::Timer vis_timer_, query_timer_;

  ros::Subscriber waypoint_sub_;

  ros::Publisher replan_pub_, bspline_pub_;

  void execFSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);

  void waypointCallback(const nav_msgs::PathConstPtr& msg);

public:
  PlanningFSM(/* args */)
  {
  }
  ~PlanningFSM()
  {
  }

  void init(ros::NodeHandle& nh);
};

}  // namespace dyn_planner

#endif