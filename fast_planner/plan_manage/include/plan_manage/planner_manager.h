#ifndef _KGB_TRAJECTORY_GENERATOR_H_
#define _KGB_TRAJECTORY_GENERATOR_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/non_uniform_bspline.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>

#include <plan_env/edt_environment.h>
#include <poly_traj/polynomial_traj.h>

#include <ros/ros.h>

namespace fast_planner {

struct PlanParameters {
  /* planning algorithm parameters */
  double max_vel_, max_acc_, max_jerk_;  // physical limits
  double local_segment_length_;          // local replanning trajectory length
  double control_points_distance_;       // distance between adjacient B-spline
                                         // control points
  double clearance_threshold_;
  int dynamic_environment_;

  /* parameters to evaluate performance */

  /* processing time */
  double time_search_ = 0.0;
  double time_optimize_ = 0.0;
  double time_adjust_ = 0.0;
};

struct LocalTrajectoryInfo {
  /* info of generated traj */

  int traj_id_;
  double traj_duration_, t_start_, t_end_, time_start_;
  ros::Time time_traj_start_;
  Eigen::Vector3d pos_traj_start_;
  NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_, yaw_traj_, yawdot_traj_,
      yawdotdot_traj_;
};

class IntermediatePlanData {
public:
  IntermediatePlanData(/* args */) {
  }
  ~IntermediatePlanData() {
  }

  vector<Eigen::Vector3d> global_waypoints_;

  // initial trajectory segment
  NonUniformBspline initial_local_segment_;
  vector<Eigen::Vector3d> local_start_end_derivative_;

  // kinodynamic path
  vector<Eigen::Vector3d> kino_path_;
};

// Fast Planner Manager
// Key algorithms of mapping and planning are called

class FastPlannerManager {
public:
  FastPlannerManager();
  ~FastPlannerManager();

  /* main planning interface */
  bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel);

  bool planGlobalReferenceTraj(const Eigen::Vector3d& start_pos);

  bool topoAndVisibReplan();

  void planHeading(const Eigen::Vector3d& start_yaw);

  /* planning info and module initializer */
  void initializePlanningModules(ros::NodeHandle& nh);
  void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);

  /* planning and trajectory info query */
  LocalTrajectoryInfo* getLocalTrajectoryInfo();
  bool checkTrajCollision(double& distance);

  PlanParameters* getPlanParameters();
  IntermediatePlanData* getIntermediatePlanData();

  EDTEnvironment::Ptr getEDTEnvironment();

private:
  /* main planning algorithms & modules */
  SDFMap::Ptr sdf_map_;
  EDTEnvironment::Ptr edt_environment_;

  Astar::Ptr geometric_path_finder_;
  KinodynamicAstar kinodynamic_path_finder_;
  vector<BsplineOptimizer> bspline_optimizers_;

  PlanParameters pp_;
  LocalTrajectoryInfo traj_info_;
  IntermediatePlanData plan_data_;

  void updateTrajInfo();

  void reparameterizeBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts,
                             double& dt, double& time_inc);

  void calcNextYaw(const double& last_yaw, double& yaw);

public:
  typedef shared_ptr<FastPlannerManager> Ptr;
};
}  // namespace fast_planner

#endif