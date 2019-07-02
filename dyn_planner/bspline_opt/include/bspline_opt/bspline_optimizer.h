#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include "plan_env/edt_environment.h"

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace dyn_planner
{
class BsplineOptimizer
{
private:
  EDTEnvironment::Ptr edt_env_;
  Eigen::MatrixXd control_points_;  // nx3
  Eigen::Vector3d end_pt_;
  vector<tuple<int, int, Eigen::Vector3d>> ranges_;
  bool use_guide_;

  /* optimization parameters */
  double lamda1_;             // curvature weight
  double lamda2_;             // distance weight
  double lamda3_;             // feasibility weight
  double lamda4_;             // end point weight
  double lamda5_;             // guide cost weight
  double dist0_;              // safe distance
  double dist1_;              // unsafe distance
  double max_vel_, max_acc_;  // constrains parameters
  int variable_num_;
  int algorithm_;
  int max_iteration_num_, iter_num_;
  std::vector<double> best_variable_;
  double min_cost_;
  int start_id_, end_id_;

  /* bspline */
  double bspline_interval_;  // ts
  int order_;                // bspline order

  int end_constrain_;
  bool dynamic_;
  double time_traj_start_;

  int collision_type_;

public:
  BsplineOptimizer()
  {
  }
  ~BsplineOptimizer()
  {
  }

  enum END_CONSTRAINT
  {
    HARD_CONSTRAINT = 1,
    SOFT_CONSTRAINT = 2
  };

  /* main API */
  void setControlPoints(Eigen::MatrixXd points);
  void setBSplineInterval(double ts);
  void setEnvironment(const EDTEnvironment::Ptr& env);

  void setParam(ros::NodeHandle& nh);
  void setOptimizationRange(int start, int end);

  void optimize(int end_cons, bool dynamic, double time_start = -1.0);
  Eigen::MatrixXd getControlPoints();

private:
  /* NLopt cost */
  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  /* helper function */
  void getDistanceAndGradient(Eigen::Vector3d& pos, double& dist, Eigen::Vector3d& grad);

  /* calculate each part of cost function with control points q */
  void combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  void calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  void calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  void calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  void calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);

public:
  /* for evaluation */
  vector<double> vec_cost_;
  vector<double> vec_time_;
  ros::Time time_start_;

  void getCostCurve(vector<double>& cost, vector<double>& time)
  {
    cost = vec_cost_;
    time = vec_time_;
  }

  typedef shared_ptr<BsplineOptimizer> Ptr;
};
}  // namespace dyn_planner
#endif