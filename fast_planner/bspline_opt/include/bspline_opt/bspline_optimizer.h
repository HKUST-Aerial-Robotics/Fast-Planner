#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <plan_env/edt_environment.h>
#include <ros/ros.h>

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace fast_planner {
class BsplineOptimizer {
private:
  EDTEnvironment::Ptr edt_environment_;

  // main input
  Eigen::MatrixXd control_points_;     // B-spline control points, nx3
  double bspline_interval_;            // B-spline knot span
  int order_;                          // bspline degree
  Eigen::Vector3d end_pt_;             // end of the trajectory
  vector<Eigen::Vector3d> guide_pts_;  // geometric guiding path
  int optimization_phase_;             // use different objective function
  bool dynamic_;                       // moving obstacles ?
  double time_traj_start_;             // global time for moving obstacles

  /* optimization parameters */
  double lambda1_;            // curvature weight
  double lambda2_;            // distance weight
  double lambda3_;            // feasibility weight
  double lambda4_;            // end point weight
  double lambda5_;            // guide cost weight
  double lambda6_;            // visibility cost weight
  double dist0_;              // safe distance
  double max_vel_, max_acc_;  // dynamic limits
  double visib_min_;          // threshold of visibility
  int algorithm1_;            // optimization algorithms used in different phase
  int algorithm2_;            //
  int max_iteration_num_[3];  // optimization stopping criteria
  int visible_num_;           //

  /* intermediate variables */
  /* buffer for gradient of cost function, to avoid repeated allocation and
   * release of memory */
  vector<Eigen::Vector3d> g_q_;
  vector<Eigen::Vector3d> g_smoothness_;
  vector<Eigen::Vector3d> g_distance_;
  vector<Eigen::Vector3d> g_feasibility_;
  vector<Eigen::Vector3d> g_endpoint_;
  vector<Eigen::Vector3d> g_guide_;
  vector<Eigen::Vector3d> g_visib_;
  int variable_num_;                   // optimization variables
  int iter_num_;                       // iteration of the solver
  std::vector<double> best_variable_;  //
  double min_cost_;                    //
  int start_id_, end_id_;              //
  int end_constrain_;                  // hard / soft constraints of end point
  vector<Eigen::Vector3d> block_pts_;  // blocking points to compute visibility

public:
  BsplineOptimizer() {
  }
  ~BsplineOptimizer() {
  }

  /* final position constraint. In hard constraint, the last p control points
   * will not be optimized and the end position cost funtion is disabled. In
   * soft one, the last p control points are also added to the optimization
   * variables and the endpoint cost function is computed*/
  enum END_CONSTRAINT { HARD_CONSTRAINT = 1, SOFT_CONSTRAINT = 2 };

  /* different phases of the optimization. In phase one, only the smoothness and
  guiding path cost are computed. In phase two, smoothness, collision and
  dynamic feasibility are computed. In the future we will include phase three,
  in which a visibility cost is added on the basis of phase two */
  enum OPTIMIZATION_PHASE { FIRST_PHASE = 1, SECOND_PHASE = 2, VISIB_PHASE = 3 };

  /* main API */
  void setEnvironment(const EDTEnvironment::Ptr& env);
  void setParam(ros::NodeHandle& nh);
  Eigen::MatrixXd BsplineOptimizeTraj(Eigen::MatrixXd points, double ts, int obj, double time_limit,
                                      const vector<Eigen::Vector3d>& guide_pt);
  /* helper function */
  void setControlPoints(Eigen::MatrixXd points);
  void setBSplineInterval(double ts);
  void setGuidePath(const vector<Eigen::Vector3d>& guide_pt);
  void setOptimizationPhase(int obj);
  void setOptimizationRange(int start, int end);
  void optimize(int end_cons, bool dynamic, double time_limit, double time_start = -1.0);
  Eigen::MatrixXd getControlPoints();
  vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);

private:
  /* cost function */
  /* calculate each part of cost function with control points q as input */
  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  void combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);
  void calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient);
  void calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient);
  void calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                           vector<Eigen::Vector3d>& gradient);
  void calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient);

  /* for benckmark evaluation only */
public:
  vector<double> vec_cost_;
  vector<double> vec_time_;
  ros::Time time_start_;

  void getCostCurve(vector<double>& cost, vector<double>& time) {
    cost = vec_cost_;
    time = vec_time_;
  }

  typedef shared_ptr<BsplineOptimizer> Ptr;
};
}  // namespace fast_planner
#endif