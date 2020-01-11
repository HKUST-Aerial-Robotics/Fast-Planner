#ifndef _NON_UNIFORM_BSPLINE_H_
#define _NON_UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace fast_planner {
class NonUniformBspline {
private:
  /* non-uniform bspline */
  int p_, n_, m_;  // p degree, n+1 control points, m = n+p+1
  Eigen::MatrixXd control_points_;
  Eigen::VectorXd u_;  // knots vector
  double interval_;    // knot span \delta t

  Eigen::Vector3d x0_, v0_, a0_;

  Eigen::MatrixXd getDerivativeControlPoints();

  double limit_vel_, limit_acc_, limit_ratio_;  // physical limits and time adjustment ratio

public:
  NonUniformBspline() {
  }
  NonUniformBspline(Eigen::MatrixXd points, int order, double interval, bool zero = true);
  ~NonUniformBspline();

  // get / set basic bspline info

  void setKnot(Eigen::VectorXd knot);
  Eigen::VectorXd getKnot();
  Eigen::MatrixXd getControlPoint();
  double getInterval();
  void getTimeSpan(double& um, double& um_p);

  // compute position / derivative

  Eigen::Vector3d evaluateDeBoor(double t);
  NonUniformBspline getDerivative();

  // B-spline interpolation of points in point_set, with boundary vel&acc
  // constraints
  static void parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                    const vector<Eigen::Vector3d>& start_end_derivative,
                                    Eigen::MatrixXd& ctrl_pts);

  /* check feasibility, adjust time */

  void setPhysicalLimits(double vel, double acc);
  bool checkFeasibility(bool show = false);
  double checkRatio();
  void lengthenTime(double ratio);

  /* for performance evaluation */

  double getTimeSum();
  double getLength(double res = 0.01);
  double getJerk();

  void getMeanAndMaxVel(double& mean_v, double& max_v);
  void getMeanAndMaxAcc(double& mean_a, double& max_a);

  // maybe deprecated ?

  bool reallocateTime(bool show = false);
  bool adjustTime(bool show = false);
  pair<Eigen::Vector3d, Eigen::Vector3d> getHeadTailPts();
  void recomputeInit();

  // typedef std::shared_ptr<NonUniformBspline> Ptr;
};
}  // namespace fast_planner
#endif