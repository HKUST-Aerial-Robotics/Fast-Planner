#ifndef _NON_UNIFORM_BSPLINE_H_
#define _NON_UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>

using namespace std;

namespace dyn_planner
{
class NonUniformBspline
{
private:
  /* non-uniform bspline */
  int p_, n_, m_;
  Eigen::MatrixXd control_points_;
  Eigen::VectorXd u_;  // knots vector
  double interval_;    // init interval

  Eigen::Vector3d x0_, v0_, a0_;

  Eigen::MatrixXd getDerivativeControlPoints();

public:
  static double limit_vel_, limit_acc_, limit_ratio_;

  NonUniformBspline()
  {
  }
  NonUniformBspline(Eigen::MatrixXd points, int order, double interval, bool zero = true);
  ~NonUniformBspline();
  void setKnot(Eigen::VectorXd knot);
  Eigen::VectorXd getKnot();

  Eigen::MatrixXd getControlPoint()
  {
    return control_points_;
  }

  void getTimeSpan(double& um, double& um_p);

  Eigen::Vector3d evaluateDeBoor(double t);

  NonUniformBspline getDerivative();

  static void getControlPointEqu3(Eigen::MatrixXd samples, double ts, Eigen::MatrixXd& control_pts);
  static void BsplineParameterize(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                  const vector<Eigen::Vector3d>& start_end_derivative, Eigen::MatrixXd& ctrl_pts);

  /* check feasibility, reallocate time and recompute first 3 ctrl pts */
  bool checkFeasibility(bool show = false);
  bool reallocateTime(bool show = false);
  bool adjustTime(bool show = false);
  void recomputeInit();

  /* for evaluation */
  double getTimeSum();
  double getLength();
  double getJerk();

  void getMeanAndMaxVel(double& mean_v, double& max_v);
  void getMeanAndMaxAcc(double& mean_a, double& max_a);

  // typedef std::shared_ptr<NonUniformBspline> Ptr;
};
}  // namespace dyn_planner
#endif