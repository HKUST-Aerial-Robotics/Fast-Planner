#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
using namespace std;

namespace dyn_planner
{
void BsplineOptimizer::setControlPoints(Eigen::MatrixXd points)
{
  this->control_points_ = points;
  this->start_id_ = order_;
  this->end_id_ = this->control_points_.rows() - order_;
  use_guide_ = false;
}

void BsplineOptimizer::setOptimizationRange(int start, int end)
{
  this->start_id_ = min(max(start, order_), int(control_points_.rows() - order_));
  this->end_id_ = min(max(end, order_), int(control_points_.rows() - order_));
  cout << "opt range:" << this->start_id_ << ", " << this->end_id_ << endl;
}

void BsplineOptimizer::setParam(ros::NodeHandle& nh)
{
  nh.param("optimization/lamda1", lamda1_, -1.0);
  nh.param("optimization/lamda2", lamda2_, -1.0);
  nh.param("optimization/lamda3", lamda3_, -1.0);
  nh.param("optimization/lamda4", lamda4_, -1.0);
  nh.param("optimization/lamda5", lamda5_, -1.0);
  nh.param("optimization/dist0", dist0_, -1.0);
  nh.param("optimization/dist1", dist1_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/max_iteration_num", max_iteration_num_, -1);
  nh.param("optimization/algorithm", algorithm_, -1);
  nh.param("optimization/order", order_, -1);

  std::cout << "lamda1: " << lamda1_ << std::endl;
  std::cout << "lamda2: " << lamda2_ << std::endl;
  std::cout << "lamda3: " << lamda3_ << std::endl;
  std::cout << "lamda4: " << lamda4_ << std::endl;
}

void BsplineOptimizer::setBSplineInterval(double ts)
{
  this->bspline_interval_ = ts;
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_env_ = env;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints()
{
  return this->control_points_;
}

/* best algorithm_ is 40: SLSQP(constrained), 11 LBFGS(unconstrained barrier
method */
void BsplineOptimizer::optimize(int end_cons, bool dynamic, double time_start)
{
  /* ---------- initialize solver ---------- */
  end_constrain_ = end_cons;
  dynamic_ = dynamic;
  time_traj_start_ = time_start;
  iter_num_ = 0;

  if (end_constrain_ == HARD_CONSTRAINT)
  {
    variable_num_ = 3 * (end_id_ - start_id_);
    // std::cout << "hard: " << end_constrain_ << std::endl;
  }
  else if (end_constrain_ == SOFT_CONSTRAINT)
  {
    variable_num_ = 3 * (control_points_.rows() - start_id_);
    // std::cout << "soft: " << end_constrain_ << std::endl;
  }

  min_cost_ = std::numeric_limits<double>::max();

  nlopt::opt opt(nlopt::algorithm(algorithm_), variable_num_);

  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_);
  // opt.set_xtol_rel(1e-4);
  // opt.set_maxtime(1e-2);

  /* ---------- init variables ---------- */
  vector<double> q(variable_num_);
  double final_cost;
  for (int i = 0; i < int(control_points_.rows()); ++i)
  {
    if (i < start_id_)
      continue;

    if (end_constrain_ == HARD_CONSTRAINT && i >= end_id_)
    {
      continue;
      // std::cout << "jump" << std::endl;
    }

    for (int j = 0; j < 3; j++)
      q[3 * (i - start_id_) + j] = control_points_(i, j);
  }

  if (end_constrain_ == SOFT_CONSTRAINT)
  {
    end_pt_ = (1 / 6.0) *
              (control_points_.row(control_points_.rows() - 3) + 4 * control_points_.row(control_points_.rows() - 2) +
               control_points_.row(control_points_.rows() - 1));
    // std::cout << "end pt" << std::endl;
  }

  try
  {
    /* ---------- optimization ---------- */
    cout << "[Optimization]: begin-------------" << endl;
    cout << fixed << setprecision(7);
    vec_time_.clear();
    vec_cost_.clear();
    time_start_ = ros::Time::now();

    nlopt::result result = opt.optimize(q, final_cost);

    /* ---------- get results ---------- */
    std::cout << "[Optimization]: iter num: " << iter_num_ << std::endl;
    // cout << "Min cost:" << min_cost_ << endl;

    for (int i = 0; i < control_points_.rows(); ++i)
    {
      if (i < start_id_)
        continue;

      if (end_constrain_ == HARD_CONSTRAINT && i >= end_id_)
        continue;

      for (int j = 0; j < 3; j++)
        control_points_(i, j) = best_variable_[3 * (i - start_id_) + j];
    }

    cout << "[Optimization]: end-------------" << endl;
  }
  catch (std::exception& e)
  {
    cout << "[Optimization]: nlopt exception: " << e.what() << endl;
  }
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient)
{
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Eigen::Vector3d(0, 0, 0));

  Eigen::Vector3d jerk;

  for (int i = 0; i < q.size() - order_; i++)
  {
    /* evaluate jerk */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();

    /* jerk gradient */
    gradient[i + 0] += 2.0 * jerk * (-1.0);
    gradient[i + 1] += 2.0 * jerk * (3.0);
    gradient[i + 2] += 2.0 * jerk * (-3.0);
    gradient[i + 3] += 2.0 * jerk * (1.0);
  }
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient)
{
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Eigen::Vector3d(0, 0, 0));

  double dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = end_constrain_ == SOFT_CONSTRAINT ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++)
  {
    if (!dynamic_)
    {
      edt_env_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
    }
    else
    {
      double time = double(i + 2 - order_) * bspline_interval_ + time_traj_start_;
      edt_env_->evaluateEDTWithGrad(q[i], time, dist, dist_grad);
    }

    cost += dist < dist0_ ? pow(dist - dist0_, 2) : 0.0;
    gradient[i] += dist < dist0_ ? 2.0 * (dist - dist0_) * dist_grad : g_zero;
  }
}

void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient)
{
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Eigen::Vector3d(0, 0, 0));

  /* ---------- abbreviation ---------- */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* ---------- velocity feasibility ---------- */
  for (int i = 0; i < q.size() - 1; i++)
  {
    Eigen::Vector3d vi = q[i + 1] - q[i];
    for (int j = 0; j < 3; j++)
    {
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      cost += vd > 0.0 ? pow(vd, 2) : 0.0;

      gradient[i + 0](j) += vd > 0.0 ? 2.0 * vd * ts_inv2 * (-2.0) * vi(j) : 0.0;
      gradient[i + 1](j) += vd > 0.0 ? 2.0 * vd * ts_inv2 * (2.0) * vi(j) : 0.0;
    }
  }

  /* ---------- acceleration feasibility ---------- */
  for (int i = 0; i < q.size() - 2; i++)
  {
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];
    for (int j = 0; j < 3; j++)
    {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      cost += ad > 0.0 ? pow(ad, 2) : 0.0;

      gradient[i + 0](j) += ad > 0.0 ? 2.0 * ad * ts_inv4 * (2.0) * ai(j) : 0.0;
      gradient[i + 1](j) += ad > 0.0 ? 2.0 * ad * ts_inv4 * (-4.0) * ai(j) : 0.0;
      gradient[i + 2](j) += ad > 0.0 ? 2.0 * ad * ts_inv4 * (2.0) * ai(j) : 0.0;
    }
  }
}

void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient)
{
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Eigen::Vector3d(0, 0, 0));

  // zero cost and gradient in hard constraints
  if (end_constrain_ == SOFT_CONSTRAINT)
  {
    Eigen::Vector3d q_3, q_2, q_1, qd;
    q_3 = q[q.size() - 3];
    q_2 = q[q.size() - 2];
    q_1 = q[q.size() - 1];

    qd = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
    cost += qd.squaredNorm();

    gradient[q.size() - 3] += 2 * qd * (1 / 6.0);
    gradient[q.size() - 2] += 2 * qd * (4 / 6.0);
    gradient[q.size() - 1] += 2 * qd * (1 / 6.0);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad, double& f_combine)
{
  /* ---------- convert to control point vector ---------- */
  vector<Eigen::Vector3d> q;
  // q.resize(control_points_.rows());

  /* first p points */
  for (int i = 0; i < order_; i++)
    q.push_back(control_points_.row(i));

  /* optimized control points */
  for (int i = 0; i < variable_num_ / 3; i++)
  {
    Eigen::Vector3d qi(x[3 * i], x[3 * i + 1], x[3 * i + 2]);
    q.push_back(qi);
  }

  /* last p points */
  if (end_constrain_ == END_CONSTRAINT::HARD_CONSTRAINT)
  {
    for (int i = 0; i < order_; i++)
      q.push_back(control_points_.row(control_points_.rows() - order_ + i));
  }

  /* ---------- evaluate cost and gradient ---------- */
  double f_smoothness, f_distance, f_feasibility, f_endpoint;

  vector<Eigen::Vector3d> g_smoothness, g_distance, g_feasibility, g_endpoint;
  g_smoothness.resize(control_points_.rows());
  g_distance.resize(control_points_.rows());
  g_feasibility.resize(control_points_.rows());
  g_endpoint.resize(control_points_.rows());

  calcSmoothnessCost(q, f_smoothness, g_smoothness);
  calcDistanceCost(q, f_distance, g_distance);
  calcFeasibilityCost(q, f_feasibility, g_feasibility);
  calcEndpointCost(q, f_endpoint, g_endpoint);

  /* ---------- convert to NLopt format...---------- */
  grad.resize(variable_num_);

  f_combine = lamda1_ * f_smoothness + lamda2_ * f_distance + lamda3_ * f_feasibility + lamda4_ * f_endpoint;

  for (int i = 0; i < variable_num_ / 3; i++)
    for (int j = 0; j < 3; j++)
    {
      /* the first p points is static here */
      grad[3 * i + j] = lamda1_ * g_smoothness[i + order_](j) + lamda2_ * g_distance[i + order_](j) +
                        lamda3_ * g_feasibility[i + order_](j) + lamda4_ * g_endpoint[i + order_](j);
    }

  /* ---------- print cost ---------- */
  iter_num_ += 1;

  if (iter_num_ % 100 == 0)
  {
    // cout << iter_num_ << " smooth: " << lamda1_ * f_smoothness << " , dist: " << lamda2_ * f_distance
    //      << ", fea: " << lamda3_ * f_feasibility << ", end: " << lamda4_ * f_endpoint << ", total: " << f_combine
    //      << endl;
  }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data)
{
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);

  double cost;
  opt->combineCost(x, grad, cost);

  /* save the min cost result */
  if (cost < opt->min_cost_)
  {
    opt->min_cost_ = cost;
    opt->best_variable_ = x;
  }

  return cost;

  // /* ---------- evaluation ---------- */

  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

}  // namespace dyn_planner