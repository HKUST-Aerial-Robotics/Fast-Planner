#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
// using namespace std;

namespace fast_planner {
void BsplineOptimizer::setControlPoints(Eigen::MatrixXd points) {
  this->control_points_ = points;
  this->start_id_ = order_;
  this->end_id_ = this->control_points_.rows() - order_;
}

void BsplineOptimizer::setOptimizationRange(int start, int end) {
  this->start_id_ = min(max(start, order_), int(control_points_.rows() - order_));
  this->end_id_ = min(max(end, order_), int(control_points_.rows() - order_));
  cout << "opt range:" << this->start_id_ << ", " << this->end_id_ << endl;
}

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  nh.param("optimization/lambda1", lambda1_, -1.0);
  nh.param("optimization/lambda2", lambda2_, -1.0);
  nh.param("optimization/lambda3", lambda3_, -1.0);
  nh.param("optimization/lambda4", lambda4_, -1.0);
  nh.param("optimization/lambda5", lambda5_, -1.0);
  nh.param("optimization/lambda6", lambda6_, -1.0);
  nh.param("optimization/dist0", dist0_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/algorithm1", algorithm1_, -1);
  nh.param("optimization/algorithm2", algorithm2_, -1);
  nh.param("optimization/order", order_, -1);
  nh.param("optimization/visib_min", visib_min_, -1.0);
  nh.param("optimization/visible_num", visible_num_, -1);
}

void BsplineOptimizer::setBSplineInterval(double ts) {
  this->bspline_interval_ = ts;
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() {
  return this->control_points_;
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) {
  guide_pts_ = guide_pt;
}

void BsplineOptimizer::setOptimizationPhase(int obj) {
  optimization_phase_ = obj;
}

/* best algorithm_ is 40: SLSQP(constrained), 11 LBFGS(unconstrained barrier
method */
void BsplineOptimizer::optimize(int end_cons, bool dynamic, double time_limit, double time_start) {
  /* initialize solver */
  end_constrain_ = end_cons;
  dynamic_ = dynamic;
  time_traj_start_ = time_start;
  iter_num_ = 0;
  min_cost_ = std::numeric_limits<double>::max();

  vector<Eigen::Vector3d> zeros_grads(control_points_.rows());
  fill(zeros_grads.begin(), zeros_grads.end(), Eigen::Vector3d(0, 0, 0));
  g_q_ = zeros_grads;
  g_smoothness_ = zeros_grads;
  g_distance_ = zeros_grads;
  g_feasibility_ = zeros_grads;
  g_endpoint_ = zeros_grads;
  g_guide_ = zeros_grads;
  g_visib_ = zeros_grads;

  if (end_constrain_ == HARD_CONSTRAINT) {
    variable_num_ = 3 * (end_id_ - start_id_);
  } else if (end_constrain_ == SOFT_CONSTRAINT) {
    variable_num_ = 3 * (control_points_.rows() - start_id_);

    // end position used for hard constraint
    end_pt_ = (1 / 6.0) *
        (control_points_.row(control_points_.rows() - 3) +
         4 * control_points_.row(control_points_.rows() - 2) +
         control_points_.row(control_points_.rows() - 1));
    std::cout << "end pt" << std::endl;
  }

  /* do optimization using NLopt slover */
  nlopt::opt opt(nlopt::algorithm(optimization_phase_ == FIRST_PHASE ? algorithm1_ : algorithm2_),
                 variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_[optimization_phase_ - 1]);
  opt.set_xtol_rel(1e-5);
  opt.set_maxtime(time_limit);

  vector<double> q(variable_num_);
  double final_cost;
  for (int i = start_id_; i < int(control_points_.rows()); ++i) {
    if (end_constrain_ == HARD_CONSTRAINT && i >= end_id_) continue;
    for (int j = 0; j < 3; j++) {
      q[3 * (i - start_id_) + j] = control_points_(i, j);
    }
  }

  vector<double> lb(variable_num_), ub(variable_num_);
  const double bound = 10.0;
  for (int i = 0; i < variable_num_; ++i) {
    lb[i] = q[i] - bound;
    ub[i] = q[i] + bound;
  }
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);

  try {
    // cout << fixed << setprecision(7);
    // vec_time_.clear();
    // vec_cost_.clear();
    // time_start_ = ros::Time::now();

    nlopt::result result = opt.optimize(q, final_cost);

    /* retrieve the optimization result */
    // cout << "Min cost:" << min_cost_ << endl;
  } catch (std::exception& e) {
    ROS_ERROR("[Optimization]: nlopt exception");
    cout << e.what() << endl;
  }

  for (int i = start_id_; i < control_points_.rows(); ++i) {
    if (end_constrain_ == HARD_CONSTRAINT && i >= end_id_) continue;
    for (int j = 0; j < 3; j++) {
      control_points_(i, j) = best_variable_[3 * (i - start_id_) + j];
    }
  }

  if (optimization_phase_ > 1) std::cout << "[Optimization]: iter num: " << iter_num_ << std::endl;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(Eigen::MatrixXd points, double ts, int obj,
                                                      double time_limit,
                                                      const vector<Eigen::Vector3d>& guide_pt) {
  setControlPoints(points);
  setBSplineInterval(ts);
  setOptimizationPhase(obj);
  setGuidePath(guide_pt);

  optimize(HARD_CONSTRAINT, false, time_limit);
  return this->control_points_;
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Eigen::Vector3d(0, 0, 0));

  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - order_; i++) {
    /* evaluate jerk */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();

    temp_j = 2.0 * jerk;
    /* jerk gradient */
    gradient[i + 0] += -temp_j;
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;
  }
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Eigen::Vector3d(0, 0, 0));

  double dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = end_constrain_ == SOFT_CONSTRAINT ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    if (!dynamic_) {
      edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
      if (dist_grad.norm() > 1e-4) dist_grad.normalize();
    } else {
      double time = double(i + 2 - order_) * bspline_interval_ + time_traj_start_;
      edt_environment_->evaluateEDTWithGrad(q[i], time, dist, dist_grad);
    }

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}

void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Eigen::Vector3d(0, 0, 0));

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (int i = 0; i < q.size() - 1; i++) {
    Eigen::Vector3d vi = q[i + 1] - q[i];
    for (int j = 0; j < 3; j++) {
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {
        cost += pow(vd, 2);

        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  /* acceleration feasibility */
  for (int i = 0; i < q.size() - 2; i++) {
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];
    for (int j = 0; j < 3; j++) {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += pow(ad, 2);

        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
}

void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Eigen::Vector3d(0, 0, 0));

  // zero cost and gradient in hard constraints
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

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /* convert the NLopt format vector to control points. */

  // vector<Eigen::Vector3d> q;
  for (int i = 0; i < order_; i++) {
    // q.push_back(control_points_.row(i));
    g_q_[i] = control_points_.row(i);
  }
  for (int i = 0; i < variable_num_ / 3; i++) {
    // Eigen::Vector3d qi(x[3 * i], x[3 * i + 1], x[3 * i + 2]);
    // q.push_back(qi);
    g_q_[i + order_] = Eigen::Vector3d(x[3 * i], x[3 * i + 1], x[3 * i + 2]);
  }
  if (end_constrain_ == END_CONSTRAINT::HARD_CONSTRAINT) {
    for (int i = 0; i < order_; i++)
      // q.push_back(control_points_.row(control_points_.rows() - order_ + i));
      g_q_[order_ + variable_num_ / 3 + i] = control_points_.row(control_points_.rows() - order_ + i);
  }

  /*  evaluate costs and their gradient  */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_visib;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_visib = 0.0;

  if (optimization_phase_ == FIRST_PHASE) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);

    f_combine = lambda1_ * f_smoothness + lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / 3; i++)
      for (int j = 0; j < 3; j++)
        grad[3 * i + j] = lambda1_ * g_smoothness_[i + order_](j) + lambda5_ * g_guide_[i + order_](j);

  } else if (optimization_phase_ == SECOND_PHASE || optimization_phase_ == VISIB_PHASE) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    calcDistanceCost(g_q_, f_distance, g_distance_);
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);

    f_combine = lambda1_ * f_smoothness + lambda2_ * f_distance + lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / 3; i++)
      for (int j = 0; j < 3; j++)
        grad[3 * i + j] = lambda1_ * g_smoothness_[i + order_](j) +
            lambda2_ * g_distance_[i + order_](j) + lambda3_ * g_feasibility_[i + order_](j);
  }

  if (end_constrain_ == SOFT_CONSTRAINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);

    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / 3; i++)
      for (int j = 0; j < 3; j++)
        grad[3 * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  }

  /*  print cost  */
  // if (optimization_phase_ == SECOND_PHASE) {
  //   cout << iter_num_ << ", total: " << f_combine
  //        << ", smooth: " << lambda1_ * f_smoothness
  //        << " , dist:" << lambda2_ * f_distance
  //        << ", fea: " << lambda3_ * f_feasibility << endl;
  //   // << ", end: " << lambda4_ * f_endpoint
  //   // << ", guide: " << lambda5_ * f_guide
  // }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_ = cost;
    opt->best_variable_ = x;
  }
  return cost;

  // /* evaluation */
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

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

}  // namespace fast_planner