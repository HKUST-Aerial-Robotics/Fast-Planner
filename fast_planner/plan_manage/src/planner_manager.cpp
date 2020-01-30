#include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace fast_planner {

FastPlannerManager::FastPlannerManager() {
}

FastPlannerManager::~FastPlannerManager() {
  std::cout << "des manager" << std::endl;
}

void FastPlannerManager::initializePlanningModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_environment_, -1);
  nh.param("manager/clearance_threshold", pp_.clearance_threshold_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_segment_length_, -1.0);
  nh.param("manager/control_points_distance", pp_.control_points_distance_, -1.0);

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_topo_path", use_topo_path, false);
  nh.param("manager/use_optimization", use_optimization, false);
  nh.param("manager/use_active_perception", use_active_perception, false);

  traj_info_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    geometric_path_finder_.reset(new Astar);
    geometric_path_finder_->setParam(nh);
    geometric_path_finder_->setEnvironment(edt_environment_);
    geometric_path_finder_->init();
  }

  if (use_kinodynamic_path) {
    kinodynamic_path_finder_.setParam(nh);
    kinodynamic_path_finder_.setEnvironment(edt_environment_);
    kinodynamic_path_finder_.init();
  }

  if (use_optimization) {
    for (int i = 0; i < 10; ++i) {
      BsplineOptimizer optimizer;
      optimizer.setParam(nh);
      optimizer.setEnvironment(edt_environment_);
      bspline_optimizers_.push_back(optimizer);
    }
  }
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance) {

  double t_now = (ros::Time::now() - traj_info_.time_traj_start_).toSec();

  double tm, tmp;
  traj_info_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = traj_info_.position_traj_.evaluateDeBoor(tm + t_now);

  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < traj_info_.traj_duration_) {
    fut_pt = traj_info_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.09) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

EDTEnvironment::Ptr FastPlannerManager::getEDTEnvironment() {
  return edt_environment_;
}

PlanParameters* FastPlannerManager::getPlanParameters() {
  return &pp_;
}

LocalTrajectoryInfo* FastPlannerManager::getLocalTrajectoryInfo() {
  return &traj_info_;
}

IntermediatePlanData* FastPlannerManager::getIntermediatePlanData() {
  return &plan_data_;
}

bool FastPlannerManager::kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel) {

  std::cout << "[kino replan]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
       << start_acc.transpose() << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose()
       << endl;

  if ((start_pt - end_pt).norm() < 0.2) {
    cout << "Close goal" << endl;
    return false;
  }

  ros::Time t1, t2;

  traj_info_.time_traj_start_ = ros::Time::now();
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // kinodynamic path searching

  t1 = ros::Time::now();

  kinodynamic_path_finder_.reset();

  int status = kinodynamic_path_finder_.search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

  if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    kinodynamic_path_finder_.reset();
    status = kinodynamic_path_finder_.search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success." << endl;
  }

  plan_data_.kino_path_ = kinodynamic_path_finder_.getKinoTraj(0.01);

  t_search = (ros::Time::now() - t1).toSec();

  // parameterize the path to bspline

  double ts = pp_.control_points_distance_ / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kinodynamic_path_finder_.getSamples(ts, point_set, start_end_derivatives);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  NonUniformBspline init(ctrl_pts, 3, ts);

  // bspline trajectory optimization

  t1 = ros::Time::now();

  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  if (status != KinodynamicAstar::REACH_END) {
    cost_function |= BsplineOptimizer::ENDPOINT;
  }

  ctrl_pts = bspline_optimizers_[0].BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

  t_opt = (ros::Time::now() - t1).toSec();

  // iterative time adjustment

  t1 = ros::Time::now();
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

  double to = pos.getTimeSum();
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  bool feasible = pos.checkFeasibility(false);

  int iter_num = 0;
  while (!feasible && ros::ok()) {

    feasible = pos.reallocateTime();

    if (++iter_num >= 3) break;
  }

  // pos.checkFeasibility(true);
  // cout << "[Main]: iter num: " << iter_num << endl;

  double tn = pos.getTimeSum();

  cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  t_adjust = (ros::Time::now() - t1).toSec();

  // save planned results

  traj_info_.position_traj_ = pos;

  double t_total = t_search + t_opt + t_adjust;
  cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
       << ", adjust time:" << t_adjust << endl;

  pp_.time_search_ = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_ = t_adjust;

  updateTrajInfo();

  return true;
}

void FastPlannerManager::updateTrajInfo() {
  traj_info_.velocity_traj_ = traj_info_.position_traj_.getDerivative();
  traj_info_.acceleration_traj_ = traj_info_.velocity_traj_.getDerivative();

  traj_info_.position_traj_.getTimeSpan(traj_info_.t_start_, traj_info_.t_end_);
  traj_info_.pos_traj_start_ = traj_info_.position_traj_.evaluateDeBoor(traj_info_.t_start_);
  traj_info_.traj_duration_ = traj_info_.t_end_ - traj_info_.t_start_;

  traj_info_.traj_id_ += 1;
}

void FastPlannerManager::reparameterizeBspline(NonUniformBspline& bspline, double ratio,
                                               Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  /* extend time and reparameterize */
  double time_origin = bspline.getTimeSum();
  int seg_num = bspline.getControlPoint().rows() - 3;

  bspline.lengthenTime(ratio);

  double duration = bspline.getTimeSum();
  dt = duration / double(seg_num);
  time_inc = duration - time_origin;

  double tm, tmp;
  bspline.getTimeSpan(tm, tmp);
  vector<Eigen::Vector3d> point_set;

  for (double time = tm; time <= tmp + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoor(time));
  }

  NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                           ctrl_pts);
}

void FastPlannerManager::planHeading(const Eigen::Vector3d& start_yaw) {

  auto t1 = ros::Time::now();

  // calculate waypoints of heading

  auto& pos = traj_info_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw = 0.3;
  int seg_num = ceil(duration / dt_yaw);
  dt_yaw = duration / seg_num;

  const double forward_t = 2.0;
  double last_yaw = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 1; i < seg_num; ++i) {
    double tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);

    double tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-3) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));

    } else {
      waypt = waypts.back();
    }

    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;

  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = traj_info_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));

  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve

  bspline_optimizers_[1].setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;

  yaw = bspline_optimizers_[1].BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 3, 3);

  // update traj info

  traj_info_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  traj_info_.yawdot_traj_ = traj_info_.yaw_traj_.getDerivative();
  traj_info_.yawdotdot_traj_ = traj_info_.yawdot_traj_.getDerivative();

  std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
