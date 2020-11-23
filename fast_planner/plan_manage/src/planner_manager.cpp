/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace fast_planner {

// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() { std::cout << "des manager" << std::endl; }

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_topo_path", use_topo_path, false);
  nh.param("manager/use_optimization", use_optimization, false);

  local_data_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    geo_path_finder_.reset(new Astar);
    geo_path_finder_->setParam(nh);
    geo_path_finder_->setEnvironment(edt_environment_);
    geo_path_finder_->init();
  }

  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
  }
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance) {

  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  double tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

  double          radius = 0.0;
  Eigen::Vector3d fut_pt;
  double          fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.1) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

// !SECTION

// SECTION kinodynamic replanning

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

  local_data_.start_time_ = ros::Time::now();
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // kinodynamic path searching

  t1 = ros::Time::now();

  kino_path_finder_->reset();

  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

  if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success." << endl;
  }

  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  t_search = (ros::Time::now() - t1).toSec();

  // parameterize the path to bspline

  double                  ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  NonUniformBspline init(ctrl_pts, 3, ts);

  // bspline trajectory optimization

  t1 = ros::Time::now();

  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  if (status != KinodynamicAstar::REACH_END) {
    cost_function |= BsplineOptimizer::ENDPOINT;
  }

  ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

  t_opt = (ros::Time::now() - t1).toSec();

  // iterative time adjustment

  t1                    = ros::Time::now();
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

  local_data_.position_traj_ = pos;

  double t_total = t_search + t_opt + t_adjust;
  cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
       << ", adjust time:" << t_adjust << endl;

  pp_.time_search_   = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_   = t_adjust;

  updateTrajInfo();

  return true;
}

// !SECTION

// SECTION topological replanning

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  plan_data_.clearTopoPaths();

  // generate global reference trajectory

  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) std::cout << "no global waypoints!" << std::endl;

  points.insert(points.begin(), start_pos);

  // insert intermediate points if too far
  vector<Eigen::Vector3d> inter_points;
  const double            dist_thresh = 4.0;

  for (int i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;

      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  // write position matrix
  int             pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
  }

  time(0) *= 2.0;
  time(0) = max(1.0, time(0));
  time(time.rows() - 1) *= 2.0;
  time(time.rows() - 1) = max(1.0, time(time.rows() - 1));

  PolynomialTraj gl_traj = minSnapTraj(pos, zero, zero, zero, zero, time);

  auto time_now = ros::Time::now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  // truncate a local trajectory

  double            dt, duration;
  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, 3, dt);

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_    = time_now;
  ROS_INFO("global trajectory generated.");

  updateTrajInfo();

  return true;
}

bool FastPlannerManager::topoReplan(bool collide) {
  ros::Time t1, t2;

  /* truncate a new local segment for replanning */
  ros::Time time_now = ros::Time::now();
  double    t_now    = (time_now - global_data_.global_start_time_).toSec();
  double    local_traj_dt, local_traj_duration;
  double    time_inc = 0.0;

  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration);
  NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
  local_data_.start_time_ = time_now;

  if (!collide) {  // simply truncate the segment and do nothing
    refineTraj(init_traj, time_inc);
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);

  } else {
    plan_data_.initial_local_segment_ = init_traj;
    vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
    findCollisionRange(colli_start, colli_end, start_pts, end_pts);

    if (colli_start.size() == 1 && colli_end.size() == 0) {
      ROS_WARN("Init traj ends in obstacle, no replanning.");
      local_data_.position_traj_ = init_traj;
      global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);

    } else {
      NonUniformBspline best_traj;

      // local segment is in collision, call topological replanning
      /* search topological distinctive paths */
      ROS_INFO("[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr>            graph;
      vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      topo_prm_->findTopoPaths(colli_start.front(), colli_end.back(), start_pts, end_pts, graph,
                               raw_paths, filtered_paths, select_paths);

      if (select_paths.size() == 0) {
        ROS_WARN("No path.");
        return false;
      }
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      /* optimize trajectory using different topo paths */
      ROS_INFO("[Optimize]: ---------");
      t1 = ros::Time::now();

      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      vector<thread> optimize_threads;
      for (int i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(&FastPlannerManager::optimizeTopoBspline, this, t_now,
                                      local_traj_duration, select_paths[i], i);
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      for (int i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (ros::Time::now() - t1).toSec();
      cout << "[planner]: optimization time: " << t_opt << endl;
      selectBestTraj(best_traj);
      refineTraj(best_traj, time_inc);

      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(local_data_.position_traj_, t_now,
                                local_traj_duration + time_inc + t_now, time_inc);
    }
  }
  updateTrajInfo();
  return true;
}

void FastPlannerManager::selectBestTraj(NonUniformBspline& traj) {
  // sort by jerk
  vector<NonUniformBspline>& trajs = plan_data_.topo_traj_pos2_;
  sort(trajs.begin(), trajs.end(),
       [&](NonUniformBspline& tj1, NonUniformBspline& tj2) { return tj1.getJerk() < tj2.getJerk(); });
  traj = trajs[0];
}

void FastPlannerManager::refineTraj(NonUniformBspline& best_traj, double& time_inc) {
  ros::Time t1 = ros::Time::now();
  time_inc     = 0.0;
  double    dt, t_inc;
  const int max_iter = 1;

  // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
  Eigen::MatrixXd ctrl_pts      = best_traj.getControlPoint();
  int             cost_function = BsplineOptimizer::NORMAL_PHASE;

  best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = best_traj.checkRatio();
  std::cout << "ratio: " << ratio << std::endl;
  reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
  time_inc += t_inc;

  ctrl_pts  = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
  best_traj = NonUniformBspline(ctrl_pts, 3, dt);
  ROS_WARN_STREAM("[Refine]: cost " << (ros::Time::now() - t1).toSec()
                                    << " seconds, time change is: " << time_inc);
}

void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void FastPlannerManager::reparamBspline(NonUniformBspline& bspline, double ratio,
                                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  int    prev_num    = bspline.getControlPoint().rows();
  double time_origin = bspline.getTimeSum();
  int    seg_num     = bspline.getControlPoint().rows() - 3;
  // double length = bspline.getLength(0.1);
  // int seg_num = ceil(length / pp_.ctrl_pt_dist);

  ratio = min(1.01, ratio);
  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt              = duration / double(seg_num);
  time_inc        = duration - time_origin;

  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                           ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

void FastPlannerManager::optimizeTopoBspline(double start_t, double duration,
                                             vector<Eigen::Vector3d> guide_path, int traj_id) {
  ros::Time t1;
  double    tm1, tm2, tm3;

  t1 = ros::Time::now();

  // parameterize B-spline according to the length of guide path
  int             seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
  Eigen::MatrixXd ctrl_pts;
  double          dt;

  ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt);
  // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

  // discretize the guide path and align it with B-spline control points
  vector<Eigen::Vector3d> guide_pt;
  guide_pt = topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2);

  guide_pt.pop_back();
  guide_pt.pop_back();
  guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);

  // std::cout << "guide pt num: " << guide_pt.size() << std::endl;
  if (guide_pt.size() != int(ctrl_pts.rows()) - 6) ROS_WARN("what guide");

  tm1 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // first phase, path-guided optimization

  bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
  Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);

  plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(opt_ctrl_pts1, 3, dt);

  tm2 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // second phase, normal optimization

  Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      opt_ctrl_pts1, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);

  plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(opt_ctrl_pts2, 3, dt);

  tm3 = (ros::Time::now() - t1).toSec();
  ROS_INFO("optimization %d cost %lf, %lf, %lf seconds.", traj_id, tm1, tm2, tm3);
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double& dt, double& duration) {
  /* get the sample points local traj within radius */

  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByRadius(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
                               start_end_derivative, dt, duration);

  /* parameterization of B-spline */

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  plan_data_.local_start_end_derivative_ = start_end_derivative;
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double duration, int seg_num,
                                                     double& dt) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  /* parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

void FastPlannerManager::findCollisionRange(vector<Eigen::Vector3d>& colli_start,
                                            vector<Eigen::Vector3d>& colli_end,
                                            vector<Eigen::Vector3d>& start_pts,
                                            vector<Eigen::Vector3d>& end_pts) {
  bool               last_safe = true, safe;
  double             t_m, t_mp;
  NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  /* find range of collision */
  double t_s = -1.0, t_e;
  for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {

    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
      if (t_s < 0.0) t_s = tc - 0.05;
    } else if (!last_safe && safe) {
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
  }

  if (colli_start.size() == 0) return;

  if (colli_start.size() == 1 && colli_end.size() == 0) return;

  /* find start and end safe segment */
  double dt = initial_traj->getInterval();
  int    sn = ceil((t_s - t_m) / dt);
  dt        = (t_s - t_m) / sn;

  for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  dt = initial_traj->getInterval();
  sn = ceil((t_mp - t_e) / dt);
  dt = (t_mp - t_e) / sn;
  // std::cout << "dt: " << dt << std::endl;
  // std::cout << "sn: " << sn << std::endl;
  // std::cout << "t_m: " << t_m << std::endl;
  // std::cout << "t_mp: " << t_mp << std::endl;
  // std::cout << "t_s: " << t_s << std::endl;
  // std::cout << "t_e: " << t_e << std::endl;

  if (dt > 1e-4) {
    for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
      end_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
  } else {
    end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
  }
}

// !SECTION

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  ROS_INFO("plan yaw");
  auto t1 = ros::Time::now();
  // calculate waypoints of heading

  auto&  pos      = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw  = 0.3;
  int    seg_num = ceil(duration / dt_yaw);
  dt_yaw         = duration / seg_num;

  const double            forward_t = 2.0;
  double                  last_yaw  = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int>             waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
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

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

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
