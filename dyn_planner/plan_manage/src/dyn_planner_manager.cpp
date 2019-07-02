#include <plan_manage/dyn_planner_manager.h>
#include <fstream>

namespace dyn_planner
{
DynPlannerManager::~DynPlannerManager()
{
  traj_id_ = 0;
}

void DynPlannerManager::setParam(ros::NodeHandle& nh)
{
  nh.param("manager/time_sample", time_sample_, -1.0);
  nh.param("manager/max_vel", max_vel_, -1.0);
  nh.param("manager/dynamic", dynamic_, -1);
  nh.param("manager/margin", margin_, -1.0);
}

void DynPlannerManager::setPathFinder0(const Astar::Ptr& finder)
{
  path_finder0_ = finder;
}

void DynPlannerManager::setPathFinder(const KinodynamicAstar::Ptr& finder)
{
  path_finder_ = finder;
}

void DynPlannerManager::setOptimizer(const BsplineOptimizer::Ptr& optimizer)
{
  bspline_optimizer_ = optimizer;
}

void DynPlannerManager::setEnvironment(const EDTEnvironment::Ptr& env)
{
  edt_env_ = env;
}

bool DynPlannerManager::checkTrajCollision()
{
  /* check collision */
  for (double t = t_start_; t <= t_end_; t += 0.02)
  {
    Eigen::Vector3d pos = traj_pos_.evaluateDeBoor(t);
    double dist = dynamic_ ? edt_env_->evaluateCoarseEDT(pos, time_start_ + t - t_start_) :
                             edt_env_->evaluateCoarseEDT(pos, -1.0);

    if (dist < margin_)
    {
      return false;
    }
  }

  return true;
}

void DynPlannerManager::retrieveTrajectory()
{
  traj_vel_ = traj_pos_.getDerivative();
  traj_acc_ = traj_vel_.getDerivative();

  traj_pos_.getTimeSpan(t_start_, t_end_);
  pos_traj_start_ = traj_pos_.evaluateDeBoor(t_start_);
  traj_duration_ = t_end_ - t_start_;

  traj_id_ += 1;
}

void DynPlannerManager::getSolvingTime(double& ts, double& to, double& ta)
{
  ts = time_search_;
  to = time_optimize_;
  ta = time_adjust_;
}

bool DynPlannerManager::generateTrajectory(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt, Eigen::Vector3d end_vel)
{
  std::cout << "[planner]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", " << start_acc.transpose()
       << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose() << endl;

  if ((start_pt - end_pt).norm() < 0.2)
  {
    cout << "Close goal" << endl;
    return false;
  }

  time_traj_start_ = ros::Time::now();
  time_start_ = -1.0;

  double t_search = 0.0, t_sample = 0.0, t_axb = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  ros::Time t1, t2;
  t1 = ros::Time::now();
  /* ---------- search kino path ---------- */
  path_finder_->reset();

  int status = path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true, dynamic_, time_start_);
  if (status == KinodynamicAstar::NO_PATH)
  {
    cout << "[planner]: init search fail!" << endl;
    path_finder_->reset();
    status = path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false, dynamic_, time_start_);
    if (status == KinodynamicAstar::NO_PATH)
    {
      cout << "[planner]: Can't find path." << endl;
      return false;
    }
    else
    {
      cout << "[planner]: retry search success." << endl;
    }
  }
  else
  {
    cout << "[planner]: init search success." << endl;
  }

  t2 = ros::Time::now();
  t_search = (t2 - t1).toSec();

  /* ---------- bspline parameterization ---------- */
  t1 = ros::Time::now();

  int K;
  double ts = time_sample_ / max_vel_;
  Eigen::MatrixXd vel_acc;

  Eigen::MatrixXd samples = path_finder_->getSamples(ts, K);
  // cout << "ts: " << ts << endl;
  // cout << "sample:\n" << samples.transpose() << endl;

  t2 = ros::Time::now();
  t_sample = (t2 - t1).toSec();

  t1 = ros::Time::now();

  Eigen::MatrixXd control_pts;
  NonUniformBspline::getControlPointEqu3(samples, ts, control_pts);

  NonUniformBspline init = NonUniformBspline(control_pts, 3, ts);

  t2 = ros::Time::now();
  t_axb = (t2 - t1).toSec();

  /* ---------- optimize trajectory ---------- */
  t1 = ros::Time::now();

  // cout << "ctrl pts:" << control_pts << endl;

  bspline_optimizer_->setControlPoints(control_pts);
  bspline_optimizer_->setBSplineInterval(ts);

  if (status != KinodynamicAstar::REACH_END)
    bspline_optimizer_->optimize(BsplineOptimizer::SOFT_CONSTRAINT, dynamic_, time_start_);
  else
    bspline_optimizer_->optimize(BsplineOptimizer::HARD_CONSTRAINT, dynamic_, time_start_);

  control_pts = bspline_optimizer_->getControlPoints();

  t2 = ros::Time::now();
  t_opt = (t2 - t1).toSec();

  /* ---------- time adjustment ---------- */

  t1 = ros::Time::now();
  NonUniformBspline pos = NonUniformBspline(control_pts, 3, ts);

  double tm, tmp, to, tn;
  pos.getTimeSpan(tm, tmp);
  to = tmp - tm;

  bool feasible = pos.checkFeasibility(false);

  int iter_num = 0;
  while (!feasible && ros::ok())
  {
    ++iter_num;

    feasible = pos.reallocateTime();
    /* actually this not needed, converges within 10 iteration */
    if (iter_num >= 50)
      break;
  }

  // cout << "[Main]: iter num: " << iter_num << endl;
  pos.getTimeSpan(tm, tmp);
  tn = tmp - tm;
  cout << "[planner]: Reallocate ratio: " << tn / to << endl;

  t2 = ros::Time::now();
  t_adjust = (t2 - t1).toSec();

  pos.checkFeasibility(true);
  // drawVelAndAccPro(pos);

  /* save result */
  traj_pos_ = pos;

  double t_total = t_search + t_sample + t_axb + t_opt + t_adjust;

  cout << "[planner]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_sample + t_axb + t_opt
       << ", adjust time:" << t_adjust << endl;

  time_search_ = t_search;
  time_optimize_ = t_sample + t_axb + t_opt;
  time_adjust_ = t_adjust;

  time_traj_start_ = ros::Time::now();
  time_start_ = -1.0;

  return true;
}

}  // namespace dyn_planner
