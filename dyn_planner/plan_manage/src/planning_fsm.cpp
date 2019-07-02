
#include <plan_manage/planning_fsm.h>

namespace dyn_planner
{
void PlanningFSM::init(ros::NodeHandle& nh)
{
  /* ---------- init global param---------- */
  nh.param("bspline/limit_vel", NonUniformBspline::limit_vel_, -1.0);
  nh.param("bspline/limit_acc", NonUniformBspline::limit_acc_, -1.0);
  nh.param("bspline/limit_ratio", NonUniformBspline::limit_ratio_, -1.0);

  /* ---------- fsm param ---------- */
  nh.param("fsm/flight_type", flight_type_, -1);
  nh.param("fsm/thresh_replan", thresh_replan_, -1.0);
  nh.param("fsm/thresh_no_replan", thresh_no_replan_, -1.0);
  nh.param("fsm/wp_num", wp_num_, -1);

  for (int i = 0; i < wp_num_; i++)
  {
    nh.param("fsm/wp" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/wp" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/wp" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  current_wp_ = 0;
  exec_state_ = EXEC_STATE::INIT;
  have_goal_ = false;

  /* ---------- init edt environment ---------- */
  sdf_map_.reset(new SDFMap);
  sdf_map_->init(nh);

  edt_env_.reset(new EDTEnvironment);
  edt_env_->setMap(sdf_map_);

  /* ---------- init path finder and optimizer ---------- */
  path_finder0_.reset(new Astar);
  path_finder0_->setParam(nh);
  path_finder0_->setEnvironment(edt_env_);
  path_finder0_->init();

  path_finder_.reset(new KinodynamicAstar);
  path_finder_->setParam(nh);
  path_finder_->setEnvironment(edt_env_);
  path_finder_->init();

  bspline_optimizer_.reset(new BsplineOptimizer);
  bspline_optimizer_->setParam(nh);
  bspline_optimizer_->setEnvironment(edt_env_);

  planner_manager_.reset(new DynPlannerManager);
  planner_manager_->setParam(nh);
  planner_manager_->setPathFinder0(path_finder0_);
  planner_manager_->setPathFinder(path_finder_);
  planner_manager_->setOptimizer(bspline_optimizer_);
  planner_manager_->setEnvironment(edt_env_);

  visualization_.reset(new PlanningVisualization(nh));

  /* ---------- callback ---------- */
  exec_timer_ = node_.createTimer(ros::Duration(0.01), &PlanningFSM::execFSMCallback, this);

  safety_timer_ = node_.createTimer(ros::Duration(0.1), &PlanningFSM::safetyCallback, this);

  waypoint_sub_ = node_.subscribe("/waypoint_generator/waypoints", 1, &PlanningFSM::waypointCallback, this);

  replan_pub_ = node_.advertise<std_msgs::Empty>("planning/replan", 10);
  bspline_pub_ = node_.advertise<plan_manage::Bspline>("planning/bspline", 10);
}


void PlanningFSM::waypointCallback(const nav_msgs::PathConstPtr& msg)
{
  if (msg->poses[0].pose.position.z < 0.0)
    return;

  cout << "Triggered!" << endl;
  trigger_ = true;

  if (flight_type_ == FLIGHT_TYPE::MANUAL_GOAL)
  {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
  }
  else if (flight_type_ == FLIGHT_TYPE::PRESET_GOAL)
  {
    end_pt_(0) = waypoints_[current_wp_][0];
    end_pt_(1) = waypoints_[current_wp_][1];
    end_pt_(2) = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % wp_num_;
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_goal_ = true;

  if (exec_state_ == WAIT_GOAL)
    changeExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeExecState(REPLAN_TRAJ, "TRIG");
}

void PlanningFSM::changeExecState(EXEC_STATE new_state, string pos_call)
{
  string state_str[5] = { "INIT", "WAIT_GOAL", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void PlanningFSM::printExecState()
{
  string state_str[5] = { "INIT", "WAIT_GOAL", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void PlanningFSM::execFSMCallback(const ros::TimerEvent& e)
{
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100)
  {
    printExecState();
    if (!edt_env_->odomValid())
      cout << "no odom." << endl;
    if (!edt_env_->mapValid())
      cout << "no map." << endl;
    if (!trigger_)
      cout << "wait for goal." << endl;
    fsm_num = 0;
  }
  switch (exec_state_)
  {
    case INIT:
    {
      if (!edt_env_->odomValid())
      {
        return;
      }
      if (!edt_env_->mapValid())
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeExecState(WAIT_GOAL, "FSM");
      break;
    }

    case WAIT_GOAL:
    {
      if (!have_goal_)
        return;
      else
      {
        changeExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      nav_msgs::Odometry odom = edt_env_->getOdom();
      start_pt_(0) = odom.pose.pose.position.x;
      start_pt_(1) = odom.pose.pose.position.y;
      start_pt_(2) = odom.pose.pose.position.z;

      start_vel_(0) = odom.twist.twist.linear.x;
      start_vel_(1) = odom.twist.twist.linear.y;
      start_vel_(2) = odom.twist.twist.linear.z;
      start_acc_.setZero();

      bool success = planSearchOpt();
      if (success)
      {
        changeExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        // have_goal_ = false;
        // changeExecState(WAIT_GOAL, "FSM");
        changeExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - planner_manager_->time_traj_start_).toSec();
      t_cur = min(planner_manager_->traj_duration_, t_cur);
      Eigen::Vector3d pos = planner_manager_->traj_pos_.evaluateDeBoor(planner_manager_->t_start_ + t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > planner_manager_->traj_duration_ - 1e-2)
      {
        have_goal_ = false;
        changeExecState(WAIT_GOAL, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < thresh_no_replan_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((planner_manager_->pos_traj_start_ - pos).norm() < thresh_replan_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - planner_manager_->time_traj_start_).toSec();
      start_pt_ = planner_manager_->traj_pos_.evaluateDeBoor(planner_manager_->t_start_ + t_cur);
      start_vel_ = planner_manager_->traj_vel_.evaluateDeBoor(planner_manager_->t_start_ + t_cur);
      start_acc_ = planner_manager_->traj_acc_.evaluateDeBoor(planner_manager_->t_start_ + t_cur);
      // cout << "t_cur: " << t_cur << endl;
      // cout << "start pt: " << start_pt_.transpose() << endl;

      /* inform server */
      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = planSearchOpt();
      if (success)
      {
        changeExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        // have_goal_ = false;
        // changeExecState(WAIT_GOAL, "FSM");
        changeExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void PlanningFSM::safetyCallback(const ros::TimerEvent& e)
{
  /* ---------- check goal safety ---------- */
  if (have_goal_)
  {
    double dist =
        planner_manager_->dynamic_ ?
            edt_env_->evaluateCoarseEDT(end_pt_, planner_manager_->time_start_ + planner_manager_->traj_duration_) :
            edt_env_->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= planner_manager_->margin_)
    {
      /* try to find a max distance goal around */
      bool new_goal = false;
      const double dr = 0.5, dtheta = 30, dz = 0.3;
      double new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;
      for (double r = dr; r <= 5 * dr + 1e-3; r += dr)
      {
        for (double theta = -90; theta <= 270; theta += dtheta)
        {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz)
          {
            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + dz;
            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->dynamic_ ?
                       edt_env_->evaluateCoarseEDT(new_pt,
                                                   planner_manager_->time_start_ + planner_manager_->traj_duration_) :
                       edt_env_->evaluateCoarseEDT(new_pt, -1.0);
            if (dist > max_dist)
            {
              /* reset end_pt_ */
              goal(0) = new_x;
              goal(1) = new_y;
              goal(2) = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > planner_manager_->margin_)
      {
        cout << "change goal, replan." << endl;
        end_pt_ = goal;
        have_goal_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ)
        {
          changeExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      }
      else
      {
        // have_goal_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeExecState(WAIT_GOAL, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeExecState(REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == EXEC_STATE::EXEC_TRAJ)
  {
    bool safe = planner_manager_->checkTrajCollision();

    if (!safe)
    {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool PlanningFSM::planSearchOpt()
{
  bool plan_success = planner_manager_->generateTrajectory(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  if (plan_success)
  {
    planner_manager_->retrieveTrajectory();

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = planner_manager_->time_traj_start_;
    bspline.traj_id = planner_manager_->traj_id_;
    Eigen::MatrixXd ctrl_pts = planner_manager_->traj_pos_.getControlPoint();
    for (int i = 0; i < ctrl_pts.rows(); ++i)
    {
      Eigen::Vector3d pvt = ctrl_pts.row(i);
      geometry_msgs::Point pt;
      pt.x = pvt(0);
      pt.y = pvt(1);
      pt.z = pvt(2);
      bspline.pts.push_back(pt);
    }
    Eigen::VectorXd knots = planner_manager_->traj_pos_.getKnot();
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }
    bspline_pub_.publish(bspline);

    /* visulization */
    vector<Eigen::Vector3d> kino_path = path_finder_->getKinoTraj(0.02);

    visualization_->drawPath(kino_path, 0.1, Eigen::Vector4d(1, 0, 0, 1));

    visualization_->drawBspline(planner_manager_->traj_pos_, 0.1, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true, 0.12,
                                Eigen::Vector4d(0, 1, 0, 1));
    return true;
  }
  else
  {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

// PlanningFSM::
}  // namespace dyn_planner
