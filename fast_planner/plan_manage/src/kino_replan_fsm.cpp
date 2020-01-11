
#include <plan_manage/kino_replan_fsm.h>

namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_ = 0;
  exec_state_ = FSM_EXEC_STATE::INIT;
  have_target_ = false;

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initializePlanningModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0) = waypoints_[current_wp_][0];
    end_pt_(1) = waypoints_[current_wp_][1];
    end_pt_(2) = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  drone_odometry_ = *msg;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!planner_manager_->checkOdometryAvailable()) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!planner_manager_->checkOdometryAvailable()) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_(0) = drone_odometry_.pose.pose.position.x;
      start_pt_(1) = drone_odometry_.pose.pose.position.y;
      start_pt_(2) = drone_odometry_.pose.pose.position.z;

      start_vel_(0) = drone_odometry_.twist.twist.linear.x;
      start_vel_(1) = drone_odometry_.twist.twist.linear.y;
      start_vel_(2) = drone_odometry_.twist.twist.linear.z;
      start_acc_.setZero();

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajectoryInfo* info = planner_manager_->getLocalTrajectoryInfo();
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->time_traj_start_).toSec();
      t_cur = min(info->traj_duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoor(info->t_start_ + t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->traj_duration_ - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // cout << "near end" << endl;
        return;

      } else if ((info->pos_traj_start_ - pos).norm() < replan_thresh_) {
        // cout << "near start" << endl;
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajectoryInfo* info = planner_manager_->getLocalTrajectoryInfo();
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->time_traj_start_).toSec();

      start_pt_ = info->position_traj_.evaluateDeBoor(info->t_start_ + t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoor(info->t_start_ + t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoor(info->t_start_ + t_cur);
      // cout << "t_cur: " << t_cur << endl;
      // cout << "start pt: " << start_pt_.transpose() << endl;

      /* inform server */
      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajectoryInfo* info = planner_manager_->getLocalTrajectoryInfo();

  if (have_target_) {
    auto edt_env = planner_manager_->getEDTEnvironment();

    double dist = planner_manager_->getPlanParameters()->dynamic_environment_ ?
        edt_env->evaluateCoarseEDT(end_pt_, info->time_start_ + info->traj_duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.1) {
      /* try to find a max distance goal around */
      bool new_goal = false;
      const double dr = 0.5, dtheta = 30, dz = 0.3;
      double new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + dz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->getPlanParameters()->dynamic_environment_ ?
                edt_env->evaluateCoarseEDT(new_pt, info->time_start_ + info->traj_duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0) = new_x;
              goal(1) = new_y;
              goal(2) = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.1) {
        cout << "change goal, replan." << endl;
        end_pt_ = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  if (plan_success) {

    planner_manager_->updateTrajectoryInfo();
    auto info = planner_manager_->getLocalTrajectoryInfo();

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->time_traj_start_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd ctrl_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < ctrl_pts.rows(); ++i) {
      Eigen::Vector3d pvt = ctrl_pts.row(i);
      geometry_msgs::Point pt;

      pt.x = pvt(0);
      pt.y = pvt(1);
      pt.z = pvt(2);
      bspline.pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    /* visulization */
    auto plan_data = planner_manager_->getIntermediatePlanData();

    visualization_->drawGeometricPath(plan_data->kino_path_, 0.1, Eigen::Vector4d(1, 0, 0, 1));

    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 1.0, 0.0, 1), false,
                                0.15, Eigen::Vector4d(1, 0, 0, 1));
    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

// KinoReplanFSM::
}  // namespace fast_planner
