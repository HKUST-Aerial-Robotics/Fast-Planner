#include "bspline_opt/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "plan_manage/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

ros::Publisher state_pub, pos_cmd_pub, traj_pub;

nav_msgs::Odometry odom;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {5.7, 5.7, 6.2};
// double vel_gain[3] = {3.4, 3.4, 4.0};
double pos_gain[3] = { 5.7, 5.7, 6.2 };
double vel_gain[3] = { 3.4, 3.4, 4.0 };

using fast_planner::NonUniformBspline;

bool receive_traj = false;
vector<NonUniformBspline> traj;
ros::Time time_traj_start;
int traj_id;
double traj_duration;
double t_cmd_start, t_cmd_end;

// yaw control
double last_yaw_;
double time_forward_;

vector<Eigen::Vector3d> traj_cmd, traj_real;

Eigen::Vector3d hover_pt;

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawState(Eigen::Vector3d pos, Eigen::Vector3d vec, int id, Eigen::Vector4d color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;
  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;
  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);
  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);
  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);
  state_pub.publish(mk_state);
}

void bsplineCallback(plan_manage::BsplineConstPtr msg) {
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  Eigen::MatrixXd ctrl_pts(msg->pts.size(), 3);
  for (int i = 0; i < msg->pts.size(); ++i) {
    Eigen::Vector3d pt;
    pt(0) = msg->pts[i].x;
    pt(1) = msg->pts[i].y;
    pt(2) = msg->pts[i].z;
    ctrl_pts.row(i) = pt.transpose();
  }

  NonUniformBspline bspline(ctrl_pts, msg->order, 0.1);
  bspline.setKnot(knots);

  time_traj_start = msg->start_time;
  traj_id = msg->traj_id;

  traj.clear();
  traj.push_back(bspline);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj[0].getTimeSpan(t_cmd_start, t_cmd_end);
  traj_duration = t_cmd_end - t_cmd_start;

  receive_traj = true;
}

void replanCallback(std_msgs::Empty msg) {
  /* reset duration */
  const double time_out = 0.01;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - time_traj_start).toSec() + time_out;
  traj_duration = min(t_stop, traj_duration);
  t_cmd_end = t_cmd_start + traj_duration;
}

void newCallback(std_msgs::Empty msg) {
  traj_cmd.clear();
  traj_real.clear();
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;

  traj_real.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real.size() > 10000) traj_real.erase(traj_real.begin(), traj_real.begin() + 1000);
}

void visCallback(const ros::TimerEvent& e) {
  // displayTrajWithColor(traj_real, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);

  displayTrajWithColor(traj_cmd, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

void cmdCallback(const ros::TimerEvent& e) {
  /* no publishing before receive traj */
  if (!receive_traj) return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - time_traj_start).toSec();

  Eigen::Vector3d pos, vel, acc, pos_forward;

  if (t_cur < traj_duration && t_cur >= 0.0) {
    pos = traj[0].evaluateDeBoor(t_cmd_start + t_cur);
    vel = traj[1].evaluateDeBoor(t_cmd_start + t_cur);
    acc = traj[2].evaluateDeBoor(t_cmd_start + t_cur);

    double t_f = min(t_cur + time_forward_, traj_duration - 1e-3);
    pos_forward = traj[0].evaluateDeBoor(t_cmd_start + t_f);
  } else if (t_cur >= traj_duration) {
    /* hover when finish traj */
    pos = traj[0].evaluateDeBoor(t_cmd_end);
    vel.setZero();
    acc.setZero();

    pos_forward = pos;
  } else {
    cout << "[Traj server]: invalid time." << endl;
  }

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  Eigen::Vector3d pos_err = pos_forward - pos;
  if (pos_err.norm() > 1e-3) {
    cmd.yaw = atan2(pos_err(1), pos_err(0));
  } else {
    cmd.yaw = last_yaw_;
  }
  cmd.yaw_dot = 1.0;

  pos_cmd_pub.publish(cmd);

  last_yaw_ = cmd.yaw;

  drawState(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  drawState(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
  drawState(pos, pos_err, 2, Eigen::Vector4d(1, 1, 0, 0.7));

  traj_cmd.push_back(pos);
  if (pos.size() > 10000) traj_cmd.erase(traj_cmd.begin(), traj_cmd.begin() + 1000);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);

  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);

  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);

  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  state_pub = node.advertise<visualization_msgs::Marker>("planning/state", 10);
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
