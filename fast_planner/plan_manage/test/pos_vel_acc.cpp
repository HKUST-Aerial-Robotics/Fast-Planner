#include <ros/ros.h>

#include <dyn_planner/non_uniform_bspline.h>

#include <visualization_msgs/Marker.h>

using namespace fast_planner;

ros::Publisher pos_pub_;
ros::Publisher vel_pub_;
ros::Publisher acc_pub_;

int pub_id;

void displaySphereList(vector<Eigen::Vector3d> list, double resolution, Eigen::Vector4d color, int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  // traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0, mk.pose.orientation.y = 0.0, mk.pose.orientation.z = 0.0,
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2), mk.color.a = color(3);
  mk.scale.x = resolution, mk.scale.y = resolution, mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0), pt.y = list[i](1), pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  // traj_pub.publish(mk);
  if (pub_id == 0) {
    pos_pub_.publish(mk);
  } else if (pub_id == 1) {
    vel_pub_.publish(mk);
  } else if (pub_id == 2) {
    acc_pub_.publish(mk);
  }
}

void drawBspline(NonUniformBspline bspline, double size, Eigen::Vector4d color, bool show_ctrl_pts,
                 double size2, Eigen::Vector4d color2, int id1, int id2) {
  vector<Eigen::Vector3d> traj_pts;
  double tm, tmp;
  bspline.getTimeSpan(tm, tmp);
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }
  displaySphereList(traj_pts, size, color, id1);

  // draw the control point
  if (!show_ctrl_pts) return;

  Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();

  vector<Eigen::Vector3d> ctp;
  for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
    Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
    ctp.push_back(pt);
  }
  displaySphereList(ctp, size2, color2, id2);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_planner_node");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  pos_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/pos", 10);
  vel_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/vel", 10);
  acc_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/acc", 10);

  ros::Duration(1.0).sleep();

  /* ---------- set ctrl points ---------- */
  Eigen::MatrixXd ctrl_pts(8, 3);
  ctrl_pts.row(0) = Eigen::Vector3d(0, 0, 0);
  ctrl_pts.row(1) = Eigen::Vector3d(0.7, 0.9, 0);
  ctrl_pts.row(2) = Eigen::Vector3d(0.9, 2.1, 0);
  ctrl_pts.row(3) = Eigen::Vector3d(2.3, 2.8, 0);
  ctrl_pts.row(4) = Eigen::Vector3d(3.2, 2.8, 0);

  ctrl_pts.row(5) = Eigen::Vector3d(3.7, 1.6, 0);
  ctrl_pts.row(6) = Eigen::Vector3d(5.2, 1.3, 0);
  ctrl_pts.row(7) = Eigen::Vector3d(4.3, 0.6, 0);

  /* ---------- draw pos vel acc ---------- */
  const double ts = 1.5;
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);
  NonUniformBspline vel = pos.getDerivative();
  NonUniformBspline acc = vel.getDerivative();

  pub_id = 0;
  drawBspline(pos, 0.1, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true, 0.12, Eigen::Vector4d(0, 1, 0, 1), 0,
              1);

  pub_id = 1;
  drawBspline(vel, 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1), true, 0.12, Eigen::Vector4d(0, 0, 1, 1), 0,
              1);

  pub_id = 2;
  drawBspline(acc, 0.1, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true, 0.12, Eigen::Vector4d(0, 1, 0, 1), 0,
              1);

  ros::spin();

  return 0;
}
