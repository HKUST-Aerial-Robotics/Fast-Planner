#include "std_msgs/Empty.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <fstream>
#include <stdlib.h>

#include "sensor_msgs/PointCloud.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "grad_traj_optimization/display.h"
#include "grad_traj_optimization/grad_traj_optimizer.h"
#include <grad_traj_optimization/polynomial_traj.hpp>

using namespace std;

ros::Publisher poly_traj_pub;

void displayPathWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  poly_traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0, mk.pose.orientation.y = 0.0, mk.pose.orientation.z = 0.0,
  mk.pose.orientation.w = 1.0;
  mk.scale.x = resolution, mk.scale.y = resolution, mk.scale.z = resolution;

  mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2), mk.color.a = color(3);

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0), pt.y = path[i](1), pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  poly_traj_pub.publish(mk);
  ros::Duration(0.01).sleep();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "random");
  ros::NodeHandle node;

  // -------------------ros initialization---------------------
  poly_traj_pub = node.advertise<visualization_msgs::Marker>("/gradient_based/traj", 10, true);

  srand(ros::Time::now().toSec());
  ros::Duration(0.5).sleep();

  /* ---------- grad traj optimization ---------- */
  GradTrajOptimizer grad_traj_opt;
  //  40 x 40 x 5 map, origin = (-20, -20, 0), resolution = 0.2
  grad_traj_opt.initSDFMap(Eigen::Vector3d(40, 40, 5), Eigen::Vector3d(-40 / 2, -40 / 2, 0.0), 0.2);

  // add obstable
  vector<Eigen::Vector3d> obss;

  for (double x = 0.05; x <= 3.0; x += 0.2)
    for (double y = 2.05; y <= 2.7; y += 0.2)
      for (double z = 0.05; z <= 5.0; z += 0.2) {
        obss.push_back(Eigen::Vector3d(x, y, z));
      }

  for (double x = 0.05; x >= -3.0; x -= 0.2)
    for (double y = -2.05; y >= -2.7; y -= 0.2)
      for (double z = 0.05; z <= 5.0; z += 0.2) {
        obss.push_back(Eigen::Vector3d(x, y, z));
      }

  grad_traj_opt.updateSDFMap(obss);

  // set initial path.
  vector<Eigen::Vector3d> init_path;

  init_path.push_back(Eigen::Vector3d(0, -5, 2));
  init_path.push_back(Eigen::Vector3d(1, -4, 2));
  init_path.push_back(Eigen::Vector3d(1, -3, 2));
  init_path.push_back(Eigen::Vector3d(1, -2, 2));
  init_path.push_back(Eigen::Vector3d(1, -1, 2));

  init_path.push_back(Eigen::Vector3d(0, 0, 2));

  init_path.push_back(Eigen::Vector3d(-1, 1, 2));
  init_path.push_back(Eigen::Vector3d(-1, 2, 2));
  init_path.push_back(Eigen::Vector3d(-1, 3, 2));
  init_path.push_back(Eigen::Vector3d(-1, 4, 2));
  init_path.push_back(Eigen::Vector3d(0, 5, 2));

  grad_traj_opt.setPath(init_path);

  Eigen::MatrixXd coeff;
  Eigen::VectorXd time_sgm;

  grad_traj_opt.optimizeTrajectory(OPT_SECOND_STEP);
  grad_traj_opt.getCoefficient(coeff);
  grad_traj_opt.getSegmentTime(time_sgm);

  /* ---------- convert coefficient to polynomial ---------- */
  PolynomialTraj poly_traj;
  for (int i = 0; i < coeff.rows(); ++i) {
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j) {
      cx[j] = coeff(i, j), cy[j] = coeff(i, j + 6), cz[j] = coeff(i, j + 12);
    }
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = time_sgm(i);
    poly_traj.addSegment(cx, cy, cz, ts);
  }
  poly_traj.init();

  vector<Eigen::Vector3d> traj_vis = poly_traj.getTraj();

  // obstables
  displayPathWithColor(obss, 0.5, Eigen::Vector4d(1, 1, 0, 1), 0);

  // init path
  displayPathWithColor(init_path, 0.3, Eigen::Vector4d(1, 0, 0, 1), 1);

  // optimizer traj
  displayPathWithColor(traj_vis, 0.15, Eigen::Vector4d(0, 0, 1, 1), 2);

  ros::spin();

  return 0;
}
