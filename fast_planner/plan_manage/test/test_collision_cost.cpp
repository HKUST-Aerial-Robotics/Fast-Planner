#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/non_uniform_bspline.h>
#include <traj_utils/planning_visualization.h>

using namespace std;
using namespace fast_planner;

int succ_replan_num_ = 0;

void buildEnv(pcl::PointCloud<pcl::PointXYZ>& map) {
  const double resolution = 0.05;
  pcl::PointXYZ pt;

  /* main barrier */
  for (double x = -0.5; x <= 0.5; x += resolution)
    for (double y = -1.5; y <= 1.5; y += resolution)
      for (double z = -0.0; z <= 2.0; z += resolution) {
        pt.x = x + 1e-2;
        pt.y = y + 1e-2;
        pt.z = z + 1e-2;
        map.push_back(pt);
      }

  /* surrounding barriers */
  vector<Eigen::Vector3d> centers;
  centers.push_back(Eigen::Vector3d(2.5, -0.5, 0.0));
  centers.push_back(Eigen::Vector3d(+2.5, 2.3, 0.0));
  centers.push_back(Eigen::Vector3d(-2.5, 2.3, 0.0));
  centers.push_back(Eigen::Vector3d(0.0, 3.5, 0.0));

  for (int i = 0; i < centers.size(); ++i) {
    for (double x = -0.27; x <= 0.27; x += resolution)
      for (double y = -0.3; y <= 0.3; y += resolution)
        for (double z = 0; z <= 2.0; z += resolution) {
          pt.x = x + centers[i](0) + 1e-2;
          pt.y = y + centers[i](1) + 1e-2;
          pt.z = z + centers[i](2) + 1e-2;
          map.push_back(pt);
        }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "replan_node");
  ros::NodeHandle node("~");
  ros::Time t1, t2;

  node.param("bspline/limit_vel", NonUniformBspline::limit_vel_, -1.0);
  node.param("bspline/limit_acc", NonUniformBspline::limit_acc_, -1.0);
  node.param("bspline/limit_ratio", NonUniformBspline::limit_ratio_, -1.0);

  ros::Publisher map_pub = node.advertise<sensor_msgs::PointCloud2>("/test_collision/map", 10, true);

  PlanningVisualization::Ptr visualizer;
  visualizer.reset(new PlanningVisualization(node));
  // return 0;

  SDFMap::Ptr sdf_map;
  sdf_map.reset(new SDFMap);
  sdf_map->init(node);

  EDTEnvironment::Ptr env;
  env.reset(new EDTEnvironment);
  env->setMap(sdf_map);

  BsplineOptimizer::Ptr bspline_opt;
  bspline_opt.reset(new BsplineOptimizer);
  bspline_opt->setParam(node);
  bspline_opt->setEnvironment(env);

  /* ---------- build obstacles ---------- */
  pcl::PointCloud<pcl::PointXYZ> cloud_map;
  buildEnv(cloud_map);

  /* create SDF map of the obstacles */
  for (size_t i = 0; i < cloud_map.points.size(); ++i) {
    sdf_map->setOccupancy(
        Eigen::Vector3d(cloud_map.points[i].x, cloud_map.points[i].y, cloud_map.points[i].z));
  }
  sdf_map->updateESDF3d(true);

  /* ---------- generate naive straight line trajectory ---------- */
  /* p(t) = p0 + v0 * t, v0 = 2m/s, t \in [0, 3], delta_t = 0.2 s */
  Eigen::Vector3d p0(-5.1, 0.05, 1), v1(2, 0.1, 0), v2(2, -0.1, 0), p_t;

  double dt = 0.1;  // used for parameterization
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  for (double t = 0.0; t <= 5.0 + 1e-4; t += dt) {
    if (t <= 2.5)
      p_t = p0 + v1 * t;
    else
      p_t = p0 + v1 * 2.5 + v2 * (t - 2.5);
    point_set.push_back(p_t);
  }
  start_end_derivative.push_back(v1);
  start_end_derivative.push_back(v2);
  start_end_derivative.push_back(Eigen::Vector3d(0, 0, 0));
  start_end_derivative.push_back(Eigen::Vector3d(0, 0, 0));

  /* parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  cout << "ctrl pts:" << ctrl_pts.rows() << endl;
  /* visualize the B-spline before optimization */
  NonUniformBspline traj_init = NonUniformBspline(ctrl_pts, 3, dt);

  /* ---------- optimization ---------- */
  bspline_opt->setControlPoints(ctrl_pts);
  bspline_opt->setBSplineInterval(dt);

  t1 = ros::Time::now();

  bspline_opt->optimize(BsplineOptimizer::HARD_CONSTRAINT, false);

  t2 = ros::Time::now();
  cout << "optimization time: " << (t2 - t1).toSec() << endl;

  ctrl_pts = bspline_opt->getControlPoints();
  NonUniformBspline traj_opt = NonUniformBspline(ctrl_pts, 3, dt);

  // /* time adjustment */
  // int iter_num = 0;
  // bool feasible = traj_opt.checkFeasibility();
  // while (!feasible && ros::ok())
  // {
  //   ++iter_num;
  //   feasible = traj_opt.adjustTime();
  //   /* actually this not needed, converges within 10 iteration */
  //   if (iter_num >= 50)
  //     break;
  // }
  // cout << "iter num: " << iter_num << endl;

  /* ---------- everything done. visualization ---------- */
  ros::Duration(0.5).sleep();
  cloud_map.width = cloud_map.points.size();
  cloud_map.height = 1;
  cloud_map.is_dense = true;
  cloud_map.header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud_vis;
  pcl::toROSMsg(cloud_map, cloud_vis);
  map_pub.publish(cloud_vis);

  // visualizer->drawBspline(traj_init, 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1),
  // true, 0.12, Eigen::Vector4d(1,
  // 1, 0, 1),
  // 0,
  //                         0);

  visualizer->drawBspline(traj_opt, 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1), true, 0.12,
                          Eigen::Vector4d(0, 0, 1, 1), 1, 1);

  ros::spin();

  return 1;
}