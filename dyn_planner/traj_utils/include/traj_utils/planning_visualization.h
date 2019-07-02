#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <bspline_opt/non_uniform_bspline.h>

using std::vector;
namespace dyn_planner
{
class PlanningVisualization
{
private:
  enum DRAW_ID
  {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    PREDICTION = 500
  };

  /* data */
  ros::NodeHandle node;
  ros::Publisher traj_pub;

  void displaySphereList(vector<Eigen::Vector3d> list, double resolution, Eigen::Vector4d color, int id);

public:
  PlanningVisualization(/* args */)
  {
  }
  ~PlanningVisualization()
  {
  }

  PlanningVisualization(ros::NodeHandle& nh);

  void drawPath(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id = 0);

  void drawBspline(NonUniformBspline bspline, double size, Eigen::Vector4d color, bool show_ctrl_pts = false,
                   double size2 = 0.1, Eigen::Vector4d color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0, int id2 = 0);

  void drawGoal(Eigen::Vector3d goal, double resolution, Eigen::Vector4d color, int id = 0);

  typedef std::shared_ptr<PlanningVisualization> Ptr;
};
}  // namespace dyn_planner
#endif