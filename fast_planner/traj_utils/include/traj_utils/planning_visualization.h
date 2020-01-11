#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline_opt/non_uniform_bspline.h>
#include <iostream>
#include <poly_traj/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>

using std::vector;
namespace fast_planner {
class PlanningVisualization {
private:
  enum TRAJECTORY_PLANNING_ID {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    POLY_TRAJ = 500
  };

  enum TOPOLOGICAL_PATH_PLANNING_ID {
    GRAPH_NODE = 1,
    GRAPH_EDGE = 100,
    RAW_PATH = 200,
    FILTERED_PATH = 300,
    SELECT_PATH = 400
  };

  /* data */
  /* visib_pub is seperated from previous ones for different info */
  ros::NodeHandle node;
  ros::Publisher traj_pub_;      // 0

  vector<ros::Publisher> pubs_;  //

  int last_bspline_phase1_num_;
  int last_bspline_phase2_num_;
  int last_frontier_num_;

public:
  PlanningVisualization(/* args */) {
  }
  ~PlanningVisualization() {
  }
  PlanningVisualization(ros::NodeHandle& nh);

  // draw basic shapes
  void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution, Eigen::Vector4d color,
                         int id, int pub_id = 0);
  void displayCubeList(const vector<Eigen::Vector3d>& list, double resolution, Eigen::Vector4d color,
                       int id, int pub_id = 0);
  void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                       double line_width, Eigen::Vector4d color, int id, int pub_id = 0);

  // draw a piece-wise straight line path
  void drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution, Eigen::Vector4d color,
                         int id = 0);

  // draw a polynomial trajectory
  void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, Eigen::Vector4d color,
                          int id = 0);

  // draw a bspline trajectory
  void drawBspline(NonUniformBspline& bspline, double size, Eigen::Vector4d color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   Eigen::Vector4d color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0, int id2 = 0);

  // draw a set of bspline trajectories generated in different phases
  void drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size);
  void drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size);

  // draw topological graph and paths

  void drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>>& paths, double line_width);
  void drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths, double line_width);

  void drawGoal(Eigen::Vector3d goal, double resolution, Eigen::Vector4d color, int id = 0);

  Eigen::Vector4d getColor(double h, double alpha = 1.0);

  typedef std::shared_ptr<PlanningVisualization> Ptr;
};
}  // namespace fast_planner
#endif