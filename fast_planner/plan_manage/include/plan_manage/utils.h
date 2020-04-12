#include <Eigen/Eigen>

#include <vector>

Eigen::Vector3d getFarPoint(const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d x1,
                                                Eigen::Vector3d x2) {
  double max_dist = -1000;
  Eigen::Vector3d vl = (x2 - x1).normalized();
  Eigen::Vector3d far_pt;

  for (int i = 1; i < path.size() - 1; ++i) {
    double dist = ((path[i] - x1).cross(vl)).norm();
    if (dist > max_dist) {
      max_dist = dist;
      far_pt = path[i];
    }
  }

  return far_pt;
}
