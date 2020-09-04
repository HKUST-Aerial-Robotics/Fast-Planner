/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



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
