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



#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

class RayCaster {
private:
  /* data */
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  Eigen::Vector3d direction_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  int x_;
  int y_;
  int z_;
  int endX_;
  int endY_;
  int endZ_;
  double maxDist_;
  double dx_;
  double dy_;
  double dz_;
  int stepX_;
  int stepY_;
  int stepZ_;
  double tMaxX_;
  double tMaxY_;
  double tMaxZ_;
  double tDeltaX_;
  double tDeltaY_;
  double tDeltaZ_;
  double dist_;

  int step_num_;

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_