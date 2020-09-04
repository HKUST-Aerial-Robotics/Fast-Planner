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



#ifndef _LINEAR_OBJ_MODEL_H_
#define _LINEAR_OBJ_MODEL_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

class LinearObjModel {
private:
  /* data */
public:
  LinearObjModel(/* args */);
  ~LinearObjModel();

  void initialize(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw, double yaw_dot,
                  Eigen::Vector3d color, Eigen::Vector3d scale);

  void setLimits(Eigen::Vector3d bound, Eigen::Vector2d vel, Eigen::Vector2d acc);

  void update(double dt);  // linear trippler integrator model

  static bool collide(LinearObjModel& obj1, LinearObjModel& obj2);

  // void setInput(Eigen::Vector3d acc) { acc_ = acc; }
  void setInput(Eigen::Vector3d vel) {
    vel_ = vel;
  }

  void setYawDot(double yaw_dot) {
    yaw_dot_ = yaw_dot;
  }

  Eigen::Vector3d getPosition() {
    return pos_;
  }
  void setPosition(Eigen::Vector3d pos) {
    pos_ = pos;
  }

  Eigen::Vector3d getVelocity() {
    return vel_;
  }

  void setVelocity(double x, double y, double z) {
    vel_ = Eigen::Vector3d(x, y, z);
  }

  Eigen::Vector3d getColor() {
    return color_;
  }
  Eigen::Vector3d getScale() {
    return scale_;
  }

  double getYaw() {
    return yaw_;
  }

private:
  Eigen::Vector3d pos_, vel_, acc_;
  Eigen::Vector3d color_, scale_;
  double yaw_, yaw_dot_;

  Eigen::Vector3d bound_;
  Eigen::Vector2d limit_v_, limit_a_;
};

LinearObjModel::LinearObjModel(/* args */) {
}

LinearObjModel::~LinearObjModel() {
}

void LinearObjModel::initialize(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw,
                                double yaw_dot, Eigen::Vector3d color, Eigen::Vector3d scale) {
  pos_ = p;
  vel_ = v;
  acc_ = a;
  color_ = color;
  scale_ = scale;

  yaw_ = yaw;
  yaw_dot_ = yaw_dot;
}

void LinearObjModel::setLimits(Eigen::Vector3d bound, Eigen::Vector2d vel, Eigen::Vector2d acc) {
  bound_ = bound;
  limit_v_ = vel;
  limit_a_ = acc;
}

void LinearObjModel::update(double dt) {
  Eigen::Vector3d p0, v0, a0;
  p0 = pos_, v0 = vel_, a0 = acc_;

  /* ---------- use acc as input ---------- */
  // vel_ = v0 + acc_ * dt;
  // for (int i = 0; i < 3; ++i)
  // {
  //   if (vel_(i) > 0) vel_(i) = std::max(limit_v_(0), std::min(vel_(i),
  //   limit_v_(1)));
  //   if (vel_(i) <= 0) vel_(i) = std::max(-limit_v_(1), std::min(vel_(i),
  //   -limit_v_(0)));
  // }

  // pos_ = p0 + v0 * dt + 0.5 * acc_ * pow(dt, 2);
  // for (int i = 0; i < 2; ++i)
  // {
  //   pos_(i) = std::min(pos_(i), bound_(i));
  //   pos_(i) = std::max(pos_(i), -bound_(i));
  // }
  // pos_(2) = std::min(pos_(2), bound_(2));
  // pos_(2) = std::max(pos_(2), 0.0);

  /* ---------- use vel as input ---------- */
  pos_ = p0 + v0 * dt;
  for (int i = 0; i < 2; ++i) {
    pos_(i) = std::min(pos_(i), bound_(i));
    pos_(i) = std::max(pos_(i), -bound_(i));
  }
  pos_(2) = std::min(pos_(2), bound_(2));
  pos_(2) = std::max(pos_(2), 0.0);

  yaw_ += yaw_dot_ * dt;

  const double PI = 3.1415926;
  if (yaw_ > 2 * PI) yaw_ -= 2 * PI;

  /* ---------- reflect when collide with bound ---------- */
  const double tol = 0.1;
  if (pos_(0) > bound_(0) - tol) {
    pos_(0) = bound_(0) - tol;
    vel_(0) = -vel_(0);
  }
  if (pos_(0) < -bound_(0) + tol) {
    pos_(0) = -bound_(0) + tol;
    vel_(0) = -vel_(0);
  }

  if (pos_(1) > bound_(1) - tol) {
    pos_(1) = bound_(1) - tol;
    vel_(1) = -vel_(1);
  }
  if (pos_(1) < -bound_(1) + tol) {
    pos_(1) = -bound_(1) + tol;
    vel_(1) = -vel_(1);
  }

  if (pos_(2) > bound_(2) - tol) {
    pos_(2) = bound_(2) - tol;
    vel_(2) = -vel_(2);
  }
  if (pos_(2) < tol) {
    pos_(2) = tol;
    vel_(2) = -vel_(2);
  }
}

bool LinearObjModel::collide(LinearObjModel& obj1, LinearObjModel& obj2) {
  Eigen::Vector3d pos1, pos2, vel1, vel2, scale1, scale2;
  pos1 = obj1.getPosition();
  vel1 = obj1.getVelocity();
  scale1 = obj1.getScale();

  pos2 = obj2.getPosition();
  vel2 = obj2.getVelocity();
  scale2 = obj2.getScale();

  /* ---------- collide ---------- */
  bool collide = fabs(pos1(0) - pos2(0)) < 0.5 * (scale1(0) + scale2(0)) &&
      fabs(pos1(1) - pos2(1)) < 0.5 * (scale1(1) + scale2(1)) &&
      fabs(pos1(2) - pos2(2)) < 0.5 * (scale1(2) + scale2(2));

  if (collide) {
    double tol[3];
    tol[0] = 0.5 * (scale1(0) + scale2(0)) - fabs(pos1(0) - pos2(0));
    tol[1] = 0.5 * (scale1(1) + scale2(1)) - fabs(pos1(1) - pos2(1));
    tol[2] = 0.5 * (scale1(2) + scale2(2)) - fabs(pos1(2) - pos2(2));

    for (int i = 0; i < 3; ++i) {
      if (tol[i] < tol[(i + 1) % 3] && tol[i] < tol[(i + 2) % 3]) {
        vel1(i) = -vel1(i);
        vel2(i) = -vel2(i);
        obj1.setVelocity(vel1(0), vel1(1), vel1(2));
        obj2.setVelocity(vel2(0), vel2(1), vel2(2));

        if (pos1(i) >= pos2(i)) {
          pos1(i) += tol[i];
          pos2(i) -= tol[i];
        } else {
          pos1(i) -= tol[i];
          pos2(i) += tol[i];
        }
        obj1.setPosition(pos1);
        obj2.setPosition(pos2);

        break;
      }
    }

    return true;
  } else {
    return false;
  }
}

#endif