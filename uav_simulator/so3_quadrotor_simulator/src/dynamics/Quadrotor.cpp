#include "quadrotor_simulator/Quadrotor.h"
#include "ode/boost/numeric/odeint.hpp"
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <iostream>

#include <ros/ros.h>
namespace odeint = boost::numeric::odeint;

namespace QuadrotorSimulator
{

Quadrotor::Quadrotor(void)
{
  alpha0     = 48; // degree
  g_         = 9.81;
  mass_      = 0.98; // 0.5;
  double Ixx = 2.64e-3, Iyy = 2.64e-3, Izz = 4.96e-3;
  prop_radius_ = 0.062;
  J_           = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();

  kf_ = 8.98132e-9;
  // km_ = 2.5e-9; // from Nate
  // km = (Cq/Ct)*Dia*kf
  // Cq/Ct for 8 inch props from UIUC prop db ~ 0.07
  km_ = 0.07 * (3 * prop_radius_) * kf_;

  arm_length_          = 0.26;
  motor_time_constant_ = 1.0 / 30;
  min_rpm_             = 1200;
  max_rpm_             = 35000;

  state_.x = Eigen::Vector3d::Zero();
  // state_.x << 40.0, -60.0, 10.0;
  state_.v         = Eigen::Vector3d::Zero();
  state_.R         = Eigen::Matrix3d::Identity();
  state_.omega     = Eigen::Vector3d::Zero();
  state_.motor_rpm = Eigen::Array4d::Zero();

  external_force_.setZero();

  updateInternalState();

  input_ = Eigen::Array4d::Zero();
}

void
Quadrotor::step(double dt)
{
  auto save = internal_state_;

  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

  for (int i = 0; i < 22; ++i)
  {
    if (std::isnan(internal_state_[i]))
    {
      std::cout << "dump " << i << " << pos ";
      for (int j = 0; j < 22; ++j)
      {
        std::cout << save[j] << " ";
      }
      std::cout << std::endl;
      internal_state_ = save;
      break;
    }
  }

  for (int i = 0; i < 3; i++)
  {
    state_.x(i) = internal_state_[0 + i];
    state_.v(i) = internal_state_[3 + i];
    state_.R(i, 0) = internal_state_[6 + i];
    state_.R(i, 1) = internal_state_[9 + i];
    state_.R(i, 2) = internal_state_[12 + i];
    state_.omega(i) = internal_state_[15 + i];
  }
  state_.motor_rpm(0) = internal_state_[18];
  state_.motor_rpm(1) = internal_state_[19];
  state_.motor_rpm(2) = internal_state_[20];
  state_.motor_rpm(3) = internal_state_[21];

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(state_.R.transpose() * state_.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = state_.R * P.inverse();
  state_.R                      = R;

  // Don't go below zero, simulate floor
  if (state_.x(2) < 0.0 && state_.v(2) < 0)
  {
    state_.x(2) = 0;
    state_.v(2) = 0;
  }
  updateInternalState();
}

void
Quadrotor::operator()(const Quadrotor::InternalState& x,
                      Quadrotor::InternalState& dxdt, const double /* t */)
{
  State cur_state;
  for (int i = 0; i < 3; i++)
  {
    cur_state.x(i) = x[0 + i];
    cur_state.v(i) = x[3 + i];
    cur_state.R(i, 0) = x[6 + i];
    cur_state.R(i, 1) = x[9 + i];
    cur_state.R(i, 2) = x[12 + i];
    cur_state.omega(i) = x[15 + i];
  }
  for (int i = 0; i < 4; i++)
  {
    cur_state.motor_rpm(i) = x[18 + i];
  }

  // std::cout << "Omega: " << cur_state.omega << std::endl;
  // std::cout << "motor_rpm: " << cur_state.motor_rpm << std::endl;

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(cur_state.R.transpose() * cur_state.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = cur_state.R * P.inverse();

  Eigen::Vector3d x_dot, v_dot, omega_dot;
  Eigen::Matrix3d R_dot;
  Eigen::Array4d  motor_rpm_dot;
  Eigen::Vector3d vnorm;
  Eigen::Array4d  motor_rpm_sq;
  Eigen::Matrix3d omega_vee(Eigen::Matrix3d::Zero());

  omega_vee(2, 1) = cur_state.omega(0);
  omega_vee(1, 2) = -cur_state.omega(0);
  omega_vee(0, 2) = cur_state.omega(1);
  omega_vee(2, 0) = -cur_state.omega(1);
  omega_vee(1, 0) = cur_state.omega(2);
  omega_vee(0, 1) = -cur_state.omega(2);

  motor_rpm_sq = cur_state.motor_rpm.array().square();

  //! @todo implement
  Eigen::Array4d blade_linear_velocity;
  Eigen::Array4d motor_linear_velocity;
  Eigen::Array4d AOA;
  blade_linear_velocity = 0.104719755 // rpm to rad/s
                          * cur_state.motor_rpm.array() * prop_radius_;
  for (int i = 0; i < 4; ++i)
    AOA[i]   = alpha0 -
             atan2(motor_linear_velocity[i], blade_linear_velocity[i]) * //
               180 / 3.14159265;
  //! @todo end

  // double totalF = kf_ * motor_rpm_sq.sum();
  double thrust = kf_ * motor_rpm_sq.sum();

  Eigen::Vector3d moments;
  moments(0) = kf_ * (motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length_;
  moments(1) = kf_ * (motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length_;
  moments(2) = km_ * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) -
                      motor_rpm_sq(3));

  double resistance = 0.1 *                                        // C
                      3.14159265 * (arm_length_) * (arm_length_) * // S
                      cur_state.v.norm() * cur_state.v.norm();

  //  ROS_INFO("resistance: %lf, Thrust: %lf%% ", resistance,
  //           motor_rpm_sq.sum() / (4 * max_rpm_ * max_rpm_) * 100.0);

  vnorm = cur_state.v;
  if (vnorm.norm() != 0)
  {
    vnorm.normalize();
  }
  x_dot = cur_state.v;
  v_dot = -Eigen::Vector3d(0, 0, g_) + thrust * R.col(2) / mass_ +
          external_force_ / mass_ /*; //*/ - resistance * vnorm / mass_;

  acc_ = v_dot;
  //  acc_[2] = -acc_[2]; // to NED

  R_dot = R * omega_vee;
  omega_dot =
    J_.inverse() *
    (moments - cur_state.omega.cross(J_ * cur_state.omega) + external_moment_);
  motor_rpm_dot = (input_ - cur_state.motor_rpm) / motor_time_constant_;

  for (int i = 0; i < 3; i++)
  {
    dxdt[0 + i]  = x_dot(i);
    dxdt[3 + i]  = v_dot(i);
    dxdt[6 + i]  = R_dot(i, 0);
    dxdt[9 + i]  = R_dot(i, 1);
    dxdt[12 + i] = R_dot(i, 2);
    dxdt[15 + i] = omega_dot(i);
  }
  for (int i = 0; i < 4; i++)
  {
    dxdt[18 + i] = motor_rpm_dot(i);
  }
  for (int i = 0; i < 22; ++i)
  {
    if (std::isnan(dxdt[i]))
    {
      dxdt[i] = 0;
      //      std::cout << "nan apply to 0 for " << i << std::endl;
    }
  }
}

void
Quadrotor::setInput(double u1, double u2, double u3, double u4)
{
  input_(0) = u1;
  input_(1) = u2;
  input_(2) = u3;
  input_(3) = u4;
  for (int i = 0; i < 4; i++)
  {
    if (std::isnan(input_(i)))
    {
      input_(i) = (max_rpm_ + min_rpm_) / 2;
      std::cout << "NAN input ";
    }
    if (input_(i) > max_rpm_)
      input_(i) = max_rpm_;
    else if (input_(i) < min_rpm_)
      input_(i) = min_rpm_;
  }
}

const Quadrotor::State&
Quadrotor::getState(void) const
{
  return state_;
}
void
Quadrotor::setState(const Quadrotor::State& state)
{
  state_.x         = state.x;
  state_.v         = state.v;
  state_.R         = state.R;
  state_.omega     = state.omega;
  state_.motor_rpm = state.motor_rpm;

  updateInternalState();
}

void
Quadrotor::setStatePos(const Eigen::Vector3d& Pos)
{
  state_.x = Pos;

  updateInternalState();
}

double
Quadrotor::getMass(void) const
{
  return mass_;
}
void
Quadrotor::setMass(double mass)
{
  mass_ = mass;
}

double
Quadrotor::getGravity(void) const
{
  return g_;
}
void
Quadrotor::setGravity(double g)
{
  g_ = g;
}

const Eigen::Matrix3d&
Quadrotor::getInertia(void) const
{
  return J_;
}
void
Quadrotor::setInertia(const Eigen::Matrix3d& inertia)
{
  if (inertia != inertia.transpose())
  {
    std::cerr << "Inertia matrix not symmetric, not setting" << std::endl;
    return;
  }
  J_ = inertia;
}

double
Quadrotor::getArmLength(void) const
{
  return arm_length_;
}
void
Quadrotor::setArmLength(double d)
{
  if (d <= 0)
  {
    std::cerr << "Arm length <= 0, not setting" << std::endl;
    return;
  }

  arm_length_ = d;
}

double
Quadrotor::getPropRadius(void) const
{
  return prop_radius_;
}
void
Quadrotor::setPropRadius(double r)
{
  if (r <= 0)
  {
    std::cerr << "Prop radius <= 0, not setting" << std::endl;
    return;
  }
  prop_radius_ = r;
}

double
Quadrotor::getPropellerThrustCoefficient(void) const
{
  return kf_;
}
void
Quadrotor::setPropellerThrustCoefficient(double kf)
{
  if (kf <= 0)
  {
    std::cerr << "Thrust coefficient <= 0, not setting" << std::endl;
    return;
  }

  kf_ = kf;
}

double
Quadrotor::getPropellerMomentCoefficient(void) const
{
  return km_;
}
void
Quadrotor::setPropellerMomentCoefficient(double km)
{
  if (km <= 0)
  {
    std::cerr << "Moment coefficient <= 0, not setting" << std::endl;
    return;
  }

  km_ = km;
}

double
Quadrotor::getMotorTimeConstant(void) const
{
  return motor_time_constant_;
}
void
Quadrotor::setMotorTimeConstant(double k)
{
  if (k <= 0)
  {
    std::cerr << "Motor time constant <= 0, not setting" << std::endl;
    return;
  }

  motor_time_constant_ = k;
}

const Eigen::Vector3d&
Quadrotor::getExternalForce(void) const
{
  return external_force_;
}
void
Quadrotor::setExternalForce(const Eigen::Vector3d& force)
{
  external_force_ = force;
}

const Eigen::Vector3d&
Quadrotor::getExternalMoment(void) const
{
  return external_moment_;
}
void
Quadrotor::setExternalMoment(const Eigen::Vector3d& moment)
{
  external_moment_ = moment;
}

double
Quadrotor::getMaxRPM(void) const
{
  return max_rpm_;
}
void
Quadrotor::setMaxRPM(double max_rpm)
{
  if (max_rpm <= 0)
  {
    std::cerr << "Max rpm <= 0, not setting" << std::endl;
    return;
  }
  max_rpm_ = max_rpm;
}

double
Quadrotor::getMinRPM(void) const
{
  return min_rpm_;
}
void
Quadrotor::setMinRPM(double min_rpm)
{
  if (min_rpm < 0)
  {
    std::cerr << "Min rpm < 0, not setting" << std::endl;
    return;
  }
  min_rpm_ = min_rpm;
}

void
Quadrotor::updateInternalState(void)
{
  for (int i = 0; i < 3; i++)
  {
    internal_state_[0 + i]  = state_.x(i);
    internal_state_[3 + i]  = state_.v(i);
    internal_state_[6 + i]  = state_.R(i, 0);
    internal_state_[9 + i]  = state_.R(i, 1);
    internal_state_[12 + i] = state_.R(i, 2);
    internal_state_[15 + i] = state_.omega(i);
  }
  internal_state_[18] = state_.motor_rpm(0);
  internal_state_[19] = state_.motor_rpm(1);
  internal_state_[20] = state_.motor_rpm(2);
  internal_state_[21] = state_.motor_rpm(3);
}

Eigen::Vector3d
Quadrotor::getAcc() const
{
  return acc_;
}
}
