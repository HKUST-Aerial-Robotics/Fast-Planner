#include "quadrotor_simulator/Quadrotor.h"
#include <Eigen/Geometry>

int main(int argc, char **argv)
{
  double dt = 0.001;
  QuadrotorSimulator::Quadrotor quad;
  QuadrotorSimulator::Quadrotor::State state = quad.getState();

  const double m = quad.getMass();
  const double g = quad.getGravity();
  const double kf = quad.getPropellerThrustCoefficient();

  const double hover_rpm = std::sqrt(m*g/(4*kf));
  std::cerr << "hover rpm: " << hover_rpm << std::endl;
  state.motor_rpm = Eigen::Array4d(hover_rpm, hover_rpm, hover_rpm, hover_rpm);
  quad.setState(state);

  double thrust = m*g;
  double rpm = std::sqrt(thrust/(4*kf));
  quad.setInput(rpm, rpm, rpm, rpm);

  struct timespec ts_start, ts1, ts2, ts_sleep, ts_end;
  clock_gettime(CLOCK_MONOTONIC, &ts1);
  int64_t time_taken = 0;

  double KP = 8.0;
  double KD = 2.5;
  const double z_des = 0.5;
  clock_gettime(CLOCK_MONOTONIC, &ts_start);
  for(int i = 0; i < 6000; i++)
  {
    state = quad.getState();
    thrust = m*g + KP*(z_des - state.x(2)) + KD*(0 - state.v(2));
    rpm = std::sqrt(thrust/(4*kf));
    if( i < 3000)
      quad.setExternalForce(Eigen::Vector3d(0, 0, -KP*z_des));
    else
      quad.setExternalForce(Eigen::Vector3d(0, 0, 0));
    quad.setInput(rpm, rpm, rpm, rpm);
    quad.step(dt);
    Eigen::Vector3d euler = state.R.eulerAngles(2,1,0);
    std::cout << i*dt << ", " << state.x(2) << ", " << euler(0) << ", " << euler(1) << ", " << euler(2) << ", " <<
        state.omega(0) << ", " << state.omega(1) << ", " << state.omega(2) << ", " << state.motor_rpm(0) << std::endl;

    clock_gettime(CLOCK_MONOTONIC, &ts2);
    time_taken += ((ts2.tv_sec-ts1.tv_sec)*1000000000UL + (ts2.tv_nsec-ts1.tv_nsec));
    int64_t time_sleep = i*dt/2*1e9 - time_taken;
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    if(time_sleep > 0)
    {
      ts_sleep.tv_sec = time_sleep/1000000000UL;
      ts_sleep.tv_nsec = time_sleep - ts_sleep.tv_sec*1000000000UL;
      //nanosleep(&ts_sleep, NULL);
    }
  }
  clock_gettime(CLOCK_MONOTONIC, &ts_end);
  std::cerr << "Time: " << (ts_end.tv_sec-ts_start.tv_sec)*1e6 + (ts_end.tv_nsec-ts_start.tv_nsec)/1e3 << " usec" <<std::endl;

  return 0;
}
