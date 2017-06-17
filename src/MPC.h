#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

class MPC {
 private:
  double ref_v_;        // Target velocity
  int delay_;           // Delay timesteps to simulate latency

  double throttle_ = 0; // Last computed throttle
  double steer_ = 0;    // Last computed steering angle
 public:
  MPC(double target_velocity, int delay_steps);

  virtual ~MPC();
  // Variables to store predicted trajectory
  std::vector<double> x_predict;
  std::vector<double> y_predict;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
