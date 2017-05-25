#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();
  double limit;

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  Eigen::MatrixXd Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double limit);

  // Create the model given the current state and actuators input, timesteps and delta t.
  // Return the next state matrix
  Eigen::MatrixXd Model(Eigen::VectorXd state, Eigen::VectorXd actuators, int Num, double dt_);

  // Transform the global map coordinate into vehicle coordinate
  Eigen::MatrixXd Transform(vector<double> ptsx, vector<double> ptsy, Eigen::VectorXd state);


};



#endif /* MPC_H */
