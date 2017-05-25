#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <math.h>
#include <vector>


using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 25;
double dt = 0.02;


// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;


// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 40 ;

// The solver takes all the state variables and actuator
// variables in a singular vector. 

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;



class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  // Speed Limit
  double limit;

  FG_eval(Eigen::VectorXd coeffs, double limit) { this->coeffs = coeffs; this->limit = limit; }


  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg is a vector of constraints, x is a vector of constraints.

    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int i = 0; i < N; i++) {
      fg[0] += 1.2*CppAD::pow(vars[cte_start + i] - ref_cte, 2);
      //std::cout << "cte cost " << fg[0] << std::endl;
      fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      //std::cout << "epsi cost " << fg[0] << std::endl;
      //fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
      fg[0] += CppAD::pow(vars[v_start + i] - limit, 2);
      //std::cout << "speed cost " << limit << std::endl;
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += 0.00021*vars[v_start +i]*CppAD::pow(vars[delta_start + i], 2);
      //fg[0] += 0.04*CppAD::pow(vars[delta_start + i]*vars[v_start + i], 2);
      //fg[0] += 1000*CppAD::pow(vars[delta_start + i], 2);
      //std::cout << "steering cost " << fg[0] << std::endl;      
      fg[0] += CppAD::pow(vars[a_start + i], 2);
      //std::cout << "Throttle cost " << fg[0] << std::endl;
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      fg[0] += 20000*vars[v_start + i]*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      //fg[0] += 1000*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      //std::cout << "steering rate cost " << fg[0] << std::endl;
      fg[0] += 100*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
      //std::cout << "Throttle rate cost " << fg[0] << std::endl;
    }


    
    // Initial constraints
    //
    // Cost being located at index 0 of `fg`.
    // Constraints start at 1 position
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int i = 0; i < N - 1; i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2]* x0*x0 + coeffs[3]*x0*x0*x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3* coeffs[3]*x0*x0);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

Eigen::MatrixXd MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double limit) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];


  // TODO: Set the number of model variables (includes both states and inputs).

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;
  
  // TODO: Set the number of constraints
  //
  // Number of constraints
  size_t n_constraints = N * 6;


  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -0.25 and 0.25
  // (values in radians).
  
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.5;
    vars_upperbound[i] = 0.5;
  }

  // Acceleration/decceleration upper and lower limits.
  
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, limit);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  
  // Create result vector
  // std::vector<double> result;
  Eigen::MatrixXd result(4, N); 

  for (auto i = 0; i < N-1 ; i++){

    result(0,i) = solution.x[delta_start+i];
    result(1,i) = solution.x[a_start+i];
    result(2,i) = solution.x[x_start+i];
    result(3,i) = solution.x[y_start+i];
    
    //std::cout << i << ": " << "solution.x[delta_start+i]: " << 180*solution.x[delta_start+i]/3.14 << std::endl;
  }
  
  
  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  


   return result;
}


// Create the global Kinematic model, take current state and actuators as input, return the next state after delta t. 
Eigen::MatrixXd MPC::Model(Eigen::VectorXd state, Eigen::VectorXd actuators, int Num, double dt_) {

  // Create a new Matrix to store all new states.
  Eigen::MatrixXd next_state(state.size(), Num);

  // for vehicle coordinate, only take the speed input from state, x,y,psi all 0.0. 
  double x = 0.0;
  double y = 0.0;
  double psi = 0.0;
  double v = state(3);  


  double s_ = actuators(0);
  double a_ = actuators(1);

  // Recall the equations for the model:
  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  // v_[t+1] = v[t] + a[t] * dt

  for (int i=0; i<Num; i++){
    x += v * cos(psi) * dt_;
    y += v * sin(psi) * dt_;
    psi += (v / Lf) * s_ * dt_;
    v += a_ * dt_;
    next_state(0,i) = x;
    next_state(1,i) = y;
    next_state(2,i) = psi;
    next_state(3,i) = v;
  }  

  return next_state;

}


// Transform Function
// Translate and Rotate the global map coordinate to vehicle coordinate
// Take input: way points, car position, rotation
// Return: transformed way point, vehicle view

Eigen::MatrixXd MPC::Transform(vector<double> ptsx, vector<double> ptsy, Eigen::VectorXd state){

  // For easy reading
  auto px = state(0);
  auto py = state(1);
  auto psi = state(2);

  // The translation matrix from car location to (0,0)
  Eigen::MatrixXd T(3, 3);
  T << 1, 0, -px,
       0, 1, -py,
       0, 0,  1;
  //std::cout << T << std::endl;
          
  // The Rotation matrix from global coordinate to vehicle coordinate
  Eigen::MatrixXd R(3, 3);
  R << cos(-psi), -sin(-psi), 	0,
       sin(-psi), cos(-psi), 	0,
       0,   	  0, 		1;

  // R << cos(psi), sin(psi), 	0,
     //  -sin(psi), cos(psi), 	0,
       //0,   	  0, 		1;
  //std::cout << R << std::endl;
  
  // Create way point coordinate Matrix
  Eigen::MatrixXd wps(3, ptsx.size());

  for (int i=0; i< ptsx.size(); i++){
    wps(0,i) = ptsx[i];
    wps(1,i) = ptsy[i];
    wps(2,i) = 1;
  }
  //std::cout <<"way points in Global Map Coordinate:"<< std::endl;
  //std::cout<< wps << std::endl;
          
  // Create vehicle coordinate Matrix
  Eigen:: MatrixXd car(3, ptsx.size());
  car = R * T * wps;
  //std::cout << "Transformed vehicle coordinate Matrix:" << std::endl;
  //std::cout << car << std::endl;
  
  return car;



} 


