#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "Eigen-3.3/Eigen/Core"

using namespace std;
using CppAD::AD;

class MPC {
 public:
  MPC();

  virtual ~MPC();
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  void GetPredictions(vector<double>& mpc_x_vals, vector<double>& mpc_y_vals);

};

#endif /* MPC_H */
