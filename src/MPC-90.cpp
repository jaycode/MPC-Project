#include "MPC-90.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration

// At speed 40 and dt 0.15:
// Setting N to 20 causes the car to turn aggresively.
// Setting N to 5 causes the car to turn slowly. It scratches the sidewalk at times.
// Setting N to 7 causes the car to turn a bit more aggresively until it got stuck at a sidewalk.
size_t N = 5;
double dt = 0.15;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_cte = 0;
double ref_epsi = 0;
double max_ref_v = 300;
double min_ref_v = 70;
double ref_v = max_ref_v;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

double max_sharpness = 2.7;
double min_curverad = 9e+10;
double max_curverad = 0;

// Curve radius of a third-degree polynomial given x (x being some distance in front of the car).
// http://www.intmath.com/applications-differentiation/8-radius-curvature.php
double calc_curverad3(Eigen::VectorXd coeffs, double x) {
  double x2 = x * x;
  double dy = x2 * coeffs[0] * 3 + x * coeffs[1] * 2 + coeffs[2];
  double dy2 = dy * dy;
  double d2y = x * 2 * coeffs[0] * 3 + coeffs[1] * 2;
  return pow(1 + dy2, (3.0/2.0)) / fabs(d2y);
}

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;

    // Curveness of a polynomial depends upon its first coefficient.
    // Higher means sharper.
    // https://www.mathsisfun.com/algebra/polynomials-behave.html
    AD<double> sharpness = max(0.01, fabs(coeffs[0]));
    AD<double> sharpness_norm = sharpness / max_sharpness;

    // std::cout << "sharpness1: " << sharpness << " (" << sharpness_norm << ")" << std::endl;
    // std::cout << coeffs[0] << " : " << coeffs[1] << " : " << coeffs[2] << " : " << coeffs[3] << " : " << std::endl;


    // AD<double> curverad = calc_curverad3(coeffs, 10.0);
    // AD<double> curverad_norm = fabs(curverad - min_curverad) / fabs(max_curverad - min_curverad);
    // std::cout << "curverad: " << curverad << std::endl;
    // std::cout << " (" << curverad_norm << 
    // ") min " << min_curverad << " max " << max_curverad << std::endl;

    // std::cout << calc_curverad3(coeffs, 1.0) << " : " << calc_curverad3(coeffs, 5.0)
    //   << " : " << calc_curverad3(coeffs, 10.0) << " : " << calc_curverad3(coeffs, 15.0)
    //   << " : " << calc_curverad3(coeffs, 20.0) << " : " << calc_curverad3(coeffs, 25.0) << std::endl;

    // std::cout << vars[cte_start] << " : " << vars[cte_start + 1] << " : "
      // << vars[cte_start + 2] << " : " << vars[cte_start + 3] << " : "
      // << vars[cte_start + 4] << " : " << vars[cte_start + 5] << std::endl;

    // The part of the cost based on the reference state.
    for (int i = 0; i < N; i++) {
      fg[0] += 2 * CppAD::pow(vars[v_start + i] - ref_v, 2);

      // The higher this is, the closer the green line to to yellow line.
      fg[0] += 0.80 * CppAD::pow(vars[cte_start + i] - ref_cte, 2);

      // Make the epsilon error more important i.e. preferring the same angle of yellow
      // and green lines. The default value sometimes produces erratic (green) lines.
      fg[0] += 100 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += CppAD::pow(vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i], 2);
    }


    // Minimize the value gap between sequential actuations.
    // On sharper turns, we want the car to turn faster.
    // From Udacity Lesson on Tuning MPC, high value times
    // constraint results in smoother turns.
    // So in here, the sharper the curveness, the smaller
    // its constraint, thus smaller the multiplying factor.
    // The higher this factor, the less likely car oscillates, but harder to turn.
    AD<double> delta_factor;

    // This formula works with ref_v of 30, but not any higher. It stopped working after
    // running for a while.
    // delta_factor = ((1-sharpness_norm) * 1000);
    // This conditional works as well with ref_v of 30, but the turning was so sudden
    // if (sharpness_norm > 0.5) {
    //   delta_factor = (1-sharpness_norm) / 10;
    // }
    // else {
    //   delta_factor = ((1-sharpness_norm) * 1000);
    // }

    AD<double> v_diff = fabs(ref_v - vars[v_start]);
    AD<double> v_diff_norm = v_diff / ref_v;
    delta_factor = fabs((v_diff_norm * 0) + (1 - sharpness_norm) * 200);
    // std::cout << "delta_factor: " << delta_factor << std::endl;

    for (int i = 0; i < N - 2; i++) {
      fg[0] += delta_factor * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    // for (int i = 0; i < N - 3; i++) {
    //   double delta_factor;
    //   delta_factor = ((1-sharpness_norm) * 3000);

    //   fg[0] += delta_factor * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
    //   fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    // }


    //
    // Setup Constraints
    //

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
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

      AD<double> psides0 = CppAD::atan(coeffs[1]);

      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // The cte equation above works only for a single linear line.

      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);

      // This is important, without this the green line will go
      // the opposite direction to the yellow line.
      // The reasoning behind this is because what an optimizer needs
      // is a rate of change between the last known value (cte1 in
      // this case) with the predicted value ((pred_cte1-y1) here).
      AD<double> pred_cte1 = coeffs[0] + coeffs[1] * x1 + 
        coeffs[2] * x1 * x1 + coeffs[3] * x1 * x1 * x1;
      fg[2 + cte_start + i] = cte1 - (pred_cte1 - y1);

      fg[2 + epsi_start + i] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  size_t n_vars = (N * state.size()) + ((N - 1) * 2);
  // Set the number of constraints
  size_t n_constraints = N * state.size();

  if (epsi > M_PI) epsi -= 2*M_PI;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Lower and upper limits for variables
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  float sharpness = max(0.01, fabs(coeffs[0]));
  // if (sharpness > max_sharpness) {
  //   max_sharpness = sharpness;
  // }

  float sharpness_norm = sharpness/max_sharpness;

  // std::cout << "sharpness2: " << sharpness << " (" << sharpness_norm << ")" << std::endl;

  // double curverad = calc_curverad3(coeffs, 10.0);
  // if (curverad > max_curverad) { 
  //   max_curverad = curverad;
  // }
  // if (curverad < min_curverad) {
  //   min_curverad = curverad;
  // }
  // double curverad_norm = fabs(curverad - min_curverad) / fabs(max_curverad - min_curverad);


  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // std::cout << "v: " << v << std::endl;
  // std::cout << "vdiff: " << (ref_v - v) << std::endl;
  // double limit = fabs((ref_v - v) / ref_v);
  double limit = 0.4;
  // std::cout << "limit: " << limit << std::endl;
  if (limit > 1.0) limit = 1.0;

  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -limit;
    vars_upperbound[i] = limit;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    // We want the car to brake a bit when approaching a sharp turn but not on straight road.
    // For speed 30mph this is not a problem i.e. -1.0 to 1.0 would work quite well, but
    // it stopped working after running for awhile.
    if (sharpness_norm > 0.50) {
      ref_v = min_ref_v;
      vars_lowerbound[i] = -0.00;
      vars_upperbound[i] = 1.0;
      
    }
    else {
      ref_v = max_ref_v;
      vars_lowerbound[i] = 0.8;
      vars_upperbound[i] = 1.0;

    }

    // If normalized curverad is too high or too low, that means we are not sure about the
    // terrain, so move slower. Does not work since curvature calculation gives weird result.
    // if (curverad_norm < 0.02 || curverad_norm > 0.75) {
    //   ref_v = min_ref_v;
    //   vars_lowerbound[i] = -0.1;
    //   vars_upperbound[i] = 1.0;
    // }
    // else {
    //   ref_v = max_ref_v;
    //   vars_lowerbound[i] = -0.0;
    //   vars_upperbound[i] = 1.0;

    // }

    // Standard
    // vars_lowerbound[i] = -0.001;
    // vars_upperbound[i] = 1.0;
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
  FG_eval fg_eval(coeffs);

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

  // Cost
  auto cost = solution.obj_value;
  // std::cout << "Cost " << cost << std::endl;

  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start],   solution.x[a_start]};
}
