#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <math.h>

using CppAD::AD;

int N = 10;
double dt = 0.1; //100ms (set equal to process latency)

double v_ref = 45; //reference speed in m/s

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

//set the indeces of the vars structure
int x_start = 0;
int y_start = N;
int psi_start = N*2;
int v_start = N*3;
int cte_start = N*4;
int epsi_start = N*5;
int delta_start = N*6;
int a_start = N*6 + (N-1);


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
	Eigen::VectorXd weights;
  FG_eval(Eigen::VectorXd coeffs, Eigen::VectorXd weights) { this->coeffs = coeffs; this->weights = weights;}

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
 		
		//initialize cost value
		fg[0] = 0;
		
		// The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += weights[0] * CppAD::pow(vars[cte_start + t], 2); //minimize cross-track error
      fg[0] += weights[1] * CppAD::pow(vars[epsi_start + t], 2); //minimize orientation error
      fg[0] += weights[2] * CppAD::pow(vars[v_start + t] - v_ref,2); //keep speed as high as possible
    }
		
		// Minimize steering magnitude
    for (int t = 0; t < N - 1; t++) {
      fg[0] += weights[3] * CppAD::pow(vars[delta_start + t], 2); //minimize steering angle
   }

		// Minimize first derivative of steering
    for (int t = 0; t < N - 2; t++) {
      fg[0] += weights[4] * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    }		
		
		// Minimize second derivative of steering
    for (int t = 0; t < N - 3; t++) {
      fg[0] += weights[5] * CppAD::pow(vars[delta_start + t + 2] - 2*vars[delta_start + t + 1] + vars[delta_start + t], 2);
    }		
		
		//constraints - initial state
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];
		
		//constraints - next steps
		for (int t = 1; t < N ; t++) {

			//values at time t
			AD<double> x0 = vars[x_start + t - 1];
			AD<double> y0 = vars[y_start + t - 1];
			AD<double> psi0 = vars[psi_start + t - 1];
			AD<double> v0 = vars[v_start + t - 1];
			AD<double> cte0 = vars[cte_start + t - 1];
			AD<double> epsi0 = vars[epsi_start + t - 1];
			
			AD<double> delta0 = vars[delta_start + t - 1];
			AD<double> a0 = vars[a_start + t - 1];

			//values at time t+1
			AD<double> x1 = vars[x_start + t];
			AD<double> y1 = vars[y_start + t];
			AD<double> psi1 = vars[psi_start + t];
			AD<double> v1 = vars[v_start + t];
			AD<double> cte1 = vars[cte_start + t];
			AD<double> epsi1 = vars[epsi_start + t];
			
			AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;	//cubic polynomial evaluated at point x0
			AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0); //first derivative evaluated at point x0
	
			//constraints for optimizer: var(t-1) - var(t) = 0
			fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
			fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
			fg[1 + cte_start + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
			fg[1 + epsi_start + t] = epsi1 - (psi0 - psides0 + v0 * delta0 / Lf * dt);
			
		}
		
		
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, Eigen::VectorXd weights) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;


	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];	

  // Set the number of model variables (includes both states and inputs).
  int n_vars = 6 * N + 2 * (N-1); //6 for state and 2 for actions (no actions for initial state)
  
	//Set the number of constraints
  int n_constraints = 6 * N; //model equations for each time step

  // Initial value of the independent variables.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
	
	// Set the initial variable values
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
	//Set lower and upper limits for steering
	for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.463322; //25degs to radians
    vars_upperbound[i] = 0.463322;
  }
	//Set lower and upper limits for throttle
	for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1;
    vars_upperbound[i] = 1;
  }
	//No limits for other variables
	for (int i = 0; i < delta_start; i++) {
		vars_lowerbound[i] = -1.0e+37; 
		vars_upperbound[i] = 1.0e+37; 
	} 

	
	
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
	
	//costraints for initial state
	constraints_lowerbound[x_start] = x;
	constraints_upperbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_upperbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_upperbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_upperbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_upperbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi;
	constraints_upperbound[epsi_start] = epsi;
	
	
	
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, weights);

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
  std::cout << "Cost " << cost << std::endl;
	
  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
	
	vector<double> result = {solution.x[delta_start], solution.x[a_start]};
	//std::cout << "Steering " << result[0] << std::endl;
	//std::cout << "Throttle " << result[1] << std::endl;
	
	//also return the predicted path ahead
	for (int i=0; i<N; ++i) {
		result.push_back(solution.x[x_start+i]);
		result.push_back(solution.x[y_start+i]);
	}
		
  return result;
}
