#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

// Set the timestep length and duration

size_t N ;
double dt ;
double Lf ;
double v_max ;
double rev_w ;

// //放進陣列的順序（把所有狀態寫在一個陣列中）
// size_t x_start = 0;                       //x_start 0~9
// size_t y_start = x_start + N;             //y_start 10~19
// size_t psi_start = y_start + N;           //psi_start 20~29
// size_t v_start = psi_start + N;           //v_start 30~39
// size_t cte_start = v_start + N;           //cte_start 40~49
// size_t epsi_start = cte_start + N;        //epsi_start 50~59
// //size_t delta_start = epsi_start + N;      //delta_start 60~68
// size_t a_start = epsi_start + N - 1;     //a_start 69~77

size_t x_start, y_start, psi_start, v_start, cte_start, epsi_start, delta_start, a_start;

class FG_eval {
 public:
  // // Fitted polynomial coefficients
   Eigen::VectorXd coeffs;
   FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  // //typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
   typedef std::vector<AD<double>> ADvector;
   void operator()(ADvector& fg, const ADvector& vars) {

    fg[0] = 0;
    //每個參數的權重

    // const int cte_cost_weight = 7000;
    // const int epsi_cost_weight = 6000;
    // const int v_cost_weight = 1000;
    // const int delta_cost_weight = 45;
    // const int a_cost_weight = 1;
    // const int delta_change_cost_weight = 40;
    // const int a_change_cost_weight = 1;
//8000 0.06 11 0.05 30
    const int cte_cost_weight = 8000;//8000
    const int epsi_cost_weight = 1500;//2500
    const int v_cost_weight = 1000;
    const int delta_cost_weight = 120;//45
    const int a_cost_weight = 1;
    const int delta_change_cost_weight = 40;
    const int a_change_cost_weight = 1;



    //fg[0]:已什麼樣的方程式求解最佳 初始值不重要 重點是方程式
    //fg[x]:再x當下受到的限制方程式constraints[i+1]  x = i+1

    // Cost for CTE, psi error and velocity
    for (int t = 0; t < N; t++) {
      fg[0] += cte_cost_weight * CppAD::pow(vars[cte_start + t] , 2);
      fg[0] += epsi_cost_weight * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += v_cost_weight * CppAD::pow(vars[v_start + t] - v_max, 2);
    }

    // Costs for steering (delta) and acceleration (a)
    for (int t = 0; t < N-1; t++) {
      fg[0] += delta_cost_weight * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += a_cost_weight * CppAD::pow(vars[a_start + t], 2);
    }

    // Costs related to the change in steering and acceleration (makes the ride smoother)
    for (int t = 0; t < N-2; t++) {
      fg[0] += delta_change_cost_weight * pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += a_change_cost_weight * pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Setup Model Constraints

    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // State at time t + 1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // State at time t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Actuator constraints at time t only
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      //AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2);
      //AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3) + coeffs[4] * pow(x0, 4);
      //AD<double> psi_des0 = -CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0,2));

      // Setting up the rest of the model constraints
      // fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      // fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      // fg[1 + psi_start + t] = psi1 - (psi0 - delta0*v0/Lf * dt);
      // fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      // fg[1 + cte_start + t] = cte1 - ((f0-y0) + (v0 * CppAD::sin(epsi0) * dt));
      // fg[1 + epsi_start + t] = epsi1 - ((psi_des0 - psi0  ) - delta0*v0/Lf * dt);
      //AD<double> psi_des0 = -CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 );
//AD<double> psi_des0 = -CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0,2));
AD<double> psi_des0 = -CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0,2) + 4*coeffs[4]*pow(x0,3));

fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 + delta0 * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] = cte1 - ((f0-y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] = epsi1 - ((psi0 + psi_des0) + delta0 * dt);

// fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
// fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
// fg[1 + psi_start + t] = psi1 - (psi0 + delta0*fabs(v0)/Lf * dt);
// fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
// fg[1 + cte_start + t] = cte1 - ((f0-y0) + (v0 * CppAD::sin(epsi0) * dt));
// fg[1 + epsi_start + t] = epsi1 - ((psi0 + psi_des0) + delta0*fabs(v0)/Lf * dt);



    }
   }
};



class MPC
{
public:
  MPC();
  ~MPC();
  void satParameter_two(size_t N_loc,double dt_loc,const double Lf_loc);

  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,double v_max_loc);
private:


};
MPC::MPC() {}
MPC::~MPC() {}
void MPC::satParameter_two(size_t N_loc,double dt_loc,const double Lf_loc)
{
  N = N_loc;
  dt = dt_loc;
  Lf = Lf_loc;

  //放進陣列的順序（把所有狀態寫在一個陣列中）
  x_start = 0;                       //x_start 0~9
  y_start = x_start + N;             //y_start 10~19
  psi_start = y_start + N;           //psi_start 20~29
  v_start = psi_start + N;           //v_start 30~39
  cte_start = v_start + N;           //cte_start 40~49
  epsi_start = cte_start + N;        //epsi_start 50~59
  delta_start = epsi_start + N;      //delta_start 60~68
  a_start = delta_start + N - 1;     //a_start 69~77
}
std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,double v_max_loc) {

  v_max = v_max_loc;



  bool ok = true;
  //typedef CPPAD_TESTVECTOR(double) Dvector;
  //typedef std::vector<double> Dvector;

  // State vector holds all current values neede for vars below
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];


// std::cout<<"=====predict======== " <<std::endl;
// std::cout<<"v_max "<< v_max <<std::endl;
//   std::cout<<"x "<< x <<std::endl;
//   std::cout<<"y "<< y <<std::endl;
//   std::cout<<"psi "<< psi <<std::endl;
//   std::cout<<"v "<< v <<std::endl;
//   std::cout<<"cte "<< cte <<std::endl;
//   std::cout<<"epsi "<< epsi <<std::endl;

  // Setting the number of model variables (includes both states and inputs).
  // N * state vector size + (N - 1) * 2 actuators (For steering & acceleration)
  //6個狀態 + 2個輸入 共78個 跟上面的分類矩陣size一樣(0~77)
  //size_t n_vars = N * 6 + (N - 1) * 1;
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Setting the number of constraints
  //6個狀態 的size
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  //將陣列 vars初始化為0  vars.size() = 78
  std::vector<double> vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  // for (int i = v_start; i < cte_start; i++) {
  //   vars[i] = 20.0;
  //
  // }

  //陣列 vars_lowerbound.size() = 78
  //陣列 vars_upperbound.size() = 78
  //底下開始初始化vars陣列 依照性質初始化不同限制的參數（軟限制） 例如：位置不受限制為無限大 加速度 轉向角受到限制
  std::vector<double> vars_lowerbound(n_vars);
  std::vector<double> vars_upperbound(n_vars);
  // Sets lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.


  //狀態類型初始化成正無限大到負無限大
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    //std::cout << " vars_lowerbound[0] " <<vars_lowerbound[0]<< std::endl;
    vars_upperbound[i] = 1.0e19;
  }

  // for (int i = 0; i < a_start; i++) {
  //   vars_lowerbound[i] = -1.0e19;
  //   vars_upperbound[i] = 1.0e19;
  // }

  //限制類型（速度限制）初始化成正負？速度（JEFF）
  // for (int i = psi_start; i < v_start; i++) {
  //   vars_lowerbound[i] = -0.03;
  //   vars_upperbound[i] = 0.03;
  // }

  //限制類型（速度限制）初始化成正負？速度（JEFF）
  for (int i = v_start; i < cte_start; i++) {
    vars_lowerbound[i] = -0.65;
    vars_upperbound[i] = 0.65;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  //限制類型（方向盤限制）初始化成正負25度
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.9;
    vars_upperbound[i] = 0.9;
  }

  // Acceleration/decceleration upper and lower limits.
  //限制類型（加速度限制）初始化成正負1
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -0.3;
    vars_upperbound[i] = 0.3;
  }




  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  //陣列 constraints_lowerbound.size() = 60
  //陣列 constraints_upperbound.size() = 60
  //底下開始初始化constraints陣列 這部份為約束值
  std::vector<double> constraints_lowerbound(n_constraints);
  std::vector<double> constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }
  // for (int i = v_start; i < N; i++) {
  //   constraints_lowerbound[i] = 0.1;
  //   constraints_upperbound[i] = 0.1;
  // }

  // Start lower and upper limits at current values
  //將第一個時刻之狀態放進去
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

  //將擬合之路徑丟入FG_eval建構子 裏面純粹純值
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
  options += "Numeric max_cpu_time          0.1\n";

  // place to return solution
   CppAD::ipopt::solve_result<std::vector<double>> solution;
//cout <<"D" << endl;
  // solve the problem

  CppAD::ipopt::solve<std::vector<double>, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
//cout <<"E" << endl;
  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<std::vector<double>>::success;

  // Cost
  auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values, along with predicted x and y values to plot in the simulator.
  std::vector<double> solved;
  solved.push_back(solution.x[delta_start]);
  solved.push_back(solution.x[a_start]);
  for (int i = 0; i < N; ++i) {
    solved.push_back(solution.x[x_start + i]);
    solved.push_back(solution.x[y_start + i]);
  }

  // std::cout<<"=====solved   0======== " <<std::endl;
  //
  //   std::cout<<"x "<< solution.x[x_start] <<std::endl;
  //   std::cout<<"y "<< solution.x[y_start] <<std::endl;
  //   std::cout<<"psi "<< solution.x[psi_start] <<std::endl;
  //    std::cout<<"v==== "<< solution.x[v_start] <<std::endl;
     // std::cout<<"a "<< solution.x[a_start] <<std::endl;
  //   std::cout<<"delta "<< solution.x[delta_start] <<std::endl;
  //
  //   std::cout<<"=====solved  1======== " <<std::endl;
  //
  //     std::cout<<"x "<< solution.x[x_start+1] <<std::endl;
  //     std::cout<<"y "<< solution.x[y_start+1] <<std::endl;
  //     std::cout<<"psi "<< solution.x[psi_start+1] <<std::endl;
      // std::cout<<"v "<< solution.x[v_start+1] <<std::endl;
      // std::cout<<"a "<< solution.x[a_start+1] <<std::endl;
  //     std::cout<<"delta "<< solution.x[delta_start+1] <<std::endl;


  return solved;

}
