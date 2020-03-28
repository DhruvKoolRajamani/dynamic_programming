/**
 * @file problem_1.cpp
 * @brief Problem 1
 * Problem Formulation:
 *
 * system eq:
 * x(k+1) = x(k) - 0.4x^2(k) + u(k)
 *
 * constraints:
 *      0 <= x(k) <=   1
 *   -0.4 <= u(k) <= 0.4
 *
 * Cost:
 * J = 4*fabs(x(2)) + sum_k_0_1(u(k))
 *
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 *
 * @version 0.1
 * @date 2020-03-26
 *
 */

#include "utilities/utilities.hpp"
#include "trajectory/trajectory.hpp"

const int END_STATE = 20;

std::map<int, Trajectory> past_trajectories;

double stateDynamics(double cur_state, double action)
{
  return cur_state - (0.4 * pow(cur_state, 2)) + action;
  // return cur_state + action;
}

std::pair<std::pair<double, double>, double>
costFunction(double cur_state, double next_state, double action, int step)
{
  std::pair<std::pair<double, double>, double> cost_pair;
  double cost = 0.;

  if (step == END_STATE - 1)
  {
    cost = 4 * fabs(next_state) + fabs(action);
    // cost = pow(next_state, 2) + 2 * pow(action, 2);
#ifdef DEBUG
    std::cout << "X(k): " << cur_state << "  \t\tU: " << action
              << "\t\tX(k+1): " << next_state << "\t\tC[" << step << ','
              << END_STATE << "]: " << cost << std::endl;
#endif
  }
  else
  {
    cost = fabs(action);
    // cost = 2. * pow(action, 2);
#ifdef DEBUG
    std::cout << "X(k): " << cur_state << "  \t\tU: " << action
              << "\t\tX(k+1): " << next_state << "\t\tG[" << step << ','
              << END_STATE << "]: " << cost;
#endif
    for (int i = step + 1; i < END_STATE; i++)
    {
      cost += Trajectory::interpolate(past_trajectories[i], next_state);
#ifdef DEBUG
      std::cout << "\tJ[" << i << ',' << END_STATE << "]: "
                << Trajectory::interpolate(past_trajectories[i], next_state);
#endif
    }
#ifdef DEBUG
    std::cout << "\tC[" << step << ',' << END_STATE << "]: " << cost
              << std::endl;
#endif
  }

  cost_pair.first = std::pair<double, double>(cur_state, action);
  cost_pair.second = Utilities::round(cost, 3);

  return cost_pair;
}

int main(void)
{
  Constraint action_constraint(-0.4, 0.4, 0.02), state_constraint(0., 1., 0.02);

  for (int i = END_STATE - 1; i >= 0; i--)
  {
    Trajectory traj = Trajectory(i, state_constraint, action_constraint);
    traj.computeCost(costFunction, stateDynamics);
    past_trajectories.insert(std::pair<int, Trajectory>(i, traj));
  }

  double prev_state = 1.;
  std::vector<double> opt_x;
  std::vector<double> opt_u;
  opt_x.push_back(prev_state);
  for (int i = 0; i < END_STATE; i++)
  {
    std::pair<double, double> x_and_u_opt =
        past_trajectories[i].computeOptimalTrajectory(prev_state,
                                                      stateDynamics);
    prev_state = x_and_u_opt.first;
    opt_x.push_back(x_and_u_opt.first);
    opt_u.push_back(x_and_u_opt.second);
  }

  std::cout << "Optimal States\n";
  Utilities::print_vector(opt_x);
  std::cout << "\n";

  std::cout << "Optimal Actions\n";
  Utilities::print_vector(opt_u);
  std::cout << "\n";

  return 0;
}