#include "trajectory/trajectory.hpp"

/** @file trajectory.cpp
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2020-03-27
 *
 * @copyright Copyright (c) 2020
 *
 */

double Trajectory::nextState(double cur_state, double action,
                             nextStateFuncPtr func)
{
  if (func == nullptr)
    return Utilities::round(cur_state - (0.4 * pow(cur_state, 2)) + action, 3);
  else
    return Utilities::round(func(cur_state, action), 3);
}

void Trajectory::populateNextStates()
{
  for (auto cur_state : cur_states)
  {
#ifdef DEBUG
    std::cout << "Current State: " << cur_state << std::endl;
    std::cout << "\tNext States: ";
#endif
    for (auto action : actions)
    {
      double next_state =
          Utilities::round(cur_state - (0.4 * pow(cur_state, 2)) + action, 3);
      if (isValidState(next_state))
      {
#ifdef DEBUG
        std::cout << "\033[1;32m(" << action << ',' << next_state
                  << ")\033[0m, ";
#endif
        next_states.push_back(next_state);
      }
      else
      {
#ifdef DEBUG
        std::cout << '(' << action << ',' << next_state << "), ";
#endif
        auto iterator =
            std::find_if(actions.begin(), actions.end(),
                         [&](const double& a) { return a == action; });
        // if (iterator != actions.end()) actions.erase(iterator);
      }
    }
#ifdef DEBUG
    std::cout << "\n";
#endif
  }
#ifdef DEBUG
  std::cout << "\n";
#endif
}

bool Trajectory::isValidState(double cur_state)
{
  if (std::find(cur_states.begin(), cur_states.end(), cur_state) !=
      cur_states.end())
    return true;
  else
    return false;
}

Trajectory::Trajectory(int step, Constraint sc, Constraint ac)
  : traj_number(step)
{
  if (sc.segments == 0)
    cur_states = Utilities::linspace(sc.min, sc.max, sc.delta);
  else
    cur_states = Utilities::linspace(sc.min, sc.max, sc.segments);

#ifdef DEBUG
  std::cout << "Current States\n";
  Utilities::print_vector(cur_states);
  std::cout << "\n";
#endif

  if (ac.segments == 0)
    actions = Utilities::linspace(ac.min, ac.max, ac.delta);
  else
    actions = Utilities::linspace(ac.min, ac.max, ac.segments);

#ifdef DEBUG
  std::cout << "Current Actions\n";
  Utilities::print_vector(actions);
  std::cout << "\n";
#endif

  // populateNextStates();

#ifdef DEBUG
// std::cout << "Next States\n";
// Utilities::print_vector(next_states);
#endif
}

void Trajectory::computeCost(costFuncPtr func, nextStateFuncPtr nextStateFunc)
{
  std::cout << "Step:\t" << traj_number << std::endl;
#ifdef VIEW_TABLE
  std::cout << "\033[1;32m|cst\t||act\t||nst\t||cost\t||mcst\t||oact\t|\033[0m"
            << "\n";
#endif
  for (auto cur_state : cur_states)
  {
    std::map<double, double> cur_cost;
    std::map<double, std::pair<double, double>> cur_state_dyn;
    for (auto action : actions)
    {
      double next_state =
          Utilities::round(nextState(cur_state, action, nextStateFunc), 3);
      // if (isValidState(next_state))
      {
        std::pair<std::pair<double, double>, double> cost =
            func(cur_state, next_state, action, traj_number);
        costs.insert(cost);
        cur_cost.insert(std::pair<double, double>(action, cost.second));
        next_states.push_back(next_state);
        cur_state_dyn.insert(std::pair<double, std::pair<double, double>>(
            next_state, std::pair<double, double>(action, cost.second)));
      }
    }
    std::pair<double, double> min =
        Trajectory::getMin<double, double>(cur_cost);
    min_costs.insert(
        std::pair<double, double>(cur_state, Utilities::round(min.second, 3)));
    optimal_actions.insert(
        std::pair<double, double>(cur_state, Utilities::round(min.first)));

#ifdef VIEW_TABLE
    for (auto csd : cur_state_dyn)
    {
      if (csd.second.second == min_costs[cur_state])
      {
        std::cout << "|" << Utilities::round(cur_state, 2) << "\t||"
                  << Utilities::round(csd.second.first, 2) << "\t||"
                  << Utilities::round(csd.first, 2) << "\t||"
                  << Utilities::round(csd.second.second, 2) << "\t||"
                  << Utilities::round(min_costs[cur_state], 2) << "\t||"
                  << Utilities::round(optimal_actions[cur_state], 2) << "\t|\n";
      }
    }
#endif
  }
}

double Trajectory::interpolate(Trajectory next_trajectory, double next_state)
{
  if (next_trajectory.isValidState(next_state))
    return next_trajectory.min_costs[next_state];
  std::map<int, double> distanceMap;
  for (int i = 0; i < next_trajectory.cur_states.size(); i++)
  {
    double delta = fabs(next_trajectory.cur_states[i] - next_state);
    distanceMap.insert(std::pair<int, double>(i, delta));
  }
  std::pair<int, double> p1 = Trajectory::getMin<int, double>(distanceMap);
  distanceMap.erase(p1.first);
  std::pair<int, double> p2 = Trajectory::getMin<int, double>(distanceMap);

  int x1 = 0;
  int x2 = 0;
  if (next_trajectory.cur_states[p1.first] <
      next_trajectory.cur_states[p2.first])
  {
    x1 = p1.first;
    x2 = p2.first;
  }
  else
  {
    x1 = p2.first;
    x2 = p1.first;
  }

  double interpolated_cost =
      next_trajectory.min_costs[next_trajectory.cur_states[x2]] +
      (next_trajectory.min_costs[next_trajectory.cur_states[x1]] -
       next_trajectory.min_costs[next_trajectory.cur_states[x2]]) *
          ((next_state - next_trajectory.cur_states[x2]) /
           (next_trajectory.cur_states[x1] - next_trajectory.cur_states[x2]));

  return Utilities::round((interpolated_cost), 3);
}

std::pair<double, double>
Trajectory::computeOptimalTrajectory(double prev_state, nextStateFuncPtr func)
{
  double x_opt;
  double u_opt;

  u_opt = optimal_actions[prev_state];
  x_opt = nextState(prev_state, u_opt, func);

  return std::pair<double, double>(x_opt, u_opt);
}