#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "utilities/utilities.hpp"

/** @file trajectory.hpp
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2020-03-27
 *
 * @copyright Copyright (c) 2020
 *
 */

/**
 * @brief Function returns the cost at a <state, action> pair when provided the
 * current state, next state and current action.
 *
 */
typedef std::function<std::pair<std::pair<double, double>, double>(
    double, double, double, int)>
    costFuncPtr;

/**
 * @brief Function that returns the next state based on the state dynamics
 * equation.
 *
 */
typedef std::function<double(double, double)> nextStateFuncPtr;

/**
 * @struct Constraint trajectory.hpp "trajectory/trajectory.hpp"
 * @brief Constraints class to define the state and action space.
 *
 */
struct Constraint
{
  /**
   * @brief Construct a new Constraint object
   *
   * @param min
   * @param max
   * @param delta
   * @param segments
   */
  Constraint(double mn, double mx, double del = 0., int seg = 0)
    : min(mn), max(mx), delta(del), segments(seg)
  {
  }

  /** minimum value */
  double min;
  /** maximum value */
  double max;
  /** delta increment */
  double delta;
  /** number of evenly splitting segments */
  int segments;
};

/**
 * @class Trajectory trajectory.hpp "trajectory/trajectory.hpp"
 * @brief Generate a trajectory for a given step.
 *
 */
class Trajectory
{
private:
  /**
   * @brief Populates the next states based on the known state equation
   *
   */
  void populateNextStates();

  /**
   * @brief Calculates the next state given current state and action
   *
   * @param cur_state
   * @param action
   * @return double
   */
  double nextState(double cur_state, double action,
                   nextStateFuncPtr func = nullptr);

  /**
   * @brief Checks if @a state is in @a Env::states
   *
   * @param state
   * @return true
   * @return false
   */
  bool isValidState(double cur_state);

  template <typename T1, typename T2>
  static inline bool compare(std::pair<T1, T2> i, std::pair<T1, T2> j)
  {
    return i.second < j.second;
  }

  template <typename T1, typename T2>
  static std::pair<T1, T2> getMin(std::map<T1, T2> costmap)
  {
    std::pair<T1, T2> min = *min_element(costmap.begin(), costmap.end(),
                                         &Trajectory::compare<T1, T2>);
    return min;
  }

public:
  /** nth trajectory number */
  int traj_number;

  /** Allowed states */
  std::vector<double> cur_states;

  /** Next states */
  std::vector<double> next_states;

  /** Allowed actions */
  std::vector<double> actions;

  /** Cost at <cur_state, action> C*_cur_last(state,action) */
  std::map<std::pair<double, double>, double> costs;

  /** Minimum cost at <state> J*_cur_last(state,action) */
  std::map<double, double> min_costs;

  /** Optimal control action */
  std::map<double, double> optimal_actions;

  /**
   * @brief Construct a new Trajectory object
   *
   * @param step
   * @param state_constraint
   * @param action_constraint
   */
  Trajectory(int step, Constraint sc, Constraint ac);

  /**
   * @brief Construct a new Trajectory object
   *
   */
  Trajectory() {}

  /**
   * @brief Compute the cost
   *
   * @param func
   */
  void computeCost(costFuncPtr func, nextStateFuncPtr nextStateFunc = nullptr);

  /**
   * @brief Return the set of optimal actions possible from previous state
   *
   * @param prev_state
   */
  std::pair<double, double>
  computeOptimalTrajectory(double prev_state, nextStateFuncPtr func = nullptr);

  /**
   * @brief Interpolate the value of cost at the next state
   *
   * @param trajectory
   * @param next_state
   * @return double
   */
  static double interpolate(Trajectory trajectory, double next_state);
};

#endif  // TRAJECTORY_HPP