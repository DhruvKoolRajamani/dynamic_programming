#include "utilities/utilities.hpp"

/**
 * @file main.cpp
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2020-03-26
 *
 *
 */

/**
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
 */

int main(void)
{
  // 1)
  std::vector<double> states = Utilities::linspace(0., 1., 3);
  std::vector<double> actions = Utilities::linspace(-0.4, 0.4, 5);

  return 0;
}