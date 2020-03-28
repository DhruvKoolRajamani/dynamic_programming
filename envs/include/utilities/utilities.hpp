#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <iostream>
#include <bits/stdc++.h>
#include <math.h>

/** @file utilities.hpp
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2020-03-26
 *
 * @copyright Copyright (c) 2020
 *
 */

/**
 * @class Utilities utilities.hpp "utilities/utilities.hpp"
 * @brief Static Utility functions usable by all methods that include this
 * header.
 *
 */
class Utilities
{
private:
public:
  /** Creates a linespace using start and end values with num_in elements
   * including the start and end values.
   *
   * @tparam T
   * @param start_in
   * @param end_in
   * @param num_in
   * @return std::vector<double>
   */
  template <typename T>
  static std::vector<double> linspace(T start_in, T end_in, int num_in)
  {
    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0)
    {
      return linspaced;
    }
    if (num == 1)
    {
      linspaced.push_back(start);
      return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for (int i = 0; i < num - 1; ++i)
    {
      linspaced.push_back(round(start + delta * i, 3));
    }
    linspaced.push_back(end);  // I want to ensure that start and end
                               // are exactly the same as the input
    return linspaced;
  }

  /**
   * @brief Creates a linespace using start and end values with delta increments
   * to and including the start and end values.
   *
   * @tparam T
   * @param start_in
   * @param end_in
   * @param delta
   * @return std::vector<double>
   */
  template <typename T>
  static std::vector<double> linspace(T start_in, T end_in, double delta)
  {
    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);

    if (delta == 0)
    {
      return linspaced;
    }

    double cnt = start;
    while (cnt < end_in)
    {
      linspaced.push_back(cnt);
      cnt += round(delta, 3);
    }
    linspaced.push_back(end);
    return linspaced;
  }

  /**
   * @brief Prints out @a vec
   *
   * @param vec
   */
  static void print_vector(std::vector<double> vec)
  {
    std::cout << "size: " << vec.size() << std::endl;
    for (double d : vec)
      std::cout << d << " ";
    std::cout << std::endl;
  }

  /**
   * @brief Rounds of a number to the number of decimal places.
   *
   * @param var
   * @param places
   * @return double
   */
  static double round(double var, int places = 0)
  {
    if (places == 0) return (int)var;
    double factor = pow(10, places);
    bool is_negative = (var < 0) ? true : false;
    double value = 0.;
    if (is_negative)
      value = (int)(var * factor - 0.5);
    else
      value = (int)(var * factor + 0.5);
    return (double)value / factor;
  }
};

#endif  // UTILITIES_HPP