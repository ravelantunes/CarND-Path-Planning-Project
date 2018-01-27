//
// Created by Ravel Antunes on 10/22/17.
//
#include <vector>
#include "ControlState.h"

using namespace std;

#ifndef PATH_PLANNING_PATHSTRUCS_H
#define PATH_PLANNING_PATHSTRUCS_H

struct Path {
  vector<double> x_points;
  vector<double> y_points;

  vector<double> s_points;
  vector<double> d_points;

  double last_s;
  double last_d;

  double cost;

  ControlState next_state;
};

#endif //PATH_PLANNING_PATHSTRUCS_H
