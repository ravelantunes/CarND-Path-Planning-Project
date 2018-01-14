//
// Created by Ravel Antunes on 10/22/17.
//
#include <vector>
using namespace std;

#ifndef PATH_PLANNING_PATHSTRUCS_H
#define PATH_PLANNING_PATHSTRUCS_H

struct PathCartesian {
  vector<double> x_points;
  vector<double> y_points;
  double cost;
};

struct PathFrenet {
  vector<double> s_points;
  vector<double> d_points;
  double cost;
};

#endif //PATH_PLANNING_PATHSTRUCS_H