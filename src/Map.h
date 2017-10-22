//
// Created by ravelantunes on 10/21/17.
//
#include <vector>

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

using namespace std;

struct Map {
  vector<double> x_waypoints;
  vector<double> y_waypoints;
  vector<double> s_waypoints;
  vector<double> dx_waypoints;
  vector<double> dy_waypoints;
};

#endif //PATH_PLANNING_MAP_H
