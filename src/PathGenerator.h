//
// Created by Ravel Antunes on 10/22/17.
//
#include <vector>

#include "Map.h"
#include "Car.h"
#include "ControlState.h"

using namespace std;

//using Eigen::MatrixXd;
//using Eigen::VectorXd;

#ifndef PATH_PLANNING_PATHGENERATOR_H
#define PATH_PLANNING_PATHGENERATOR_H

class PathGenerator {
private:
  static const int number_of_steps_to_reuse_ = 20;
  static const int default_path_length_ = 20;

  double ref_x_;
  double ref_y_;
  double ref_angle_;

  Map map_;
  Car car_;

  PathCartesian inferSpline(tk::spline spline, vector<double> &next_x_vals, vector<double> &next_y_vals);
  tk::spline fitSpline(int target_lane, double target_velocity);
  vector<double> fitPolynomial(vector<double> start, vector <double> end, double time_steps);

public:
  vector<PathCartesian> generatePaths(ControlState controlState, Car &car, Map &map);
};

#endif //PATH_PLANNING_PATHGENERATOR_H
