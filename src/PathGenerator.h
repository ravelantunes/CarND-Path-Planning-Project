//
// Created by Ravel Antunes on 10/22/17.
//
#include <vector>

#include "Map.h"
#include "Car.h"
#include "ControlState.h"
#include "spline.h"

using namespace std;

//using Eigen::MatrixXd;
//using Eigen::VectorXd;

#ifndef PATH_PLANNING_PATHGENERATOR_H
#define PATH_PLANNING_PATHGENERATOR_H

struct SensorFusionData;

class PathGenerator {
private:
  static const int number_of_steps_to_reuse_ = 50;
  static const int default_path_length_ = 100;

  double ref_x_;
  double ref_y_;
  double ref_angle_;

  double ref_s_;
  double ref_d_;

  Map map_;

//  PathCartesian inferSpline(tk::spline spline, vector<double> &next_x_vals, vector<double> &next_y_vals);
//  tk::spline fitSpline(int target_lane, double target_velocity);
  vector<double> fitPolynomial(vector<double> start, vector <double> end, double time_steps);

  int testBestLane(Car &car, vector<SensorFusionData> &sensor_fusion_data);

public:
  vector<PathCartesian> generatePaths(ControlState controlState, Car &car, Map &map, vector<SensorFusionData> &sensor_fusion_data);

  PathCartesian normalizeWithSpline(PathCartesian &path, Car &car);
};

#endif //PATH_PLANNING_PATHGENERATOR_H
