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
  Car car_;
  vector<SensorFusionData> sensor_fusion_data_;

  vector<double> fitPolynomial(vector<double> start, vector <double> end, double time_steps);

  int testBestLane();

  Path generateInitialPath();
  Path generateFollowPath();
  Path generateLaneChangePath();

  Path cartesianPathFromCoefficients(vector<double> s_coeffs, vector<double> d_coeffs, vector<double> path_x, vector<double> path_y, double T);

  bool testPathFeasibility(Path &path);
  double scorePath(Path &path);
public:
  void setState(Car &car, Map &map, vector<SensorFusionData> &sensor_fusion_data);

  vector<Path> generatePaths();

  Path normalizeWithSpline(Path &path);
};

#endif //PATH_PLANNING_PATHGENERATOR_H
