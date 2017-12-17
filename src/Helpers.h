//
// Created by Ravel Antunes on 11/4/17.
//

#ifndef PATH_PLANNING_HELPERS_H
#define PATH_PLANNING_HELPERS_H

#include <vector>

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

int LaneToD(int lane);

int DToLane(double d);

vector<double> CarToGlobalCoordinates(double global_x, double global_y, double car_x, double car_y, double car_angle);

double EvaluatePoly(vector<double> coeffs, double x) {
  double value = 0;
  for (int c = 0; c < coeffs.size(); c++) {
    value += pow(x, c + 1) * coeffs[c];
  }
  return value;
}

#endif //PATH_PLANNING_HELPERS_H
