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

vector<double> GlobalToCarCoordinates(double global_x, double global_y, double car_x, double car_y, double car_angle);

vector<double> GlobalToCarCoordinates(double global_x, double global_y, Car &car) {
  const double relative_x = global_x - car.getX();
  const double relative_y = global_y - car.getY();
  const double translated_x = relative_x * cos(0-car.getAngle()) - relative_y * sin(0-car.getAngle());
  const double translated_y = relative_x * sin(0-car.getAngle()) + relative_y * cos(0-car.getAngle());
  return {translated_x, translated_y};
}

double EvaluatePoly(vector<double> coeffs, double x) {
  double value = 0;
  for (unsigned int c = 0; c < coeffs.size(); c++) {
    value += pow(x, c) * coeffs[c];
  }
  return value;
}

#endif //PATH_PLANNING_HELPERS_H
