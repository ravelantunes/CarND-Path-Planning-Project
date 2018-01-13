//
// Created by Ravel Antunes on 11/3/17.
//

#include <vector>
#include "ControlState.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
//#include <iostream>
#include "Map.h"
//#include <fstream>
//#include <cmath>
//#include <vector>

//using namespace std;
//
////using Eigen::MatrixXd;
////using Eigen::VectorXd;
//
//#ifndef PATH_PLANNING_PATH_GENERATOR_TEMP_CLEANUP_H
//#define PATH_PLANNING_PATH_GENERATOR_TEMP_CLEANUP_H
//
//
//vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
//
//vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
//
//int LaneToD(int lane);
//
//int DToLane(double d);
//
//vector<double> CarToGlobalCoordinates(double global_x, double global_y, double car_x, double car_y, double car_angle);
//
//PathCartesian test(tk::spline spline, vector<double> &next_x_vals, vector<double> &next_y_vals, double ref_x, double ref_y, double ref_angle) {
//  double target_x = 30.0;
//  double target_y = spline(target_x);
//  double target_dist = sqrt(target_x * target_x + target_y * target_y);
//
//  const int max_path_size = 150;
//  const double target_velocity = 40.0;
//  double x_add_on = 0;
//
//  double N = target_dist / (.02*target_velocity/2.24);
//
//  for (int i = 1; i <= max_path_size - next_x_vals.size(); i++) {
//
//    double x_point = x_add_on + (target_x) / N;
//    double y_point = spline(x_point);
//
//    x_add_on = x_point;
//
//    double x_ref = x_point;
//    double y_ref = y_point;
//
//    x_point = x_ref * cos(ref_angle) - y_ref * sin(ref_angle);
//    y_point = x_ref * sin(ref_angle) + y_ref * cos(ref_angle);
//
//    x_point += ref_x;
//    y_point += ref_y;
//
//    next_x_vals.push_back(x_point);
//    next_y_vals.push_back(y_point);
//  }
//
//  return {next_x_vals, next_y_vals};
//}
//
//tk::spline FitSpline(Car &car, int target_lane, double target_velocity, Map &map) {
//  PathCartesian previous_path = car.getCurrentPath();
//
//  const int recycle_size = 45;
//  if (previous_path.x_points.size() > recycle_size) {
//    previous_path.x_points.erase(previous_path.x_points.begin()+recycle_size, previous_path.x_points.end());
//    previous_path.y_points.erase(previous_path.y_points.begin()+recycle_size, previous_path.y_points.end());
//  }
//  double remaining_previous_steps_count = previous_path.x_points.size();
//  vector<double> spline_points_x, spline_points_y;
//
//  double ref_x = car.getX();
//  double ref_y = car.getY();
//  double ref_angle = car.getAngle();
//
//  const int max_path_size = 150;
//
//  /**
//   * Usually there will be around 47 remaining steps from previous iteration. However, we need at least 2 points
//   * to always be there, so we make sure to fill it up with car current and previous estimated position
//   **/
//  double last_wp_s;
//  if (remaining_previous_steps_count < 2) {
//    double prev_car_x = car.getX() - cos(car.getYaw());
//    double prev_car_y = car.getY() - sin(car.getYaw());
//
//    spline_points_x.push_back(prev_car_x);
//    spline_points_x.push_back(car.getX());
//
//    spline_points_y.push_back(prev_car_y);
//    spline_points_y.push_back(car.getY());
//
//    last_wp_s = car.getS();
//  }
//  else {
//    ref_x = previous_path.x_points[remaining_previous_steps_count-1];
//    ref_y = previous_path.y_points[remaining_previous_steps_count-1];
//
//    double ref_x_prev = previous_path.x_points[remaining_previous_steps_count-2];
//    double ref_y_prev = previous_path.y_points[remaining_previous_steps_count-2];
//    ref_angle = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
//
//    spline_points_x.push_back(ref_x_prev);
//    spline_points_x.push_back(ref_x);
//
//    spline_points_y.push_back(ref_y_prev);
//    spline_points_y.push_back(ref_y);
//
//    auto last_frenet = getFrenet(ref_x, ref_y, ref_angle, map.x_waypoints, map.y_waypoints);
//    last_wp_s = last_frenet[0];
//  }
//
//  int wp_increment = (rand()%40) + 15;
//  const int number_of_wp_to_use = 5;
//  for (int i = 0; i < number_of_wp_to_use; i++) {
//    vector<double> next_wp = getXY(last_wp_s + wp_increment * (i+1), LaneToD(target_lane), map.s_waypoints, map.x_waypoints, map.y_waypoints);
//    spline_points_x.push_back(next_wp[0]);
//    spline_points_y.push_back(next_wp[1]);
//  }
//
//  //Transform spline points into local coordinate
//  for (int i = 0; i < spline_points_x.size(); i++) {
//    double shift_x = spline_points_x[i] - ref_x;
//    double shift_y = spline_points_y[i] - ref_y;
//
//    spline_points_x[i] = shift_x * cos(0 - ref_angle) - shift_y * sin(0 - ref_angle);
//    spline_points_y[i] = shift_x * sin(0 - ref_angle) + shift_y * cos(0 - ref_angle);
//  }
//
//  tk::spline spl;
//  spl.set_points(spline_points_x, spline_points_y);
//
//  return spl;
//};
//
///**
// * Generate a path
// *
// * @param car
// * @return vector of vectors, where index 0 is x points and index 1 is y points
// */
//PathCartesian GeneratePathSpline(Car &car, int target_lane, double target_velocity, Map &map) {
//  PathCartesian previous_path = car.getCurrentPath();
//
//  const int recycle_size = 45;
//  if (previous_path.x_points.size() > recycle_size) {
//    previous_path.x_points.erase(previous_path.x_points.begin()+recycle_size, previous_path.x_points.end());
//    previous_path.y_points.erase(previous_path.y_points.begin()+recycle_size, previous_path.y_points.end());
//  }
//  double remaining_previous_steps_count = previous_path.x_points.size();
//  vector<double> spline_points_x, spline_points_y;
//
//  double ref_x = car.getX();
//  double ref_y = car.getY();
//  double ref_angle = car.getAngle();
//
//  const int max_path_size = 150;
//
//  /**
//   * Usually there will be around 47 remaining steps from previous iteration. However, we need at least 2 points
//   * to always be there, so we make sure to fill it up with car current and previous estimated position
//   **/
//  double last_wp_s;
//  if (remaining_previous_steps_count < 2) {
//    double prev_car_x = car.getX() - cos(car.getYaw());
//    double prev_car_y = car.getY() - sin(car.getYaw());
//
//    spline_points_x.push_back(prev_car_x);
//    spline_points_x.push_back(car.getX());
//
//    spline_points_y.push_back(prev_car_y);
//    spline_points_y.push_back(car.getY());
//
//    last_wp_s = car.getS();
//  }
//  else {
//    ref_x = previous_path.x_points[remaining_previous_steps_count-1];
//    ref_y = previous_path.y_points[remaining_previous_steps_count-1];
//
//    double ref_x_prev = previous_path.x_points[remaining_previous_steps_count-2];
//    double ref_y_prev = previous_path.y_points[remaining_previous_steps_count-2];
//    ref_angle = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
//
//    spline_points_x.push_back(ref_x_prev);
//    spline_points_x.push_back(ref_x);
//
//    spline_points_y.push_back(ref_y_prev);
//    spline_points_y.push_back(ref_y);
//
//    auto last_frenet = getFrenet(ref_x, ref_y, ref_angle, map.x_waypoints, map.y_waypoints);
//    last_wp_s = last_frenet[0];
//  }
//
//  int wp_increment = (rand()%40) + 15;
//  const int number_of_wp_to_use = 5;
//  for (int i = 0; i < number_of_wp_to_use; i++) {
//    vector<double> next_wp = getXY(last_wp_s + wp_increment * (i+1), LaneToD(target_lane), map.s_waypoints, map.x_waypoints, map.y_waypoints);
//    spline_points_x.push_back(next_wp[0]);
//    spline_points_y.push_back(next_wp[1]);
//  }
//
//  //Transform spline points into local coordinate
//  for (int i = 0; i < spline_points_x.size(); i++) {
//    double shift_x = spline_points_x[i] - ref_x;
//    double shift_y = spline_points_y[i] - ref_y;
//
//    spline_points_x[i] = shift_x * cos(0 - ref_angle) - shift_y * sin(0 - ref_angle);
//    spline_points_y[i] = shift_x * sin(0 - ref_angle) + shift_y * cos(0 - ref_angle);
//  }
//
//  tk::spline spl;
//  spl.set_points(spline_points_x, spline_points_y);
//
//
//  //Copy values from the previous path
//  vector<double> next_x_vals, next_y_vals;
//  for (int i = 0; i < previous_path.x_points.size(); i++) {
//    next_x_vals.push_back(previous_path.x_points[i]);
//    next_y_vals.push_back(previous_path.y_points[i]);
//  }
//
//  return test(spl, next_x_vals, next_y_vals, ref_x, ref_y, ref_angle);
//}
//
//vector<double> CalculateCoefficientsForJMT(vector<double> start, vector <double> end, double t) {
//  Eigen::MatrixXd matrix_a = Eigen::MatrixXd(3, 3);
//  matrix_a << pow(t, 3), pow(t, 4), pow(t, 5),
//              3*pow(t,2), 4*pow(t,3), 5*pow(t,4),
//              6*t, 12*pow(t, 2), 20*pow(t, 3);
//
//
//  Eigen::MatrixXd matrix_b = Eigen::MatrixXd(3, 1);
//  matrix_b << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * pow(t, 2)),
//              end[1] - (start[1] + start[2] * t),
//              end[2] - start[2];
//
//  Eigen::MatrixXd matrix_a_inverse = matrix_a.inverse();
//  Eigen::MatrixXd matrix_c = matrix_a_inverse * matrix_b;
//
//  vector<double> result = {start[0], start[1], .5*start[2]};
//  for(int i = 0; i < matrix_c.size(); i++)  {
//    result.push_back(matrix_c.data()[i]);
//  }
//
//  return result;
//}
//
//double EvaluatePoly(vector<double> coeffs, double x) {
//  double value = 0;
//  for (int c = 0; c < coeffs.size(); c++) {
//    value += pow(x, c + 1) * coeffs[c];
//  }
//  return value;
//}
//
////PathCartesian
//PathCartesian GeneratePathPolynomial(Car &car, int target_lane, double target_velocity, Map &map) {
//
//  //TODO: sohuld be able to copy most of the code from spline, and just change the code to call the poly function instead of spline
//  //First, move spline training outside of the main function, so it can be easily replaced by another function
//  vector<double> start_s = {0, 0, 0};
//  vector<double> start_d = {0, 0, 0};
//
//  vector<double> end_s = {100, 0, 0};
//  vector<double> end_d = {0, 0, 0};
//
//  vector<double> coeffs = CalculateCoefficientsForJMT(start_s, end_s, 10);
//  for (int i = 0; i < coeffs.size(); i++) {
//    cout << coeffs[i] << ", ";
//  }
//  cout << endl;
//
//  vector<double> s_points, d_points;
//  for (int i = 0; i < 50; i++) {
//    double s = EvaluatePoly(coeffs, i);
//    double d = 0;
//
//    s_points.push_back(s);
//    d_points.push_back(d);
//    cout << i << " -> " << s << endl;
//  }
//
//  return {s_points, d_points};
//}
//
//vector<PathCartesian> GeneratePaths(ControlState controlState, Car &car, Map &map) {
//  vector<PathCartesian> paths;
//  for (int i = 0; i < 10; i++) {
//    switch (controlState) {
//      case KEEP_LANE:
//        paths.push_back(GeneratePathSpline(car, DToLane(car.getD()), 49, map));
//        paths.push_back(GeneratePathSpline(car, DToLane(car.getD()), 40, map));
//        paths.push_back(GeneratePathSpline(car, DToLane(car.getD()), 35, map));
//        break;
//      case CHANGE_LANES:
//        break;
//      default:
//        break;
//
//    }
//
////    GeneratePathPolynomial(car, 0, 40, map);
////    paths.push_back(GeneratePath(car, 1, rand()%35 + 10, map));
////    paths.push_back(GeneratePath(car, 2, rand()%35 + 10, map));
////    paths.push_back(GeneratePath(car, 3, rand()%35 + 10, map));
//
////    paths.push_back(GeneratePath(car, 1, rand()%25 + 30, map));
////    paths.push_back(GeneratePath(car, 2, rand()%25 + 30, map));
////    paths.push_back(GeneratePath(car, 3, rand()%25 + 30, map));
//
////    paths.push_back(GeneratePath(car, 1, 30, map));
////    paths.push_back(GeneratePath(car, 2, 30, map));
////    paths.push_back(GeneratePath(car, 3, 30, map));
////
////    paths.push_back(GeneratePath(car, 1, 20, map));
////    paths.push_back(GeneratePath(car, 2, 20, map));
////    paths.push_back(GeneratePath(car, 3, 20, map));
//  }
//
////  paths.push_back(GeneratePath(car, 1, 50, map));
////  paths.push_back(GeneratePath(car, 2, 50, map));
////  paths.push_back(GeneratePath(car, 3, 50, map));
//
////  paths.push_back(GeneratePath(car, 1, 40, map));
////  paths.push_back(GeneratePath(car, 2, 40, map));
////  paths.push_back(GeneratePath(car, 3, 40, map));
//
////  paths.push_back(GeneratePath(car, 1, 30, map));
////  paths.push_back(GeneratePath(car, 2, 30, map));
////  paths.push_back(GeneratePath(car, 3, 30, map));
//
////  paths.push_back(GeneratePath(car, 1, 20, map));
////  paths.push_back(GeneratePath(car, 2, 20, map));
////  paths.push_back(GeneratePath(car, 3, 20, map));
//
//  return paths;
//}

#endif //PATH_PLANNING_PATH_GENERATOR_TEMP_CLEANUP_H
