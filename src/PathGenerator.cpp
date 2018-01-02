//
// Created by Ravel Antunes on 11/3/17.
//
#include "spline.h"
#include "PathStructs.h"
#include "PathGenerator.h"
#include <vector>
#include <iostream>
#include <cmath>
#include "Helpers.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include <iomanip>

using namespace std;

vector<PathCartesian> PathGenerator::generatePaths(ControlState controlState, Car &car, Map &map) {

  //Print car state
//  car.printState();

  //First, we erase some of the steps of the previous planned path
  //The reason for that is that the simulator will usually only be able to perform about 3 steps between iterations,
  //which wouldn't allow for the path planner to plan a full path
  const unsigned long initial_planned_path_size = car.getCurrentPath().x_points.size();

  bool already_generated_path = false;
  if (initial_planned_path_size > 0) {
    already_generated_path = true;
  }

  PathCartesian previous_path = car.getCurrentPath();
//  if (previous_path.x_points.size() > number_of_steps_to_reuse_) {
//    previous_path.x_points.erase(previous_path.x_points.begin()+number_of_steps_to_reuse_, previous_path.x_points.end());
//    previous_path.y_points.erase(previous_path.y_points.begin()+number_of_steps_to_reuse_, previous_path.y_points.end());
//  }

  //Copy the remaining steps of the previous path into the new paths
  vector<double> new_path_x, new_path_y;
  for (int i = 0; i < previous_path.x_points.size(); i++) {
    new_path_x.push_back(previous_path.x_points[i]);
    new_path_y.push_back(previous_path.y_points[i]);
  }

  //Then, we fit a polynomial from to use to generate the paths
  // State is position, speespl(i);d, acceleration
  double time_step_modifier = 0.5;
  double time_steps = default_path_length_;
  vector<double> start_s = {0, 0, 0};
  vector<double> end_s = {.4, 0, 0};
  vector<double> s_poly_coeffs = fitPolynomial(start_s, end_s, time_steps);
  vector<double> d_poly_coeffs = fitPolynomial({0, 0, 0}, {0, 0, 0}, time_steps);

  //Iterate to create paths points until it fills up the expected number of paths
  for (auto i = new_path_x.size(); i < default_path_length_; i++) {
//    if (already_generated_path) {
//      break;
//    }
    //Step to be evaluated
    double step = i+1;

    ref_s_ += EvaluatePoly(s_poly_coeffs, step);
    ref_d_ += EvaluatePoly(d_poly_coeffs, step);
    double s_point = ref_s_;
    double d_point = ref_d_;
    d_point = 6;

    vector<double> xy_points = getXY(s_point, d_point, map.s_waypoints, map.x_waypoints, map.y_waypoints);
    new_path_x.push_back(xy_points[0]);
    new_path_y.push_back(xy_points[1]);
;
    assert(new_path_x.size() == new_path_y.size());
  }

  PathCartesian path = {
      .x_points = new_path_x,
      .y_points = new_path_y
  };

  path = normalizeWithSpline(path, car);

  vector<PathCartesian> paths;
  paths.push_back(path);
  return paths;
}

vector<double> PathGenerator::getRefValuesForCurrentPath(Car &car, vector<double> &new_path_x, vector<double> &new_path_y) {
  double previous_x, previous_y, ref_angle;

  //If it generating the path for the first time, previous steps will be empty, so first 2 points needs to be
  // generated manually.
  if (new_path_x.size() < 1) {
    previous_x = car.getX();
    previous_y = car.getY();
    ref_angle = car.getAngle();
    ref_s_ = car.getS();
    ref_d_ = car.getD();
  }
  else {
    //Get heading based on last 2 steps
    previous_x = new_path_x[new_path_x.size() - 1];
    previous_y = new_path_y[new_path_y.size() - 1];

    double second_to_last_x = new_path_x[new_path_x.size() - 2];
    double second_to_last_y = new_path_y[new_path_y.size() - 2];

    ref_angle = atan2(previous_y - second_to_last_y, previous_x - second_to_last_x);
  }

  return {previous_x, previous_y, ref_angle};
}


vector<double> PathGenerator::fitPolynomial(vector<double> start, vector <double> end, double time_steps) {
  Eigen::MatrixXd matrix_a = Eigen::MatrixXd(3, 3);
  matrix_a << pow(time_steps, 3), pow(time_steps, 4), pow(time_steps, 5),
      3*pow(time_steps, 2), 4*pow(time_steps,3), 5*pow(time_steps, 4),
      6*time_steps, 12*pow(time_steps, 2), 20*pow(time_steps, 3);

  Eigen::MatrixXd matrix_b = Eigen::MatrixXd(3, 1);
  matrix_b << end[0] - (start[0] + start[1] * time_steps + 0.5 * start[2] * pow(time_steps, 2)),
      end[1] - (start[1] + start[2] * time_steps),
      end[2] - start[2];

  Eigen::MatrixXd matrix_a_inverse = matrix_a.inverse();

  Eigen::VectorXd matrix_c = matrix_a_inverse * matrix_b;

  vector<double> result = {start[0], start[1], .5*start[2], matrix_c[0], matrix_c[1], matrix_c[2]};
  return result;
}

PathCartesian PathGenerator::normalizeWithSpline(PathCartesian &path, Car &car) {

  //Convert path into local coordinates
  vector<double> local_x, local_y;
  for (int i = 0; i < path.x_points.size(); i++) {
    vector<double> local_coords = GlobalToCarCoordinates(path.x_points[i], path.y_points[i], car);
    local_x.push_back(local_coords[0]);
    local_y.push_back(local_coords[1]);
  }

  //Fit spline with local coordinates
  vector<double> s_y, s_x, time_step;
  int counter_of_spline_points = 0;
  for (int i = 0; i < path.x_points.size(); i+=30) {
    counter_of_spline_points++;
    s_x.push_back(local_x[i]);
    s_y.push_back(local_y[i]);
    time_step.push_back((double)i);
  }

  //Make sure we fit with last point
  s_x.push_back(local_x[local_x.size()-1]);
  s_y.push_back(local_y[local_y.size()-1]);
  time_step.push_back(local_x.size());

  tk::spline spl_x, spl_y;
  spl_x.set_points(time_step, s_x);
  spl_y.set_points(time_step, s_y);

  //Transform back to global coordinates
  for (int i = 0; i < path.x_points.size(); i++) {
    double x = spl_x(i);
    double y = spl_y(i);

    const double translated_x = x * cos(car.getAngle()) - y * sin(car.getAngle()) + car.getX();
    const double translated_y = x * sin(car.getAngle()) + y * cos(car.getAngle()) + car.getY();

//    DEBUGGING
//    double x_diff = (translated_x - path.x_points[i]);
//    double y_diff = (translated_y - path.y_points[i]);
//    cout << "X: " << path.x_points[i] << ", " << translated_x << "  " << "(" << x_diff << ")";
//    cout << "     ";
//    cout << "Y: " << path.y_points[i] << ", " << translated_y << "  " << "(" << y_diff << ")";
//    cout << endl;

    path.x_points[i] = translated_x;
    path.y_points[i] = translated_y;
  }

  PathCartesian new_path = {
      path.x_points,
      path.y_points
  };

  return new_path;
}

tk::spline PathGenerator::fitSplineWithPath(PathCartesian &path) {
//  //  //Transform spline points into local coordinate
//  for (int i = 0; i < path.x_points.size(); i++) {
//    double shift_x = path.x_points[i] - ref_x;
//    double shift_y = path.x_points[i] - ref_y;
//
//    path.x_points[i] = shift_x * cos(0 - ref_angle) - shift_y * sin(0 - ref_angle);
//    path.x_points[i] = shift_x * sin(0 - ref_angle) + shift_y * cos(0 - ref_angle);
//  }

  tk::spline spl;
  spl.set_points(path.x_points, path.y_points);
  return spl;
}

tk::spline PathGenerator::fitSpline(int target_lane, double target_velocity) {
//  PathCartesian previous_path = car.getCurrentPath();
//
//  double remaining_previous_steps_count = previous_path.x_points.size();
//  vector<double> spline_points_x, spline_points_y;
//
//  double ref_x = car_.getX();
//  double ref_y = car_.getY();
//  double ref_angle = car_.getAngle();
//
//  const int max_path_size = 150;
//
//  /**
//   * Usually there will be around 47 remaining steps from previous iteration. However, we need at least 2 points
//   * to always be there, so we make sure to fill it up with car current and previous estimated position
//   **/
//  double last_wp_s;
//  if (remaining_previous_steps_count < 2) {
//    double prev_car_x = car_.getX() - cos(car_.getYaw());
//    double prev_car_y = car_.getY() - sin(car_.getYaw());
//
//    spline_points_x.push_back(prev_car_x);
//    spline_points_x.push_back(car_.getX());
//
//    spline_points_y.push_back(prev_car_y);
//    spline_points_y.push_back(car_.getY());
//
//    last_wp_s = car_.getS();
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
//    auto last_frenet = getFrenet(ref_x, ref_y, ref_angle, map_.x_waypoints, map_.y_waypoints);
//    last_wp_s = last_frenet[0];
//  }
//
//  int wp_increment = (rand()%40) + 15;
//  const int number_of_wp_to_use = 5;
//  for (int i = 0; i < number_of_wp_to_use; i++) {
//    vector<double> next_wp = getXY(last_wp_s + wp_increment * (i+1), LaneToD(target_lane), map_.s_waypoints, map_.x_waypoints, map_.y_waypoints);
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

  tk::spline spl;
//  spl.set_points(spline_points_x, spline_points_y);

  return spl;
};

//
//PathCartesian PathGenerator::inferSpline(tk::spline spline, vector<double> &next_x_vals, vector<double> &next_y_vals) {
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
//    x_point = x_ref * cos(ref_angle_) - y_ref * sin(ref_angle_);
//    y_point = x_ref * sin(ref_angle_) + y_ref * cos(ref_angle_);
//
//    x_point += ref_x_;
//    y_point += ref_y_;
//
//    next_x_vals.push_back(x_point);
//    next_y_vals.push_back(y_point);
//  }
//
//  return {next_x_vals, next_y_vals};
//}