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
  car.printState();

  //First, we erase some of the steps of the previous planned path
  //The reason for that is that the simulator will usually only be able to perform about 3 steps between iterations,
  //which wouldn't allow for the path planner to plan a full path
  const int initial_planned_path_size = car_.getCurrentPath().x_points.size();
  PathCartesian previous_path = car_.getCurrentPath();
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

  int steps_taken = default_path_length_ - initial_planned_path_size;

  //Then, we fit a polynomial from to use to generate the paths
  // State is 1st, 2nd, and 3rd dx/dy
  vector<double> start_s = {0, car.getSpeed(), 0};
  vector<double> end_s = {1, 20, 0};
  vector<double> s_poly_coeffs = fitPolynomial(start_s, end_s, steps_taken);
  vector<double> d_poly_coeffs = fitPolynomial({0, 0, 0}, {0, 0, 0}, 150);

  //We try to use the angle between the last 2 steps to determine the car heading, or just use the current car angle
  //if there are less than 2 previous steps
  double previous_x, previous_y, ref_angle;

  //Iterate to create paths points until it fills up the expected number of paths
  for (auto i = new_path_x.size(); i < default_path_length_; i++) {

    if (new_path_x.size() < 2) {
      previous_x = car.getX();
      previous_y = car.getY();
      ref_angle = car.getAngle();
    }
    else {
      //Get heading based on last 2 steps
      previous_x = new_path_x[new_path_x.size() - 1];
      previous_y = new_path_y[new_path_y.size() - 1];

      double second_to_last_x = new_path_x[new_path_x.size() - 2];
      double second_to_last_y = new_path_y[new_path_y.size() - 2];

      ref_angle = atan2(previous_y - second_to_last_y, previous_x - second_to_last_x);
    }

    double step = i+1;
//    double s_evaluated = EvaluatePoly(s_poly_coeffs, step);
    double s_evaluated = 1;
    double d_evaluated = EvaluatePoly(d_poly_coeffs, step);

//    cout << s_evaluated << ", " << d_evaluated;
//    double x_point = previous_x + (s_point * cos(ref_angle) - d_point * sin(ref_angle));
//    double y_point = previous_y + (s_point * sin(ref_angle) + d_point * cos(ref_angle));
//    cout << "   frenet: " << step << " -- " << s_point << ", " << d_point << "     cartesian: " << step << " -- " << x_point << ", " << y_point << endl;

    vector<double> frenet_points = getFrenet(previous_x, previous_y, ref_angle, map.x_waypoints, map.y_waypoints);
//    double s_point = frenet_points[0] + EvaluatePoly(s_poly_coeffs, step);
    double s_point = frenet_points[0] + s_evaluated;
    double d_point = frenet_points[1] + d_evaluated;
//    double d_point = 6; //Hard coded to mid lane for now

//    s_point = 0; /////!!!!
    vector<double> xy_points = getXY(s_point, d_point, map.s_waypoints, map.x_waypoints, map.y_waypoints);
    cout << "   frenet: " << step << " -- " << s_point << ", " << d_point << "     cartesian: " << xy_points[0] << ", " << xy_points[1];
//    cout << "   car: " << car.getX() << ", " << car.getY() << "   - prev: " << (int)previous_x << ", " << (int)previous_y << endl;

    new_path_x.push_back(xy_points[0]);
    new_path_y.push_back(xy_points[1]);

//    new_path_x.push_back(car.getX());
//    new_path_y.push_back(car.getY());

    assert(new_path_x.size() == new_path_y.size());
    cout << endl;
  }
  cout << endl;

  PathCartesian path = {
      .x_points = new_path_x,
      .y_points = new_path_y
  };

  vector<PathCartesian> paths;
  paths.push_back(path);
  return paths;
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

  Eigen::MatrixXd matrix_c = matrix_a_inverse * matrix_b;

  vector<double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < matrix_c.size(); i++)  {
    result.push_back(matrix_c.data()[i]);
  }

  return result;
}

tk::spline PathGenerator::fitSpline(int target_lane, double target_velocity) {
  PathCartesian previous_path = car_.getCurrentPath();

  double remaining_previous_steps_count = previous_path.x_points.size();
  vector<double> spline_points_x, spline_points_y;

  double ref_x = car_.getX();
  double ref_y = car_.getY();
  double ref_angle = car_.getAngle();

  const int max_path_size = 150;

  /**
   * Usually there will be around 47 remaining steps from previous iteration. However, we need at least 2 points
   * to always be there, so we make sure to fill it up with car current and previous estimated position
   **/
  double last_wp_s;
  if (remaining_previous_steps_count < 2) {
    double prev_car_x = car_.getX() - cos(car_.getYaw());
    double prev_car_y = car_.getY() - sin(car_.getYaw());

    spline_points_x.push_back(prev_car_x);
    spline_points_x.push_back(car_.getX());

    spline_points_y.push_back(prev_car_y);
    spline_points_y.push_back(car_.getY());

    last_wp_s = car_.getS();
  }
  else {
    ref_x = previous_path.x_points[remaining_previous_steps_count-1];
    ref_y = previous_path.y_points[remaining_previous_steps_count-1];

    double ref_x_prev = previous_path.x_points[remaining_previous_steps_count-2];
    double ref_y_prev = previous_path.y_points[remaining_previous_steps_count-2];
    ref_angle = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    spline_points_x.push_back(ref_x_prev);
    spline_points_x.push_back(ref_x);

    spline_points_y.push_back(ref_y_prev);
    spline_points_y.push_back(ref_y);

    auto last_frenet = getFrenet(ref_x, ref_y, ref_angle, map_.x_waypoints, map_.y_waypoints);
    last_wp_s = last_frenet[0];
  }

  int wp_increment = (rand()%40) + 15;
  const int number_of_wp_to_use = 5;
  for (int i = 0; i < number_of_wp_to_use; i++) {
    vector<double> next_wp = getXY(last_wp_s + wp_increment * (i+1), LaneToD(target_lane), map_.s_waypoints, map_.x_waypoints, map_.y_waypoints);
    spline_points_x.push_back(next_wp[0]);
    spline_points_y.push_back(next_wp[1]);
  }

  //Transform spline points into local coordinate
  for (int i = 0; i < spline_points_x.size(); i++) {
    double shift_x = spline_points_x[i] - ref_x;
    double shift_y = spline_points_y[i] - ref_y;

    spline_points_x[i] = shift_x * cos(0 - ref_angle) - shift_y * sin(0 - ref_angle);
    spline_points_y[i] = shift_x * sin(0 - ref_angle) + shift_y * cos(0 - ref_angle);
  }

  tk::spline spl;
  spl.set_points(spline_points_x, spline_points_y);

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