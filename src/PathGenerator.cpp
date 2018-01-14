//
// Created by Ravel Antunes on 11/3/17.
//
#include "spline.h"
#include "PathStructs.h"
#include "PathGenerator.h"
#include "SensorFusionData.h"
#include <vector>
#include <iostream>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include <iomanip>
#include "Constants.h"
#include "Helpers.h"


using namespace std;

bool change_lane = false;

vector<PathCartesian> PathGenerator::generatePaths(ControlState controlState, Car &car, Map &map, vector<SensorFusionData> &sensor_fusion_data) {

  //Print car state
//  car.printState();

  PathCartesian previous_path = car.getCurrentPath();

  //Copy the remaining steps of the previous path into the new paths
  vector<double> new_path_x, new_path_y;
  for (int i = 0; i < previous_path.x_points.size(); i++) {
    new_path_x.push_back(previous_path.x_points[i]);
    new_path_y.push_back(previous_path.y_points[i]);
  }

  int previous_path_size = previous_path.x_points.size();

  //Length in seconds that maneuver will take
  double number_of_seconds = default_path_length_ / UPDATES_PER_SECOND;
;
  //Search for the closest car on the same lane, so we can follow
  double closest_car_distance = 1000.0;

  //Estimate ego car position in 2 seconds (safe distance to keep from next car)
  double closest_car_speed = 1000;

  //Iterate each detected car from sensor fusion data
  for (int i = 0; i < sensor_fusion_data.size(); i++) {
    SensorFusionData detected_car = sensor_fusion_data[i];

    // First, test if the car d positions overlap. We do that first since we can't predict if the car will change
    // lanes, so only care about cars on the same lane of the current step. That way, we save some cycles from the
    // s position prediction
    if (detected_car.d < car.getD() + 2 && detected_car.d > car.getD() - 2) {

      //Estimate detected car position at the step s_points_iterator
      const double estimated_car_speed = sqrt(pow(detected_car.x_speed, 2) + pow(detected_car.y_speed, 2));
      const double estimated_detected_s = detected_car.s + estimated_car_speed ;

      double distance = estimated_detected_s - car.getS();

      if (distance > 0 && distance < closest_car_distance) {
        closest_car_distance = distance;
        closest_car_speed = estimated_car_speed;
      }

    }
  }
//  cout << "Closest car is at " << closest_car_distance << endl;

  double distance_goal = MAX_DIST_PER_SEC;
  double speed_goal = MPS_LIMIT/50;
  if (closest_car_distance < CLOSEST_CAR_THRESHOLD) {
    distance_goal = (closest_car_speed - 1) /50;
    speed_goal = closest_car_speed/50;
  }

  int best_lane = 6;
  if (closest_car_distance < 70) {
    best_lane = LaneToD(testBestLane(car, sensor_fusion_data));
  }

  speed_goal = min((car.getSpeedInMeters() + 10)/50, speed_goal);

  vector<double> start_s = {0, car.getSpeedInMeters()/50, car.getAcceleration()};
  vector<double> end_s = {distance_goal, speed_goal, 0.0};

  vector<double> start_d = {0, 0, 0};
  vector<double> end_d = {(best_lane-car.getD())/150, 0, 0};

//  cout << "car d : " << car.getD() << "   target lane:" << rounded_car_d << endl;

  vector<double> s_poly_coeffs = fitPolynomial(start_s, end_s, number_of_seconds);
  vector<double> d_poly_coeffs = fitPolynomial(start_d, end_d, number_of_seconds);

  //Iterate to create paths points until it fills up the expected number of paths
  for (auto i = new_path_x.size(); i < default_path_length_; i++) {
    //Step to be evaluated
    double step = (i) * (number_of_seconds / default_path_length_);

    ref_s_ += EvaluatePoly(s_poly_coeffs, step);
    ref_d_ += EvaluatePoly(d_poly_coeffs, step);
    double s_point = ref_s_;
    double d_point = ref_d_;

    cout << "car s: " << car.getS() << "  refD: " << ref_d_<< endl;
    if (car.getS() < 150) {
      d_point = 6;
    }

    vector<double> xy_points = getXY(s_point, d_point, map.s_waypoints, map.x_waypoints, map.y_waypoints);
    double new_x = xy_points[0];
    double new_y = xy_points[1];

    new_path_x.push_back(new_x);
    new_path_y.push_back(new_y);

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

int PathGenerator::testBestLane(Car &car, vector<SensorFusionData> &sensor_fusion_data) {
  vector<double> lanes_free_length(3);

  for (int i = 0; i < sensor_fusion_data.size(); i++) {
    SensorFusionData detected_car = sensor_fusion_data[i];

    //Make sure it's a valid lane we care about
    int lane = DToLane(detected_car.d) - 1;
    if (lane < 0 || lane > 2) {
      continue;
    }

    double best_lane_distance = lanes_free_length[lane];
    double distance = detected_car.s - car.getS();
    if (distance > best_lane_distance) {
      lanes_free_length[lane] = distance;
    }
  }

//  cout << "lane1: " << lanes_free_length[0];
//  cout << "   lane2: " << lanes_free_length[1];
//  cout << "   lane3: " << lanes_free_length[2] << endl;

  //Ugly
  if (lanes_free_length[0] > lanes_free_length[1]) {
    if (lanes_free_length[0] > lanes_free_length[2]) {
      return 1;
    } else {
      return 3;
    }
  } else {
    if (lanes_free_length[1] > lanes_free_length[2]) {
      return 2;
    } else {
      return 3;
    }
  }
}

vector<double> PathGenerator::fitPolynomial(vector<double> start, vector <double> end, double T) {
  Eigen::MatrixXd matrix_a = Eigen::MatrixXd(3, 3);
  matrix_a <<   pow(T, 3),    pow(T, 4),    pow(T, 5),
              3*pow(T, 2),  4*pow(T,3),   5*pow(T, 4),
              6*T,         12*pow(T, 2), 20*pow(T, 3);

  Eigen::MatrixXd matrix_b = Eigen::MatrixXd(3, 1);
  matrix_b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * pow(T, 2)),
      end[1] - (start[1] + start[2] * T),
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