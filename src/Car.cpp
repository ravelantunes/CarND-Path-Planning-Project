//
// Created by ravelantunes on 10/21/17.
//

#include <vector>
#include <iostream>
#include <cmath>
#include "Car.h"

Car::Car() {

}

Car::~Car() {

}

void Car::update(const CarPosition &carPosition) {
  x_ = carPosition.x;
  y_ = carPosition.y;
  speed_ = carPosition.speed;
  yaw_ = carPosition.yaw;
  s_ = carPosition.s;
  d_ = carPosition.d;
  speed_ = carPosition.speed;
  angle_ = carPosition.angle;
}

const Path &Car::getCurrentPath() const {
  return currentPath_;
}

void Car::setCurrentPath(const Path &currentPath) {
  currentPath_ = currentPath;
}

void Car::setMap(const Map map) { map_ = map; }

double Car::getX() const { return x_; }

double Car::getY() const { return y_; }

double Car::getAngle() const { return angle_; }

double Car::getYaw() const { return yaw_; }

double Car::getS() const { return s_; }

double Car::getD() const { return d_; }

double Car::getSpeed() const { return speed_; }

void Car::setTargetLane(double target_lane) { target_lane_ = target_lane; }

double Car::getTargetLane() const { return target_lane_; }

void Car::setControlState(const ControlState controlState) {
  this->control_state_ = controlState;
}
ControlState Car::getControlState() const { return control_state_; }

double Car::getAcceleration() const {
  if (getCurrentPath().x_points.size() < 2) {
    return 0.0;
  }

  double x1 = getCurrentPath().x_points[0];
  double x2 = getCurrentPath().x_points[1];
  double y1 = getCurrentPath().y_points[0];
  double y2 = getCurrentPath().y_points[1];

  return std::sqrt(std::pow(y2-y1, 2) + std::pow(x2-x1, 2));
}

double Car::getSpeedInMeters() {
  return 1609.34 * speed_ / 60 / 60;
}

void Car::printState() {
  std::cout << "pos cartesian: " << getX() << ", " << getY();
  std::cout << "  frenet: " << getS() << ", " << getD();
  std::cout << "  speed: " << getSpeed();
  std::cout << "  angle: " << getAngle();
  std::cout << "  steps left: " << getCurrentPath().x_points.size();
  std::cout << std::endl;
}
