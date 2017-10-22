//
// Created by ravelantunes on 10/21/17.
//

#include <vector>
#include "Car.h"

void Car::update(const std::vector<double> &updatedCarValue) {
  x_ = updatedCarValue[0];
  y_ = updatedCarValue[1];
  speed_ = updatedCarValue[2];
  yaw_ = updatedCarValue[3];
  s_ = updatedCarValue[4];
  d_ = updatedCarValue[5];
}

void Car::setMap(const Map map) {
  map_ = map;
}

double Car::getX_() const {
  return x_;
}

double Car::getY_() const {
  return y_;
}

double Car::getSpeed_() const {
  return speed_;
}

double Car::getAngle_() const {
  return angle_;
}
