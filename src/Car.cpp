//
// Created by ravelantunes on 10/21/17.
//

#include <vector>
#include <iostream>
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

const PathCartesian &Car::getCurrentPath() const {
  return currentPath_;
}

void Car::setCurrentPath(const PathCartesian &currentPath) {
  this->currentPath_ = currentPath;
//  Car::currentPath_ = currentPath_;
}

void Car::setMap(const Map map) { map_ = map; }

double Car::getX() const { return x_; }

double Car::getY() const { return y_; }

double Car::getAngle() const { return angle_; }

double Car::getYaw() const { return yaw_; }

double Car::getS() const { return s_; }

double Car::getD() const { return d_; }

double Car::getSpeed() const { return speed_; }

void Car::printState() {
  std::cout << "pos cartesian: " << getX() << ", " << getY();
  std::cout << "  frenet: " << getS() << ", " << getD();
  std::cout << "  speed: " << getSpeed();
  std::cout << "  angle: " << getAngle();
  std::cout << std::endl;
}
