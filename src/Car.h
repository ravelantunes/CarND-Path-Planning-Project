//
// Created by ravelantunes on 10/21/17.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include "Map.h"
#include "PathStructs.h"
#include "ControlState.h"

struct CarPosition {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double angle;
  double speed;
};

class Car {

private:
  double x_;
  double y_;
  double yaw_;
  double angle_;
  double s_;
  double d_;
  double speed_;
  double target_lane_;
  ControlState control_state_;

  Map map_;

  Path currentPath_;

public:
  Car();

  virtual ~Car();

  /**
   * Called to update the car state
   *
   * @param updatedCarValue
   */
  void update(const std::vector<double> &updatedCarValue);

  double getX() const;
  double getY() const;
  double getAngle() const;
  double getYaw() const;
  double getTargetLane() const;

  double getS() const;
  double getD() const;

  //Local coordinate getters
  double getLocalX() const;
  double getLocalY() const;
  double getLocalS() const;
  double getLocalD() const;


  void setMap(const Map map);
  void setTargetLane(const double target_lane);
  void setControlState(const ControlState controlState);

  void update(const CarPosition &carPosition);

  const Path &getCurrentPath() const;

  void setCurrentPath(const Path &currentPath);

  double getSpeed() const;

  double getAcceleration() const;

  double getSpeedInMeters();

  ControlState getControlState() const;

  void printState();
};


#endif //PATH_PLANNING_CAR_H
