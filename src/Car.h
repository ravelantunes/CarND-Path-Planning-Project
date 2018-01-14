//
// Created by ravelantunes on 10/21/17.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include "Map.h"
#include "PathStructs.h"

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

  Map map_;

  PathCartesian currentPath_;

public:
  Car();

  virtual ~Car();

  /**
   * Called to update the car state
   *
   * @param updatedCarValue
   */
  void update(const std::vector<double> &updatedCarValue);

  int getCurrentLane() const;

  double getX() const;
  double getY() const;
  double getAngle() const;
  double getYaw() const;

  double getS() const;
  double getD() const;

  //Local coordinate getters
  double getLocalX() const;
  double getLocalY() const;
  double getLocalS() const;
  double getLocalD() const;


  void setMap(const Map map);

  void update(const CarPosition &carPosition);

  const PathCartesian &getCurrentPath() const;

  void setCurrentPath(const PathCartesian &currentPath);

  double getSpeed() const;

  double getAcceleration() const;

  double getSpeedInMeters();

  void printState();
};


#endif //PATH_PLANNING_CAR_H
