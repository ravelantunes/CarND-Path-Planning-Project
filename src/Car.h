//
// Created by ravelantunes on 10/21/17.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H


#include "Map.h"

class Car {

private:
  double x_;
  double y_;
  double speed_;
  double yaw_;
  double angle_;
  double s_;
  double d_;

  Map map_;


public:
  Car();

  virtual ~Car();

  int getCurrentLane() const;

  double getX() const;
  double getY() const;
  double getAngle() const;

  double getS() const;
  double getD() const;

  double getLocalX() const;
  double getLocalY() const;
  double getLocalS() const;
  double getLocalD() const;

  double getX_() const;

  void setX_(double x_);

  double getY_() const;

  void setY_(double y_);

  double getSpeed_() const;

  void setSpeed_(double speed_);

  double getAngle_() const;

  void setAngle_(double angle_);

  void update(const std::vector<double> &updatedCarValue);


  void setMap(const Map map);
};


#endif //PATH_PLANNING_CAR_H
