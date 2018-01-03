//
// Created by ravelantunes on 1/2/18.
//

#ifndef PATH_PLANNING_SENSORFUSIONDATA_H
#define PATH_PLANNING_SENSORFUSIONDATA_H

struct SensorFusionData {
  int id;
  double x;
  double y;
  double x_speed;
  double y_speed;
  double s;
  double d;
};

#endif //PATH_PLANNING_SENSORFUSIONDATA_H
