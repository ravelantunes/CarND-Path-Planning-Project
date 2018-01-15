//
// Created by ravelantunes on 1/8/18.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

static const int LANE_SIZE = 4;
static const double CAR_WIDTH = 4.0;
static const double CAR_LENGTH = 8.0;

static const double SECONDS_BETWEEN_UPDATES = 0.05;
static const double UPDATES_PER_SECOND = 1.0/SECONDS_BETWEEN_UPDATES;
static const double METERS_IN_MILE = 1609.34;
static const double MPH_SPEED_LIMIT = 50; //in miles
static const double KM_SPEED_LIMIT = MPH_SPEED_LIMIT * 1.60934;
static const double MPS_LIMIT = KM_SPEED_LIMIT/60/60*1000;
static const double MAX_DIST_PER_SEC = MPS_LIMIT/50;
static const double MAX_ACCELERATION = 10.0;
static const double CLOSEST_CAR_THRESHOLD = 30;


#endif //PATH_PLANNING_CONSTANTS_H
