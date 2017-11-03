#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "ControlState.h"
#include "Map.h"
#include "Car.h"
#include "PathStructs.h"
#include "PathGenerator.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

struct SensorFusionData {
  int id;
  double x;
  double y;
  double x_speed;
  double y_speed;
  double s;
  double d;
};

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

const int lane_size = 4;
int LaneToD(int lane) {
    return (lane_size/2) + (lane - 1) * lane_size;
}

int DToLane(double d) {
  return (d / lane_size) + 1;
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<double> GlobalToCarCoordinates(double global_x, double global_y, double car_x, double car_y, double car_angle) {
    const double relative_x = global_x - car_x;
    const double relative_y = global_y - car_y;
    const double translated_x = relative_x * cos(0-car_angle) - relative_y * sin(0-car_angle);
    const double translated_y = relative_x * sin(0-car_angle) + relative_y * cos(0-car_angle);
    return {translated_x, translated_y};
}

vector<double> CarToGlobalCoordinates(double global_x, double global_y, double car_x, double car_y, double car_angle) {
    const double translated_x = car_x * cos(car_angle) - car_y * sin(car_angle) + global_x;
    const double translated_y = car_x * sin(car_angle) + car_y * cos(car_angle) + global_y;
    return {translated_x, translated_y};
}

/**
 * Based on a path, returns another path in globla Frenet coordinates
 *
 * @param car
 * @param path
 * @param map
 * @return
 */
PathFrenet ConvertXYPathToFrenet(Car &car, PathCartesian &path, Map &map) {
  //Calculate path to frenet
  double last_x = car.getX();
  double last_y = car.getY();

  vector<double> s_points, d_points;
  for (int i = 0; i < path.x_points.size(); i++) {

    double delta_x = path.x_points[i] - last_x;
    double delta_y = path.y_points[i] - last_y;
    double car_angle = atan2(delta_y, delta_x);

    auto frenet_point = getFrenet(path.x_points[i], path.y_points[i], car_angle, map.x_waypoints, map.y_waypoints);
    s_points.push_back(frenet_point[0]);
    d_points.push_back(frenet_point[1]);
  }

  return {s_points, d_points};
}


const double CAR_WIDTH = 4.0;
const double CAR_LENGTH = 8.0;

bool CheckIfPathIsFeasible(Car &car, PathCartesian &path, vector<SensorFusionData> &sensor_fusion, Map &map) {
//  cout << "--- STARTING NEW ESTIMATION ---" << endl;
  auto path_in_frenet = ConvertXYPathToFrenet(car, path, map);

  //Check for car overlaps on each step of frenet path. Frenet points here are global frenets
  for (int s_points_iterator = 0; s_points_iterator < path_in_frenet.s_points.size(); s_points_iterator++) {
    const double s_point = path_in_frenet.s_points[s_points_iterator];
    const double d_point = path_in_frenet.d_points[s_points_iterator];

    //If at any point d is negative, or off road, that's an immediate no
    if (d_point < 0 || d_point > 12) {
      return false;
    }

//    cout << "(" << s_points_iterator << ")" << "estimating on: " << s_point << ", " << d_point << endl;

    //Iterate each detected car from sensor fusion data
    for (int i = 0; i < sensor_fusion.size(); i++) {
      SensorFusionData detected_car = sensor_fusion[i];

      // First, test if the car d positions overlap. We do that first since we can't predict if the car will change
      // lanes, so only care about cars on the same lane of the current step. That way, we save some cycles from the
      // s position prediction
      if (detected_car.d < d_point + 2 && detected_car.d > d_point - 2) {

        //Estimate detected car position at the step s_points_iterator
        const double estimated_car_speed = sqrt(pow(detected_car.x_speed, 2) + pow(detected_car.y_speed, 2));
        const double estimated_detected_s = detected_car.s + (.02 * estimated_car_speed * s_points_iterator);

//        cout << " - detected car on " << estimated_detected_s << ", " << detected_car.d << endl;

        const double distance = estimated_detected_s - car.getS();

        //Check if they overlap on the s position
        if (s_point > estimated_detected_s - 10 && s_point < estimated_detected_s + 5) {
          return false;
        }
      }
    }
  }
  return true;
}

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

const double WEIGHT_DISTANCE_TO_GOAL = 1.0;
const double WEIGHT_PENALIZE_LANE_CHANGE = 1.0;
const double WEIGHT_DELTA_FROM_TARGET_SPEED = 1.0;
const double WEIGHT_PENALIZE_IN_BETWEEN_LANE = 5.0;
const double WEIGHT_PENALIZE_CLOSE_TO_CAR = 1.0;
const double WEIGHT_TARGET_VELOCITY = 1.0;

double CalculateCostFunction(Car &car, PathCartesian &path, vector<SensorFusionData> &sensor_fusion, Map &map, bool log) {

  PathFrenet path_in_frenet = ConvertXYPathToFrenet(car, path, map);

  double cost = 0;

  double last_s_in_frenet = path_in_frenet.s_points[path_in_frenet.s_points.size()-1];
  double distance_traveled = car.getS() - last_s_in_frenet;

  double last_d_in_frenet = path_in_frenet.d_points[path_in_frenet.d_points.size()-1];
  double lane_change = abs(last_d_in_frenet - car.getD());

  double delta_from_target_speed = pow(60 - car.getSpeed(), 2);

  double calculate_between_lane = fmod(car.getS() - 2.0, 2.0);


  distance_traveled *= WEIGHT_DISTANCE_TO_GOAL;
  lane_change *= WEIGHT_PENALIZE_LANE_CHANGE;
  delta_from_target_speed *= WEIGHT_DELTA_FROM_TARGET_SPEED;
  calculate_between_lane *= WEIGHT_PENALIZE_IN_BETWEEN_LANE;

  //Add cost to change lane
  cost += distance_traveled;
  cost += lane_change;
  cost += delta_from_target_speed;
  cost += calculate_between_lane;

  if (log) {
    cout << "COST: " << cost << "  --  ";
    cout << "    distance to goal: " << distance_traveled;
    cout << "    lane change: " << lane_change;
    cout << "    speed target("<<car.getSpeed()<<"): " << delta_from_target_speed;
    cout << "    between lane: " << calculate_between_lane;

    cout << endl;
  }

  return cost;
}



int main() {
  uWS::Hub h;

  Car car;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&car, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    clock_t tStart = clock();

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          const int remaining_previous_steps_count = previous_path_x.size();

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
          PathCartesian previous_path = {previous_path_x, previous_path_y};
          Map map = {map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy};

          //Update the car position with the last received values from socket
          CarPosition car_position = {j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], deg2rad(j[1]["yaw"]), j[1]["speed"]};
          car.update(car_position);
          car.setCurrentPath(previous_path);

          //Parse the list of sensor fusion data
          vector<SensorFusionData> sensor_fusion_data;
          for (int i = 0; i < sensor_fusion.size(); i++) {
            SensorFusionData detected_car = {sensor_fusion[i][0], sensor_fusion[i][1], sensor_fusion[i][2],
                                             sensor_fusion[i][3], sensor_fusion[i][4], sensor_fusion[i][5],
                                             sensor_fusion[i][6]};
            sensor_fusion_data.push_back(detected_car);
          }

          vector<PathCartesian> paths = GeneratePaths(KEEP_LANE, car, map);
//          paths.push_back(GeneratePath(car, DToLane(car.getD()), 10, map));


          //Filter out not feasible paths
          vector<PathCartesian> feasible_paths;
          copy_if(paths.begin(), paths.end(), back_inserter(feasible_paths), [&car, &sensor_fusion_data, &map](PathCartesian p){return CheckIfPathIsFeasible(car, p, sensor_fusion_data, map); });
//          cout << "number of feasible paths: " << feasible_paths.size() << endl;

          //Get best path, first by iterating each path and caching the cost
          for (PathCartesian &path : feasible_paths) {
            const double cost = CalculateCostFunction(car, path, sensor_fusion_data, map, false);
            path.cost = cost;
          }

          PathCartesian best_path;

          sort(feasible_paths.begin(), feasible_paths.end(),
               [](const PathCartesian &a, const PathCartesian &b) -> bool
               {
                 return a.cost < b.cost;
               });

          if (feasible_paths.size() == 0) {
            cout << "NO FEASIBLE PATHS TO TAKE!" << endl;
          }

          best_path = feasible_paths[0];
          CalculateCostFunction(car, best_path, sensor_fusion_data, map, true);

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = best_path.x_points;
          msgJson["next_y"] = best_path.y_points;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }

      const double processing_time = (double)(clock() - tStart)/CLOCKS_PER_SEC * 1000;
      printf("Time taken: %.4f ms\n", processing_time);
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
