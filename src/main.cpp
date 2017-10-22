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
#include "controlstate.h"
#include "Map.h"
#include "Car.h"

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

struct CarStruct {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double angle;
    double speed;
};

struct Path {
  vector<double> x_points;
  vector<double> y_points;
  double cost;
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
 * Generate a path
 *
 * @param car
 * @return vector of vectors, where index 0 is x points and index 1 is y points
 */
Path GeneratePath(CarStruct car, int target_lane, double target_velocity, Path previous_path, Map &map) {

  const int recycle_size = 45;
  if (previous_path.x_points.size() > recycle_size) {
    previous_path.x_points.erase(previous_path.x_points.begin()+recycle_size, previous_path.x_points.end());
    previous_path.y_points.erase(previous_path.y_points.begin()+recycle_size, previous_path.y_points.end());
  }
  double remaining_previous_steps_count = previous_path.x_points.size();
  vector<double> spline_points_x, spline_points_y;
  vector<double> next_x_vals, next_y_vals;

  double ref_x = car.x;
  double ref_y = car.y;
  double ref_angle = car.angle;

  const int max_path_size = 150;

  /**
   * Usually there will be around 47 remaining steps from previous iteration. However, we need at least 2 points
   * to always be there, so we make sure to fill it up with car current and previous estimated position
   **/
  double last_wp_s;
  if (remaining_previous_steps_count < 2) {
    double prev_car_x = car.x - cos(car.yaw);
    double prev_car_y = car.y - sin(car.yaw);

    spline_points_x.push_back(prev_car_x);
    spline_points_x.push_back(car.x);

    spline_points_y.push_back(prev_car_y);
    spline_points_y.push_back(car.y);

    last_wp_s = car.s;
  }
  else {
    ref_x = previous_path.x_points[remaining_previous_steps_count-1];
    ref_y = previous_path.y_points[remaining_previous_steps_count-1];

    double ref_x_prev = previous_path.x_points[remaining_previous_steps_count-2];
    double ref_y_prev = previous_path.y_points[remaining_previous_steps_count-2];
    ref_angle = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    spline_points_x.push_back(ref_x_prev);
    spline_points_x.push_back(ref_x);

    spline_points_y.push_back(ref_y_prev);
    spline_points_y.push_back(ref_y);

    auto last_frenet = getFrenet(ref_x, ref_y, ref_angle, map.x_waypoints, map.y_waypoints);
    last_wp_s = last_frenet[0];
  }


  int wp_increment = 20;
  const int number_of_wp_to_use = 5;
  for (int i = 0; i < number_of_wp_to_use; i++) {
    vector<double> next_wp = getXY(last_wp_s + wp_increment * (i+1), LaneToD(target_lane), map.s_waypoints, map.x_waypoints, map.y_waypoints);
    spline_points_x.push_back(next_wp[0]);
    spline_points_y.push_back(next_wp[1]);
  }

  for (int i = 0; i < number_of_wp_to_use; i++) {
    vector<double> next_wp = getXY(last_wp_s + wp_increment * (i+6), LaneToD(DToLane(LaneToD(car.d))), map.s_waypoints, map.x_waypoints, map.y_waypoints);
    spline_points_x.push_back(next_wp[0]);
    spline_points_y.push_back(next_wp[1]);
  }


  //Transform spline points into local coordinate
  for (int i = 0; i < spline_points_x.size(); i++) {
    double shift_x = spline_points_x[i] - ref_x;
    double shift_y = spline_points_y[i] - ref_y;

    spline_points_x[i] = shift_x * cos(0 - ref_angle) - shift_y * sin(0 - ref_angle);
    spline_points_y[i] = shift_x * sin(0 - ref_angle) + shift_y * cos(0 - ref_angle);
  }

  tk::spline spl;
  spl.set_points(spline_points_x, spline_points_y);

  //Copy values from the previous path
  for (int i = 0; i < previous_path.x_points.size(); i++) {
    next_x_vals.push_back(previous_path.x_points[i]);
    next_y_vals.push_back(previous_path.y_points[i]);
  }

  double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;

  double N = target_dist / (.02*target_velocity/2.24);

  for (int i = 1; i <= max_path_size - next_x_vals.size(); i++) {

    double x_point = x_add_on + (target_x) / N;
    double y_point = spl(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_angle) - y_ref * sin(ref_angle);
    y_point = x_ref * sin(ref_angle) + y_ref * cos(ref_angle);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  return {next_x_vals, next_y_vals};
}


/**
 * Based on a path, returns another path in globla Frenet coordinates
 *
 * @param car
 * @param path
 * @param map
 * @return
 */
tuple<vector<double>, vector<double>> ConvertXYPathToFrenet(CarStruct &car, Path &path, Map &map) {
  //Calculate path to frenet
  double last_x = car.x;
  double last_y = car.y;

  vector<double> s_points, d_points;
  for (int i = 0; i < path.x_points.size(); i++) {

    double delta_x = path.x_points[i] - last_x;
    double delta_y = path.y_points[i] - last_y;
    double car_angle = atan2(delta_y, delta_x);

    auto frenet_point = getFrenet(path.x_points[i], path.y_points[i], car_angle, map.x_waypoints, map.y_waypoints);
    s_points.push_back(frenet_point[0]);
    d_points.push_back(frenet_point[1]);
  }
  return make_tuple(s_points, d_points);
}


const double CAR_WIDTH = 4.0;
const double CAR_LENGTH = 8.0;

double CheckIfPathIsFeasible(CarStruct &car, Path &path, vector<SensorFusionData> &sensor_fusion, Map &map) {
  auto path_in_frenet = ConvertXYPathToFrenet(car, path, map);

  vector<double> s_points = get<0>(path_in_frenet);
  vector<double> d_points = get<1>(path_in_frenet);

  //Check for car overlaps on each step of frenet path
  for (int s_points_iterator = 0; s_points_iterator < s_points.size(); s_points_iterator++) {
    const double s_point = s_points[s_point];
//    cout << s_point << ", ";
//    cout << endl;

    for (int i = 0; i < sensor_fusion.size(); i++) {
      SensorFusionData detected_car = sensor_fusion[i];

//      if (detected_car.d < path_in_frenet.[1] + 2 && detected_car.d > point_in_frenet[1] - 2) {
//
//        double estimated_car_speed = sqrt(pow(detected_car.x_speed, 2) + pow(detected_car.y_speed, 2));
//
//        detected_car.s += ((double)47) * .02 * estimated_car_speed;
//
//        double distance_to_car = detected_car.s - car.s;
//        if (detected_car.s > car.s && distance_to_car < closest_car_distance) {
//          closest_car_distance = distance_to_car;
//        }
//      }
    }
  }


//  double closest_car_distance = 10000;
//  for (int i = 0; i < sensor_fusion.size(); i++) {
//    SensorFusionData detected_car = sensor_fusion[i];
//
//    if (detected_car.d < point_in_frenet[1] + 2 && detected_car.d > point_in_frenet[1] - 2) {
//
//      double estimated_car_speed = sqrt(pow(detected_car.x_speed, 2) + pow(detected_car.y_speed, 2));
//
//      detected_car.s += ((double)47) * .02 * estimated_car_speed;
//
//      double distance_to_car = detected_car.s - car.s;
//      if (detected_car.s > car.s && distance_to_car < closest_car_distance) {
//        closest_car_distance = distance_to_car;
//      }
//    }
//  }

//  cout << "closest car on " <<  DToLane(point_in_frenet[1]) << ": " << closest_car_distance << endl;

//  return closest_car_distance;
  return 0;
}

/**
 * NOTE TO SELF: experimento with creating the same example code for waypoints on all 3 lanes,
 * and possibly farther or closer to car (about 3 waypoints per lane)
 */
double CalculateCostFunction(CarStruct &car, Path &path, vector<SensorFusionData> &sensor_fusion, Map &map) {

//  Path path_in_frenet = ConvertXYPathToFrenet(car, path, map);

  double cost = 0;
  double path_is_feasible = CheckIfPathIsFeasible(car, path, sensor_fusion, map);
  path_is_feasible = pow(path_is_feasible, 2);
  cost -= path_is_feasible;


  const int path_size = path.x_points.size();
  double delta_x = path.x_points[path_size-1] - path.x_points[path_size-2];
  double delta_y = path.y_points[path_size-1] - path.y_points[path_size-2];
  double car_angle = atan2(delta_y, delta_x);
  auto last_point_in_frenet = getFrenet(path.x_points[path_size-1], path.y_points[path_size-1], car_angle, map.x_waypoints, map.y_waypoints);
  double distance_to_goal = 6945.5 - last_point_in_frenet[0];
  cost += distance_to_goal / 5;

  //Add cost to change lane
//  cout << cost << endl;

  return cost;
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    clock_t tStart = clock();

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          int car_values[] = {j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], deg2rad(j[1]["yaw"]), j[1]["speed"]};
          vector<double> car_values_vector(car_values, car_values + 7);


          Car carObj
//          vector<double> car_values = (j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], deg2rad(j[1]["yaw"]), j[1]["speed"]);

          CarStruct car = {j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], deg2rad(j[1]["yaw"]), j[1]["speed"]};

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
          Path previous_path = {previous_path_x, previous_path_y};
          Map map = {map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy};

          vector<SensorFusionData> sensor_fusion_data;
          for (int i = 0; i < sensor_fusion.size(); i++) {
            SensorFusionData detected_car = {sensor_fusion[i][0], sensor_fusion[i][1], sensor_fusion[i][2],
                                             sensor_fusion[i][3], sensor_fusion[i][4], sensor_fusion[i][5],
                                             sensor_fusion[i][6]};
            sensor_fusion_data.push_back(detected_car);
          }

          vector<Path> paths;

          //Generate paths
          for (int lane = 1; lane <= 3; lane++) {
            Path path = GeneratePath(car, lane, 45.0, previous_path, map);
            path.cost = CalculateCostFunction(car, path, sensor_fusion_data, map);
            paths.push_back(path);
          }

          //Sort paths
          int lowest_cost_path_index = 0;
          for (int p = 0; p < paths.size(); p++) {
            Path path = paths[p];
            if (path.cost < paths[lowest_cost_path_index].cost) {
              lowest_cost_path_index = p;
            }
          }

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = paths[lowest_cost_path_index].x_points;
          msgJson["next_y"] = paths[lowest_cost_path_index].y_points;

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
//      printf("Time taken: %.4f ms\n", processing_time);
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
