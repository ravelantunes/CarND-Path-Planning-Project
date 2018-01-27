#include <fstream>
#include <uWS/uWS.h>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "ControlState.h"
#include "Map.h"
#include "Car.h"
#include "PathStructs.h"
#include "PathGenerator.h"
#include "SensorFusionData.h"
//#include "Helpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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

int main() {
  uWS::Hub h;

  Car car;
  car.setControlState(KEEP_LANE);
  PathGenerator pathGenerator;

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

  h.onMessage([&pathGenerator, &car, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          Path previous_path = {previous_path_x, previous_path_y};

          Map map = {map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy};

          //Update the car position with the last received values from socket
          double yaw_in_radians = ((double)j[1]["yaw"]) * M_PI / 180;
          CarPosition car_position = {j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], yaw_in_radians, j[1]["speed"]};
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

          pathGenerator.setState(car, map, sensor_fusion_data);
          vector<Path> paths = pathGenerator.generatePaths();

          //Select best path and set ref s,d as the last s,d of path
          Path *best_path = nullptr;
          for (int i = 0; i < paths.size(); i++) {
            if (best_path == nullptr) {
              best_path = &paths[i];
              continue;
            }

            if (paths[i].cost < best_path->cost) {
              best_path = &paths[i];
            }
          }

          pathGenerator.setLastRefs(best_path->last_s, best_path->last_d);
          if (best_path->next_state) {
            car.setControlState(best_path->next_state);
          }


          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = best_path->x_points;
          msgJson["next_y"] = best_path->y_points;

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
