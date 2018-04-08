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
#include "helper.hpp"
#include "vehicle.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * helper::pi() / 180; }
double rad2deg(double x) { return x * 180 / helper::pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = min(2 * helper::pi() - angle, angle);

  if (angle > helper::pi() / 4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0)
  {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

vector<double> distanceInFront(double my_s, int my_lane, vector<vector<double>> sensor_fusion) {
  double dist = 10000.;
  double v = 10000;
  double target_s = 2. + my_lane * 4.;
  for (vector<double> veh : sensor_fusion) {
    double veh_d = veh[6];
    if (target_s - 2. < veh_d && veh_d <= target_s + 2.) { // Same lane.
      double veh_s = veh[5];
      double veh_vx = veh[3];
      double veh_vy = veh[4];
      double veh_v = sqrt(veh_vx * veh_vx + veh_vy * veh_vy);
      double dist_veh = veh_s - my_s;
      if (dist_veh > 0 && dist_veh < dist) {
        dist = dist_veh;
        v = veh_v;
      }
    }
  }
  return {dist, v};
}

bool leftOpen(int my_s, int my_lane, vector<vector<double>> sensor_fusion) {
  if (my_lane == 0) {
    return false;
  }
  double target_s = 2. + (my_lane - 1) * 4.;
  double closest_dist_front = 10000;
  double closest_dist_rear = 10000;
  for (vector<double> veh : sensor_fusion) {
    double veh_d = veh[6];
    if (target_s-2. < veh_d && veh_d <= target_s+2.) { // Car in left lane.
      double veh_s = veh[5];
      double dist_veh = veh_s - my_s;
      if (dist_veh > 0 && dist_veh < closest_dist_front) {
        closest_dist_front = dist_veh;
      } else if (dist_veh > closest_dist_rear) {
        closest_dist_rear = fabs(dist_veh);
      }
    }
  }
  return (closest_dist_front > 50 && closest_dist_rear > 30);
}

bool rightOpen(int my_s, int my_lane, vector<vector<double>> sensor_fusion) {
  if (my_lane == 2) {
    return false;
  }
  double target_s = 2. + (my_lane + 1) * 4.;
  double closest_dist_front = 10000;
  double closest_dist_rear = 10000;
  for (vector<double> veh : sensor_fusion) {
    double veh_d = veh[6];
    if (target_s-2. < veh_d && veh_d <= target_s+2.) { // Car in left lane.
      double veh_s = veh[5];
      double dist_veh = veh_s - my_s;
      if (dist_veh > 0 && dist_veh < closest_dist_front) {
        closest_dist_front = dist_veh;
      } else if (dist_veh > closest_dist_rear) {
        closest_dist_rear = fabs(dist_veh);
      }
    }
  }
  return (closest_dist_front > 50 && closest_dist_rear > 30);
}

bool checkTooClose(double distanceInFront) {
  if (0 < distanceInFront && distanceInFront < 30) {
    return true;
  }
  return false;
}

// FIXME:
double acceleration(double dist, double v_diff) {
  if (v_diff == 0) {
    v_diff = 0.01;
  }
  double param = dist / v_diff;
  double sigm = 1. / (1. + exp(-1. * param));
  return 0.3 * sigm;
}

void printSensorFusion(vector<vector<double>> sensor_fusion) {
  int i = 0;
  for (auto veh : sensor_fusion) {
    int j = 0;
    for (auto val : veh) {
      cout << i << "veh[" << j << "]: " << val << " ";
      ++j;
    }
    ++i;
  }
  cout << endl;
}

int main()
{
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
  while (getline(in_map_, line))
  {
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

  int lane = 1;
  double ref_vel = 0;
  const double target_vel = 49.5;

  Vehicle vehicle;
  vehicle.map_waypoints_x = map_waypoints_x;
  vehicle.map_waypoints_y = map_waypoints_y;
  vehicle.map_waypoints_s = map_waypoints_s;
  vehicle.map_waypoints_dx = map_waypoints_dx;
  vehicle.map_waypoints_dy = map_waypoints_dy;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane, &ref_vel, target_vel, &vehicle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                                                        uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          vehicle.Update(j);

          json msgJson;

          msgJson["next_x"] = vehicle.next_vals_x;
          msgJson["next_y"] = vehicle.next_vals_y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
