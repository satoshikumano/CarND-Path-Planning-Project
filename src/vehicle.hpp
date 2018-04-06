#ifndef PPP_VEHCILE
#define PPP_VEHCILE

#include "json.hpp"

using json = nlohmann::json;
using namespace std;

enum VehicleState {
  KL,
  LCL,
  LCR,
};

class OtherVehicle {
  public:

  double id;
  double x;
  double y;
  double vx;
  double vy;
  // Norm of vx + vy
  double v;
  double s;
  double d;

  // sf: vector<doube> in sensor_fusion verctor.
  OtherVehicle(vector<double> sf);
};

class Vehicle {
  public:

  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  int lane;
  double ref_vel;
  static constexpr double target_vel = 49.5;

  vector<double> previous_path_x;
  vector<double> previous_path_y;

  vector<vector<double>> sensor_fusion;

  VehicleState state;
  bool too_slow;

  vector<vector<double>> next_vals();

  // Key: lane no.
  map<int, OtherVehicle> nearest_vehicles_front;
  // Key: lane no.
  map<int, OtherVehicle> nearest_vehicles_rear;

  Vehicle();
  void Update(json data);

  private:
  void UpdateNearestVehicles();
};

#endif


