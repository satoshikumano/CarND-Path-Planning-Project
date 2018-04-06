#include "vehicle.hpp"

Vehicle::Vehicle() {
  ref_vel = 0;
  too_slow = false;
  state = VehicleState::KL;
}

void Vehicle::Update(json j)
{
  car_x = j[1]["x"];
  car_y = j[1]["y"];
  car_s = j[1]["s"];
  car_d = j[1]["d"];
  car_yaw = j[1]["yaw"];
  car_speed = j[1]["speed"];
  previous_path_x = j[1]["previous_path_x"].get<vector<double>>();
  previous_path_y = j[1]["previous_path_y"].get<vector<double>>();
  sensor_fusion = j[1]["sensor_fusion"].get<vector<vector<double>>>();
}