#include "vehicle.hpp"


bool operator<(const OtherVehicle &lhs, const OtherVehicle &rhs)
{
  return lhs.id < rhs.id;
}

void OtherVehicle::Init(vector<double> sf)
{
  id = sf[0];
  x = sf[1];
  y = sf[2];
  vx = sf[3];
  vy = sf[4];
  s = sf[5];
  d = sf[6];
}

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

  UpdateNearestVehicles();
  
}

void Vehicle::UpdateNearestVehicles()
{
  vector<double> lanec = {2, 6, 10};
  vector<double> nearest_front_dist = {10000, 10000, 10000};
  vector<double> nearest_rear_dist = {10000, 10000, 10000};

  for (auto sf : sensor_fusion) {
    OtherVehicle ov;
    ov.Init(sf);

    double id = sf[0];
    double x = sf[1];
    double y = sf[2];
    double vx = sf[3];
    double vy = sf[4];
    double s = sf[5];
    double d = sf[6];
    
    double dist = car_s - s;

    int lane_idx = 0;
    for (double c : lanec) {
      if (c - 2 < d && d <= c + 2) {
        auto prev_dist_f = nearest_front_dist[lane_idx];
        auto prev_dist_r = nearest_rear_dist[lane_idx];
        if (dist > 0) {
          if (dist < prev_dist_f) {
            nearest_front_dist[lane_idx] = dist;
            nearest_vehicles_front[lane_idx] = ov;
          }
        } else {
          dist = fabs(dist);
          if (dist < prev_dist_r) {
            nearest_rear_dist[lane_idx] = dist;
            nearest_vehicles_rear[lane_idx] = ov;
          }
        }
      }
      ++lane_idx;
    }
  }
}
