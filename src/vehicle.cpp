#include "vehicle.hpp"
#include "spline.h"
#include "helper.hpp"

bool operator<(const OtherVehicle &lhs, const OtherVehicle &rhs)
{
  return lhs.id < rhs.id;
}

void OtherVehicle::Init(vector<double> sf)
{
  cout << "OtherVehicle::Init" << endl;
  id = sf[0];
  x = sf[1];
  y = sf[2];
  vx = sf[3];
  vy = sf[4];
  s = sf[5];
  d = sf[6];
}

Vehicle::Vehicle() {
  cout << "Vehicle::Vehicle" << endl;
  ref_vel = 0;
  too_slow = false;
  state = VehicleState::KL;
}

void Vehicle::Update(json j)
{
  cout << "Vehicle::Update" << endl;
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
  if (state == KL) {
    OtherVehicle vehicle_front = nearest_vehicles_front[lane];
    double dist = car_s - vehicle_front.s;
    bool tooClose = dist < 30;
    if (tooClose && ref_vel > 0) {
      ref_vel -= .224;
    } else {
      if (ref_vel < target_vel)
        ref_vel += .224;
      if (ref_vel >= target_vel)
        ref_vel -= .224;
    }
    if (tooClose) {
      if (LeftOpen()) {
        state = LCL;
        lane -= 1;
      } else if (RightOpen()) {
        state = LCR;
        lane += 1;
      }
    }
    GenerateKLTrajectories();
    return;
  }
  if (state == LCL || state == LCR) {
    GenerateKLTrajectories();
    if (fabs(car_d - 2. + 4. * lane) < 0.05) {
      state = KL;
    }
  }
  cout << "nx: ";
  for (auto nx : next_vals_x) {
    cout << nx << " ";
  }
  cout << endl;
  cout << "ny: ";
  for (auto ny : next_vals_y) {
    cout << ny << " ";
  }
  cout << endl;
}

void Vehicle::UpdateNearestVehicles()
{
  cout << "Vehicle::UpdateNearestVehicles" << endl;
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
    
    double dist = s - car_s;

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

bool Vehicle::LeftOpen() {
  cout << "Vehicle::LeftOpen" << endl;
  if (lane == 0) {
    return false;
  }
  auto veh_left_front = nearest_vehicles_front[lane-1];
  auto veh_left_front_dist = veh_left_front.s - car_s;

  auto veh_left_rear = nearest_vehicles_rear[lane-1];
  auto veh_left_rear_dist = car_s - veh_left_front.s;

  return (veh_left_front_dist > 50 && veh_left_rear_dist > 30);
}

bool Vehicle::RightOpen() {
  cout << "Vehicle::RightOpen" << endl;
  if (lane == 2) {
    return false;
  }
  auto veh_left_front = nearest_vehicles_front[lane+1];
  auto veh_left_front_dist = veh_left_front.s - car_s;

  auto veh_left_rear = nearest_vehicles_rear[lane-1];
  auto veh_left_rear_dist = car_s - veh_left_front.s;

  return (veh_left_front_dist > 50 && veh_left_rear_dist > 30);
}

void Vehicle::GenerateKLTrajectories()
{
  cout << "Vehicle::GenerateKLTrajectories" << endl;
  int prev_size = previous_path_x.size();
  vector<double> ptsx;
  vector<double> ptsy;
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = car_yaw;
  
  if (prev_size < 2)
  {
    cout << "Vehicle::GenerateKLTrajectories2" << endl;
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  else
  {
    cout << "Vehicle::GenerateKLTrajectories3" << endl;
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  vector<double> next_wp0;
  vector<double> next_wp1;
  vector<double> next_wp2;

  if (state == KL) {
    cout << "Vehicle::GenerateKLTrajectories4" << endl;
    vector<double> next_wp0 = helper::getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = helper::getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = helper::getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);
    cout << "Vehicle::GenerateKLTrajectories4-2" << endl;
  }
  else if (state == LCL || state == LCR) {
    cout << "Vehicle::GenerateKLTrajectories5" << endl;
    vector<double> next_wp0 = helper::getXY(car_s + 30, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = helper::getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);    

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp2[1]);
  }


  for (int i = 0; i < ptsx.size(); ++i)
  {
    cout << "Vehicle::GenerateKLTrajectories6" << endl;
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  for (int i = 0; i < previous_path_x.size(); ++i)
  {
    cout << "Vehicle::GenerateKLTrajectories7" << endl;
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 90.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0;

  for (int i = 0; i <= 50 - previous_path_x.size(); ++i)
  {
    cout << "Vehicle::GenerateKLTrajectories8" << endl;
    double N = target_dist / (0.02 * ref_vel / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);
    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  cout << "Vehicle::GenerateKLTrajectories9" << endl;
  next_vals_x = next_x_vals;
  next_vals_y = next_y_vals;
}