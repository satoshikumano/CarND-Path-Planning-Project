#ifndef PPP_HELPER
#define PPP_HELPER

#include <math.h>
#include <vector>

using namespace std;

namespace helper {

  constexpr double pi() { return M_PI; }

  // Transform from Frenet s,d coordinates to Cartesian x,y
  vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

}

#endif