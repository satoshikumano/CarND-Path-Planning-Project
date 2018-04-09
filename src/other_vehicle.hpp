#ifndef PPP_OTHER_VEHICLE
#define PPP_OTHER_VEHICLE

#include <math.h>
#include <vector>

using namespace std;

class OtherVehicle {
  public:
  double id;
  double x;
  double y;
  double vx;
  double vy;
  double v;
  double s;
  double d;

  void Init(vector<double> sf) {
    id = sf[0];
    x = sf[1];
    y = sf[2];
    vx = sf[3];
    vy = sf[4];
    s = sf[5];
    d = sf[6];
    v = sqrt(vx*vx + vy*vy);
  }

};

#endif