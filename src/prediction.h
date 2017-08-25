#ifndef PREDICTION_H
#define PREDICTION_H

#include <iostream>
#include <math.h>

namespace pathplanner {
  using namespace std;

  struct prediction {
    double s;
    double d;
    double vx;
    double vy;

    bool is_in_lane(int lane) {
      return d < (4.0 * (lane + 1)) && d >(4.0 * lane);
    }

    double get_velocity() {
      return sqrt(vx*vx + vy*vy);
    }

    void display() {
      cout << "s: " << s << " d: " << d << " vx: " << vx << " vy: " << vy << endl;
    }
  };
}

#endif