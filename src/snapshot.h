#ifndef SNAPSHOT_H
#define SNAPSHOT_H

namespace pathplanner {
  using namespace std;

  enum class CarState { CS = 0, KL = 1, PLCL = 2, PLCR = 3, LCL = 4, LCR = 5 };

  struct snapshot {
    double x;
    double y;
    double dx;
    double dy;
    double ddx;
    double ddy;
    double s;
    double d;
    double yaw;
    int lane;
    int proposed_lane;
    CarState state;
    double ref_vel;


    double get_speed() {
      return sqrt(dx*dx + dy*dy);
    }

    double get_acceleration() {
      return sqrt(ddx*ddx + ddy*ddy);
    }

    void display() {
      cout << "snapshot: x " << x << " y " << y << " dx " << dx << " dy "
        << dy << " ddx " << ddx << " ddy " << ddy << " s " << s << " d " << d << " yaw " << yaw
        << " lane " << lane << endl;
    }
  };
}

#endif
