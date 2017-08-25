#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

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

  class Vehicle {

  private:
    double const LANE_WIDTH = 4.0;
    double const MIDDLE_LANE = LANE_WIDTH/2;
    int updates = 0;

    void update_accel(double vx, double vy, double diff);

  public:
    static double SAFE_DISTANCE;

    int id;
    double x;
    double y;
    double dx;
    double dy;
    double ddx;
    double ddy;
    double yaw;
    double s;
    double d;
    int lane = 1;

    double get_velocity();

    bool shouldPredict();

    Vehicle(int id);

    Vehicle(int id, double x, double y, double dx, double dy, double s, double d);

    virtual ~Vehicle();

    void update_params(double x, double y, double yaw, double s, double d, double speed, double diff);

    void update_yaw(double x, double y, double vx, double vy, double s, double d, double diff);

    void display();

    void increment(double t);

    prediction state_at(double t);

    bool is_in_front_of(prediction pred, int checked_lane);

    bool is_behind_of(prediction pred, int lane);

    bool is_close_to(prediction pred, int lane);

    vector<prediction> generate_predictions(int horizon = 10);
  };

}

#endif