#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

#include "FSM.h"

namespace pathplanner {
  using namespace std;

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

  class Vehicle {

  private:
    
    double const LANE_WIDTH = 4.0;
    double const MIDDLE_LANE = LANE_WIDTH/2;
    
    //double const TOO_SHORT_DISTANCE = 15.0;
    //double const MANOEUVRE = 10;
    double const MANOEUVRE = 4;
    //double const PREDICTION_DISTANCE = 20;

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
    double ref_vel = 0.0;
    CarState state = CarState::CS;

    double get_velocity() {
      return sqrt(dx*dx + dy*dy);
    }

    int lane = 1;

    struct collider {
      bool collision; // is there a collision?
      int time; // time collision happens
    };

    bool shouldPredict() {
      return updates > 3;
    }

    int preferred_buffer = 6; // impacts "keep lane" behavior.

    int max_acceleration;

    

    Vehicle(int id);

    Vehicle(int id, double x, double y, double dx, double dy, double s, double d);

    /**
    * Destructor
    */
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

    snapshot get_snapshot();

    void restore_state_from_snapshot(snapshot snapshot);
  };

}

#endif