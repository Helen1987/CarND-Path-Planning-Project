#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

namespace pathplanner {
  using namespace std;

  class Vehicle {

  private:
    double TIME_INTERVAL = 0.02;
    double LANE_WIDTH = 4;
    double MIDDLE_LANE = LANE_WIDTH/2;
    double SAFE_DISTANCE = 10.0;
    double SPEED_INCREMENT = .224;
    double MAX_SPEED = 49.5;

    double x;
    double y;
    double dx;
    double dy;
    double ddx;
    double ddy;
    double s;
    double d;

    int leading_vehicle = -1;

    struct estimate {
      string state;
      double cost;
    };

    double get_velocity() {
      return sqrt(dx*dx + dy*dy);
    }

  public:
    int id;

    double ref_vel = 0.0;
    int lane = 1;

    struct collider {
      bool collision; // is there a collision?
      int  time; // time collision happens
    };

    struct prediction {
      double s;
      double d;
      double vx;
      double vy;
    };

    struct snapshot {
      double x;
      double y;
      double dx;
      double dy;
      double ddx;
      double ddy;
      double s;
      double d;
      double ref_vel;
      double lane;
      string state;
    };

    int preferred_buffer = 6; // impacts "keep lane" behavior.

    int max_acceleration;

    string state;

    Vehicle(int id);

    Vehicle(int id, double x, double y, double dx, double dy, double s, double d);

    /**
    * Destructor
    */
    virtual ~Vehicle();

    void update_params(double x, double y, double v, double yaw, double s, double d);

    void update_state(map<int, vector<prediction>> predictions, int lanes_available);

   // void configure(vector<int> road_data);

    string display();

    void increment(int dt = 1);

    prediction state_at(int count);

    bool collides_with(Vehicle other, int at_time);

    collider will_collide_with(Vehicle other, int timesteps);

    void realize_state(map<int, vector<prediction> > predictions);

    void realize_constant_speed();

    void realize_keep_lane(map<int, vector<prediction> > predictions);

    void realize_lane_change(map<int, vector<prediction> > predictions, string direction);

    void realize_prep_lane_change(map<int, vector<prediction> > predictions, string direction);

    vector<prediction> generate_predictions(int horizon);

    string get_next_state(map<int, vector<prediction>> predictions, int lanes_available);
    vector<snapshot> trajectory_for_state(string state, map<int, vector<prediction>> predictions,
      int horizon = 5);
    void restore_state_from_snapshot(snapshot snapshot);
    double calculate_cost(vector<snapshot> trajectory, map<int, vector<prediction>>predictions,
      string state, bool verbose = false);

    void _update_ref_speed_for_lane(map<int, vector<prediction> > predictions, int lane, int s);
    snapshot get_snapshot();
  };

}

#endif