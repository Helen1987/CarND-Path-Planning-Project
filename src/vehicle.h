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

  class Vehicle {

  private:
    double const TIME_INTERVAL = 0.02;
    double const LANE_WIDTH = 4.0;
    double const MIDDLE_LANE = LANE_WIDTH/2;
    double const SAFE_DISTANCE = 10.0;
    double const SPEED_INCREMENT = .224;

    double x;
    double y;
    double dx;
    double dy;
    double ddx;
    double ddy;
    double yaw;

    struct estimate {
      string state;
      double cost;
    };

    double get_velocity() {
      return sqrt(dx*dx + dy*dy);
    }

  public:
    double const MAX_SPEED = 49.5;

    static vector<double> map_waypoints_x;
    static vector<double> map_waypoints_y;
    static vector<double> map_waypoints_s;

    int id;
    double s;
    double d;

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

      bool is_in_lane(int lane) {
        return d < (4.0 * (lane + 1)) && d >(4.0 * lane);
      }

      void display() {
        cout << "s: " << s << " d: " << d << " vx: " << vx << " vy: " << vy << endl;
      }
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
      double yaw;
      double ref_vel;
      double lane;
      string state;

      double get_speed() {
        return sqrt(dx*dx+dy*dy);
      }

      double get_acceleration() {
        return sqrt(ddx*ddx + ddy*ddy);
      }
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

    void update_params(double x, double y, double v, double yaw, double s, double d, double diff);

    void update_yaw(double x, double y, double vx, double vy, double s, double d, double diff);

    void update_state(map<int, vector<prediction>> predictions, int lanes_available);

   // void configure(vector<int> road_data);

    void display();

    void increment(double t = 5.0);

    prediction state_at(double t);

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

    void _update_ref_speed_for_lane(map<int, vector<prediction> > predictions, int lane, int s);
    snapshot get_snapshot();
  };

}

#endif