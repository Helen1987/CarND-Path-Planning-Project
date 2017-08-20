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
    double const SAFE_DISTANCE = 15.0;
    //double const TOO_SHORT_DISTANCE = 15.0;
    double const SPEED_INCREMENT = .224;
    double const PREDICTION_INTERVAL = 0.5;
    //double const MANOEUVRE = 10;
    double const MANOEUVRE = 5;

    double x;
    double y;
    double dx;
    double dy;
    double ddx;
    double ddy;
    double yaw;

    int updates = 0;

    struct estimate {
      string state;
      double cost;
    };

    void set_velocity(double ref_vel, double diff) {
      //cout << "new vel: " << ref_vel << " old dx: " << this->dx << " old dy: " << this->dy << endl;
      double new_vx = ref_vel*cos(this->yaw);
      double new_vy = ref_vel*sin(this->yaw);
      this->ddx = (new_vx - this->dx) / diff;
      if (this->ddx < 0.01) {
        this->ddx = 0;
      }
      this->ddy = (new_vy - this->dy) / diff;
      if (this->ddy) {
        this->ddy = 0;
      }
      this->dx = new_vx;
      this->dy = new_vy;
      //cout << "updated vel: " << get_velocity() << " new dx: " << this->dx << " new dy: " << this->dy << endl;
    }

    /*bool is_in_the_same_lane(double other_d) {
      return (other_d < LANE_WIDTH * (lane + 1)) && (other_d > LANE_WIDTH * lane);
    }*/

  public:
    double const MAX_SPEED = 49.5;

    static vector<double> map_waypoints_x;
    static vector<double> map_waypoints_y;
    static vector<double> map_waypoints_s;

    int id;
    double s;
    double d;

    double get_velocity() {
      return sqrt(dx*dx + dy*dy);
    }

    int lane = 1;
    int proposed_lane = 1;

    struct collider {
      bool collision; // is there a collision?
      int time; // time collision happens
    };

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
      //double ref_vel;
      int lane;
      int proposed_lane;

      string state;

      double get_speed() {
        return sqrt(dx*dx+dy*dy);
      }

      double get_acceleration() {
        return sqrt(ddx*ddx + ddy*ddy);
      }

      void display() {
        cout << "snapshot: x " << x << " y " << y << " dx " << dx << " dy "
          << dy << " ddx " << ddx << " ddy " << ddy << " s " << s << " d " << d << " yaw " << yaw
          << " lane " << lane << " state " << state << endl;
      }
    };

    bool shouldPredict() {
      return updates > 10;
    }

    int preferred_buffer = 6; // impacts "keep lane" behavior.

    int max_acceleration;

    string state;

    Vehicle(int id);

    Vehicle(int id, double x, double y, double dx, double dy, double s, double d);

    /**
    * Destructor
    */
    virtual ~Vehicle();

    void update_params(double x, double y, double yaw, double s, double d, double diff);

    void update_yaw(double x, double y, double vx, double vy, double s, double d, double diff);

    void update_state(map<int, vector<prediction>> predictions, int lanes_available);

   // void configure(vector<int> road_data);

    void display();

    void increment(double t);

    prediction state_at(double t);

    //bool collides_with(Vehicle other, int at_time);

    bool is_in_front_of(prediction pred);

    bool is_behind_of(prediction pred);

    bool is_close_to(prediction pred);

    //collider will_collide_with(Vehicle other, int timesteps);

    void realize_state(map<int, vector<prediction> > predictions, bool verbosity = false);

    void realize_constant_speed();

    void realize_keep_lane(map<int, vector<prediction> > predictions, bool verbosity = false);

    void realize_lane_change(map<int, vector<prediction> > predictions, string direction, bool verbosity = false);

    void realize_prep_lane_change(map<int, vector<prediction> > predictions, string direction, bool verbosity = false);

    vector<prediction> generate_predictions(int horizon = 10);

    string get_next_state(map<int, vector<prediction>> predictions, int lanes_available);
    vector<snapshot> trajectory_for_state(string state, map<int, vector<prediction>> predictions,
      int horizon = 10);
    void restore_state_from_snapshot(snapshot snapshot);

    void _update_ref_speed_for_lane(map<int, vector<prediction> > predictions, int lane, int s, bool verbosity = false);
    snapshot get_snapshot();
  };

}

#endif