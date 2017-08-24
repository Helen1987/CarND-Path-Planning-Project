#ifndef FSM_H
#define FSM_H

#include <iostream>
#include <vector>
#include <map>

namespace pathplanner {
  using namespace std;

  class Vehicle;
  struct prediction;

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

  template <typename Enumeration>
  auto as_integer(Enumeration const value)
    -> typename std::underlying_type<Enumeration>::type
  {
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
  }

  class FSM {

  public:
    FSM(Vehicle& car);;
    ~FSM();

    Vehicle& ego_car;
    double car_s;

    bool verbosity = false;
    static double PREDICTION_INTERVAL;
    double get_expected_velocity();
    void update_state(map<int, vector<prediction>> predictions);
    void realize_state(map<int, vector<prediction> > predictions);

  private:
    double const SPEED_INCREMENT = .224;
    double const MAX_SPEED = 49.5;
    double const TIME_INTERVAL = 0.02;
    double const PREDICTIONS_COUNT = 5;


    int const lanes_available = 3;

    double ref_vel = 0;
    int proposed_lane;
    CarState state = CarState::CS;

    void realize_constant_speed();

    void realize_keep_lane(map<int, vector<prediction> > predictions);

    void realize_lane_change(map<int, vector<prediction> > predictions, string direction);

    void realize_prep_lane_change(map<int, vector<prediction> > predictions, string direction);

    CarState get_next_state(map<int, vector<prediction>> predictions);

    void _update_ref_speed_for_lane(map<int, vector<prediction> > predictions, int lane);

    vector<snapshot> trajectory_for_state(CarState state, map<int, vector<prediction>> predictions, int horizon);

    void restore_state_from_snapshot(snapshot snapshot);

    snapshot get_snapshot();
  };

}

#endif