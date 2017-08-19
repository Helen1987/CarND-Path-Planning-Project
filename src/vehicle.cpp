#include "helper_functions.h"
#include "vehicle.h"

#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include "estimator.h"



namespace pathplanner {
  using namespace helpers;

  vector<double> Vehicle::map_waypoints_s;
  vector<double> Vehicle::map_waypoints_y;
  vector<double> Vehicle::map_waypoints_x;

  Vehicle::Vehicle(int id, double x, double y, double dx, double dy, double s, double d) {

    this->id = id;
    this->x = x;
    this->y = y;
    this->dx = dx;
    this->dy = dy;
    this->s = s;
    this->d = d;
    this->ddx = 0;
    this->ddy = 0;
    double angle = atan2(dy, dx);
    this->yaw = (abs(angle) < 0.1) ? 0 : angle;
    state = "CS";
    max_acceleration = 10;
  }

  Vehicle::Vehicle(int id) {
    this->id = id;
    state = "CS";
    max_acceleration = 10;
  }

  Vehicle::~Vehicle() {}

  // TODO - Implement this method.
  void Vehicle::update_state(map<int, vector<Vehicle::prediction>> predictions, int lanes_available) {
    /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
    - The vehicle will attempt to drive its target speed, unless there is
    traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
    - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
    behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
    - The vehicle will find the nearest vehicle in the adjacent lane which is
    BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
    3 : [
    {"s" : 4, "lane": 0},
    {"s" : 6, "lane": 0},
    {"s" : 8, "lane": 0},
    {"s" : 10, "lane": 0},
    ]
    }

    */
    state = get_next_state(predictions, lanes_available);
  }

  string Vehicle::get_next_state(map<int, vector<prediction>> predictions, int lanes_available) {
    vector<string> states;
    string PLCR = "PLCR", PLCL = "PLCL", KL = "KL", LCR = "LCR", LCL = "LCL";
    if (state == PLCR) {
      states = vector<string>{ PLCR, LCR, KL };
    }
    else if (state == PLCL) {
      states = vector<string>{ PLCL, LCL, KL };
    }
    else if (state == LCR) {
      states = vector<string>{ LCR, KL };
    }
    else if (state == LCL) {
      states = vector<string>{ LCL, KL };
    }
    else {
      states = vector<string>{ KL };
      if (this->d > LANE_WIDTH) {
        states.push_back(PLCL);
      }
      if (this->d < (lanes_available - 1)*LANE_WIDTH) {
        states.push_back(PLCR);
      }
    }
    auto restore_snapshot = this->get_snapshot();
    //cout << "before estimate: ";
    //restore_snapshot.display();
    //display();
    auto costs = vector<Vehicle::estimate>();
    Estimator estimator = Estimator();
    for (auto state : states) {
      Vehicle::estimate estimate = Vehicle::estimate();
      estimate.state = state;
      auto trajectory = trajectory_for_state(state, predictions);
      estimate.cost = estimator.calculate_cost(*this, trajectory, predictions, state);
      costs.push_back(estimate);
    }
    //restore_snapshot.display();
    //restore_state_from_snapshot(restore_snapshot);
    //cout << "after estimate: ";
    //display();
    auto best = min_element(std::begin(costs), std::end(costs),
      [](Vehicle::estimate est1, Vehicle::estimate est2) {
      return est1.cost < est2.cost;
    });
    //cout << "best estimate: " << (*best).cost << " in state " << (*best).state << " speed " << get_velocity() << endl;
    return (*best).state;
  }

  vector<Vehicle::snapshot> Vehicle::trajectory_for_state(string state, map<int, vector<Vehicle::prediction>> predictions,
    int horizon /*=5*/) {
    // remember current state
    auto initial_snapshot = this->get_snapshot();
    //cout << "init snapshot " << state << endl;
    //initial_snapshot.display();

    // pretend to be in new proposed state
    this->state = state;
    vector<snapshot> trajectory = { initial_snapshot };
    for (int i = 0; i < horizon; ++i) {
      restore_state_from_snapshot(initial_snapshot);
      this->state = state;
      realize_state(predictions);
      if (this->d > 12) {
        cout << "outside the road" << endl;
      }
      increment(PREDICTION_INTERVAL);
      trajectory.push_back(this->get_snapshot());

      // need to remove first prediction for each vehicle.
      for (auto pair : predictions) {
        auto vehicle_pred = pair.second;
        vehicle_pred.erase(vehicle_pred.begin());
      }
    }

    // restore state from snapshot
    restore_state_from_snapshot(initial_snapshot);
    return trajectory;
  }

  Vehicle::snapshot Vehicle::get_snapshot() {
    Vehicle::snapshot snapshot_temp;
    snapshot_temp.x = this->x;
    snapshot_temp.y = this->y;
    snapshot_temp.dx = this->dx;
    snapshot_temp.dy = this->dy;
    snapshot_temp.s = this->s;
    snapshot_temp.d = this->d;
    snapshot_temp.ddx = this->ddx;
    snapshot_temp.ddy = this->ddy;
    snapshot_temp.yaw = this->yaw;
    snapshot_temp.state = this->state;
    snapshot_temp.lane = this->lane;
    //snapshot_temp.ref_vel = this->ref_vel;

    return snapshot_temp;
  }

  void Vehicle::restore_state_from_snapshot(Vehicle::snapshot snapshot) {
    //s = snapshot
    this->s = snapshot.s;
    this->d = snapshot.d;
    this->x = snapshot.x;
    this->y = snapshot.y;
    this->dx = snapshot.dx;
    this->dy = snapshot.dy;
    this->ddx = snapshot.ddx;
    this->ddy = snapshot.ddy;
    this->yaw = snapshot.yaw;
    this->state = snapshot.state;
    //this->ref_vel = snapshot.ref_vel;
    this->lane = snapshot.lane;
  }

  //void Vehicle::configure(vector<int> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    //target_speed = road_data[0];
    //lanes_available = road_data[1];
    //goal_s = road_data[2];
    //goal_lane = road_data[3];
    //max_acceleration = road_data[4];
  //}

  void Vehicle::display() {
    cout << "vehicle " << this->id << " info" << endl;
    cout << "s:    " << this->s;
    cout << " d: " << this->d;
    cout << " x:    " << this->x;
    cout << " y: " << this->y;
    cout << " yaw: " << this->yaw;
    cout << " vx:    " << this->dx;
    cout << " vy:    " << this->dy;
    cout << " ax:    " << this->ddx;
    cout << " ay:    " << this->ddy << endl;
  }

  void Vehicle::update_params(double x, double y, double yaw, double s, double d, double diff) {
    this->x = x;
    this->y = y;
    this->yaw = abs(deg2rad(yaw)) < 0.1 ? 0 : deg2rad(yaw);
    //set_velocity(v, diff);
    this->s = s;
    this->d = d;
    //display();
  }

  void Vehicle::update_yaw(double x, double y, double vx, double vy, double s, double d, double diff) {
    /*this->yaw = atan2(y - this->y, x - this->x);*/
    double new_angle = atan2(vy, vx);
    this->yaw = (abs(new_angle) < 0.1) ? 0 : new_angle;
    this->x = x;
    this->y = y;
    this->ddx = (vx - this->dx) / diff;
    if (this->ddx < 0.01) {
      this->ddx = 0;
    }
    this->ddy = (vy - this->dy) / diff;
    if (this->ddy < 0.01) {
      this->ddy = 0;
    }
    this->dx = vx;
    this->dy = vy;
    this->s = s;
    this->d = d;
    ++updates;
    //display();
  }

  void Vehicle::increment(double t /*=PREDICTION_INTERVAL*/) {

    //double t = TIME_INTERVAL*dt;
    if (abs(this->ddy) < 0.001) {
      this->y += this->dy * t;
    }
    else {
      this->y += this->dy * t + this->ddy*t*t / 2;
      this->dy += this->ddy * t;
    }
    if (abs(this->ddx) < 0.001) {
      this->x += this->dx * t;
    }
    else {
      this->x += this->dx * t + this->ddx*t*t / 2;
      this->dx += this->ddx * t;
    }
    vector<double> frenet = getFrenet(this->x, this->y, this->yaw, Vehicle::map_waypoints_x, Vehicle::map_waypoints_y);
    this->s = frenet[0];
    this->d = frenet[1];
  }

  Vehicle::prediction Vehicle::state_at(double t) {

    /*
    Predicts state of vehicle in t seconds (assuming 0 acceleration)
    */
    prediction pred;
    //double t = count*TIME_INTERVAL;
    double x, y;
    if (abs(this->ddy) < 0.001) {
      y = this->y + this->dy * t;
      pred.vy = this->dy;
    }
    else {
      y = this->y + this->dy * t + this->ddy * t * t / 2;
      pred.vy = this->dy + this->ddy * t;
    }
    if (abs(this->ddy) < 0.001) {
      x = this->x + this->dx * t;
      pred.vx = this->dx;
    }
    else {
      x = this->x + this->dx * t + this->ddx * t * t / 2;
      pred.vx = this->dx + this->ddx * t;
    }
    vector<double> frenet = getFrenet(x, y, this->yaw, Vehicle::map_waypoints_x, Vehicle::map_waypoints_y);
    pred.s = frenet[0];
    pred.d = frenet[1];
    //cout << "vehicle " << this->id << " at: " << t << endl;
    //pred.display();
    return pred;
  }

  bool Vehicle::collides_with(Vehicle other, int at_time) {

    /*
    Simple collision detection.
    */
    prediction check1 = state_at(at_time*TIME_INTERVAL);
    prediction check2 = other.state_at(at_time*TIME_INTERVAL);
    return (abs(check1.d - check2.d) < LANE_WIDTH) && (abs(check1.s - check2.s) <= SAFE_DISTANCE);
  }

  Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

    Vehicle::collider collider_temp;
    collider_temp.collision = false;
    collider_temp.time = -1;

    for (int t = 0; t < timesteps + 1; t++)
    {
      if (collides_with(other, t))
      {
        collider_temp.collision = true;
        collider_temp.time = t;
        return collider_temp;
      }
    }

    return collider_temp;
  }

  void Vehicle::realize_state(map<int, vector<prediction>> predictions) {

    /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if (state.compare("CS") == 0)
    {
      realize_constant_speed();
    }
    else if (state.compare("KL") == 0)
    {
      realize_keep_lane(predictions);
    }
    else if (state.compare("LCL") == 0)
    {
      realize_lane_change(predictions, "L");
    }
    else if (state.compare("LCR") == 0)
    {
      realize_lane_change(predictions, "R");
    }
    else if (state.compare("PLCL") == 0)
    {
      realize_prep_lane_change(predictions, "L");
    }
    else if (state.compare("PLCR") == 0)
    {
      realize_prep_lane_change(predictions, "R");
    }
  }

  void Vehicle::realize_constant_speed() {
    ddx = 0;
    ddy = 0;
  }

  void Vehicle::_update_ref_speed_for_lane(map<int, vector<prediction>> predictions, int lane, int s) {
    bool too_close = false;
    for (auto pair : predictions) {
      prediction pred = pair.second[0];

      if (is_in_the_same_lane(pred.d)) {

        if ((pred.s > s) && (pred.s - s) < SAFE_DISTANCE) {
          cout << "pred d: " << pred.d << " my lane: " << lane << endl;
          cout << "pred s: " << pred.s << " my s: " << s << endl;
          too_close = true;
        }
      }
    }
    double velocity = get_velocity();
    //cout << "was vel: " << velocity;
    if (too_close) {
      velocity -= SPEED_INCREMENT;
      if (velocity < 0) {
        velocity = 0;
      }
    }
    else if (velocity < MAX_SPEED) {
      velocity += SPEED_INCREMENT;
      //cout << "_update_ref_speed_for_lane" << ref_vel << endl;
    }
    //cout << "became vel: " << velocity << endl;
    set_velocity(velocity, PREDICTION_INTERVAL);
    //cout << "upd ref speed vel " << velocity << endl;
  }

  void Vehicle::realize_keep_lane(map<int, vector<prediction>> predictions) {
    _update_ref_speed_for_lane(predictions, this->lane, this->s);
  }

  void Vehicle::realize_lane_change(map<int, vector<prediction>> predictions, string direction) {
    int delta = -1;
    if (direction.compare("L") == 0)
    {
      delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    _update_ref_speed_for_lane(predictions, lane, s);
  }

  void Vehicle::realize_prep_lane_change(map<int, vector<prediction> > predictions, string direction) {
    int delta = -1;
    if (direction.compare("L") == 0)
    {
      delta = 1;
    }
    int lane = this->lane + delta;

    vector<vector<prediction>> at_behind;
    for (auto pair : predictions) {
      int v_id = pair.first;
      vector<prediction> v = pair.second;
      if (is_in_the_same_lane(v[0].d) && (v[0].s < this->s)) {
        at_behind.push_back(v);
      }
    }
    if (at_behind.size() > 0)
    {
      int max_s = -1000;
      vector<prediction> nearest_behind = {};
      for (auto pred: at_behind) {
        if ((pred[0].s) > max_s)
        {
          max_s = pred[0].s;
          nearest_behind = pred;
        }
      }
      double target_vel = (nearest_behind[1].s - nearest_behind[0].s) / PREDICTION_INTERVAL;
      double velocity = get_velocity();
      double delta_v = velocity - target_vel;
      double delta_s = this->s - nearest_behind[0].s;
      //cout << "was vel: " << velocity;
      if (delta_v < -0.01)
      {
        if (abs(delta_v) < SPEED_INCREMENT) {
          velocity += SPEED_INCREMENT;
        }
        else {
          velocity += 2*SPEED_INCREMENT;
        }
        //cout << " realize_prep_lane_change " << ref_vel << endl;
      }
      else {
        velocity += SPEED_INCREMENT;
      }
      if (velocity > MAX_SPEED) {
        velocity = MAX_SPEED;
      }
      //cout << " became vel: " << velocity << endl;
      set_velocity(velocity, PREDICTION_INTERVAL);
    }
  }

  vector<Vehicle::prediction> Vehicle::generate_predictions(int horizon = 5) {

    vector<prediction> predictions;
    //cout << "in 30m: " << interval << " intervals" << endl;
    for (int i = 0; i < horizon; i++)
    {
      predictions.push_back(state_at(i*PREDICTION_INTERVAL));
    }
    return predictions;
  }
}