#include "helper_functions.h"
#include "vehicle.h"

#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

#include "estimator.h"
#include "map.h"
#include "FSM.h"


namespace pathplanner {
  using namespace helpers;

  double Vehicle::SAFE_DISTANCE = 10.0;

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
    max_acceleration = 10;
  }

  Vehicle::Vehicle(int id) {
    this->id = id;
    max_acceleration = 10;
  }

  Vehicle::~Vehicle() {}

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
    cout << " ay:    " << this->ddy;
    cout << " line: " << this->lane << endl;
  }

  /*void Vehicle::reset(double x, double y, double vx, double vy, double s, double d) {
    double new_angle = atan2(vy, vx);
    this->yaw = (abs(new_angle) < 0.1) ? 0 : new_angle;
    this->x = x;
    this->y = y;
    this->ddx = 0;
    this->ddy = 0;
    this->dx = vx;
    this->dy = vy;
    this->s = s;
    this->d = d;
  }*/

  void Vehicle::update_params(double x, double y, double yaw, double s, double d, double speed, double diff) {
    this->x = x;
    this->y = y;
    this->yaw = deg2rad(yaw);
    update_accel(speed*cos(this->yaw), speed*sin(this->yaw), diff);
    this->s = s;
    this->d = d;
    //display();
  }

  void Vehicle::update_accel(double vx, double vy, double diff) {
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
  }

  void Vehicle::update_yaw(double x, double y, double vx, double vy, double s, double d, double diff) {
    /*this->yaw = atan2(y - this->y, x - this->x);*/
    double new_angle = atan2(vy, vx);
    this->yaw = (abs(new_angle) < 0.1) ? 0 : new_angle;
    this->x = x;
    this->y = y;
    update_accel(vx, vy, diff);
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
    double new_angle = atan2(dy, dx);
    this->yaw = (new_angle < 0.1) ? 0 : new_angle;
    Frenet frenet = Map::getFrenet(this->x, this->y, this->yaw);
    this->s = frenet.s;
    this->d = frenet.d;
  }

  prediction Vehicle::state_at(double t) {
    prediction pred;
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
    double new_angle = atan2(pred.vy, pred.vx);
    double yaw = (new_angle < 0.1) ? 0 : new_angle;
    Frenet frenet = Map::getFrenet(x, y, yaw);
    pred.s = frenet.s;
    pred.d = frenet.d;
    return pred;
  }

  bool Vehicle::is_in_front_of(prediction pred, int checked_lane) {
    return pred.is_in_lane(checked_lane) && pred.s < s;
  }

  bool Vehicle::is_behind_of(prediction pred, int lane) {
    return pred.is_in_lane(lane) && (pred.s > s && (pred.s - s) < 3*SAFE_DISTANCE);
  }

  bool Vehicle::is_close_to(prediction pred, int lane) {
    return pred.is_in_lane(lane) && pred.s > s && (pred.s - s < SAFE_DISTANCE);
  }

  vector<prediction> Vehicle::generate_predictions(int horizon) {

    vector<prediction> predictions;
    //cout << "in 30m: " << interval << " intervals" << endl;
    for (int i = 0; i < horizon; i++)
    {
      predictions.push_back(state_at(i*FSM::PREDICTION_INTERVAL));
    }
    return predictions;
  }

  void Vehicle::restore_state_from_snapshot(snapshot snapshot) {
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
    //this->state = snapshot.state;
    this->lane = snapshot.lane;
    //this->ref_vel = snapshot.ref_vel;
  }

  snapshot Vehicle::get_snapshot() {
    snapshot snapshot_temp;
    snapshot_temp.x = this->x;
    snapshot_temp.y = this->y;
    snapshot_temp.dx = this->dx;
    snapshot_temp.dy = this->dy;
    snapshot_temp.s = this->s;
    snapshot_temp.d = this->d;
    snapshot_temp.ddx = this->ddx;
    snapshot_temp.ddy = this->ddy;
    snapshot_temp.yaw = this->yaw;
    //snapshot_temp.state = this->state;
    snapshot_temp.lane = this->lane;
    //snapshot_temp.ref_vel = this->ref_vel;

    return snapshot_temp;
  }
}