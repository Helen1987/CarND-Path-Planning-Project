#include "trajectory.h"
#include <iostream>
#include <vector>
#include "libs/spline.h"

namespace pathplanner {
  using namespace std;

  Trajectory::Trajectory(vector<double> previous_path_x, vector<double> previous_path_y) {
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
  }

  void Trajectory::convert2Local(vector<double>& ptsx, vector<double>& ptsy) {

    for (int i = 0; i < ptsx.size(); ++i) {
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;
      ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
      ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
    }
  }

  Trajectory::Coord Trajectory::convert2global(double x, double y) {

    Coord coord;
    coord.x = (x*cos(ref_yaw) - y*sin(ref_yaw));
    coord.y = (x*sin(ref_yaw) + y*cos(ref_yaw));

    coord.x += ref_x;
    coord.y += ref_y;

    return coord;
  }

  void Trajectory::update_trajectory(vector<double> ptsx, vector<double> ptsy, double ref_vel) {

    convert2Local(ptsx, ptsy);

    // create a spline
    tk::spline s;
    // set x, y points to the spline
    s.set_points(ptsx, ptsy);

    // fill with the previous points from the last time
    for (int i = 0; i < previous_path_x.size(); ++i) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    // calculate how to break up spline points so that we travel at our ddesired reference velocity
    // we have local coordinates here
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

    double x_add_on = 0;

    // fill up the rest planner after filling it with the previous points, here we will always output 50 points
    for (int i = 1; i <= 50 - previous_path_x.size(); ++i) {
      double N = (target_dist / (.02*ref_vel / 2.24));
      double x_point = x_add_on + (target_x) / N;
      double y_point = s(x_point);

      x_add_on = x_point;

      // rotate back to normal after rotating it earlier
      Coord point = convert2global(x_point, y_point);

      next_x_vals.push_back(point.x);
      next_y_vals.push_back(point.y);
    }
  }
}