#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_
#include <vector>

namespace pathplanner {
  using namespace std;

  class Trajectory
  {
    public:
      struct Coord {
        double x;
        double y;
      };

      Trajectory(vector<double> previous_path_x, vector<double> previous_path_y);
      virtual ~Trajectory() {}

      double ref_x;
      double ref_y;
      double ref_yaw;

      // actual (x, y) we use for the planner
      vector<double> next_x_vals;
      vector<double> next_y_vals;

      vector<double> previous_path_x;
      vector<double> previous_path_y;

      void update_trajectory(vector<double> ptsx, vector<double> ptsy, double ref_vel);

    private:

      void convert2Local(vector<double>& ptsx, vector<double>& ptsy);

      Coord convert2global(double x, double y);

  };


}

#endif //TRAJECTORY_H_