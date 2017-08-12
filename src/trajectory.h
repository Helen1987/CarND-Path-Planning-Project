#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_
#include <vector>

namespace pathplanner {
  using namespace std;

  class Trajectory
  {
    public:
      Trajectory(vector<double> previous_path_x, vector<double> previous_path_y);
      virtual ~Trajectory() {}
      // actual (x, y) we use for the planner
      vector<double> next_x_vals;
      vector<double> next_y_vals;

      vector<double> previous_path_x;
      vector<double> previous_path_y;

      void updateTrajectoryWaypoint(vector<double> ptsx, vector<double> ptsy,
        double ref_x, double ref_y, double ref_vel, double ref_yaw);

    private:


  };


}

#endif //TRAJECTORY_H_