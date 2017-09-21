#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_
#include <vector>
#include <math.h>
#include <functional>
#include "vehicle.h"

namespace pathplanner {
  using namespace std;

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

  class Estimator
  {
    public:
      Estimator(double max_speed, bool verbose);

      virtual ~Estimator();

      double calculate_cost(double car_s, double ref_vel, vector<snapshot> trajectory,
        map<int, vector<prediction>>predictions, CarState state);

    private:
      bool verbose = false;

      struct collision {
        bool hasCollision = false;
        int step = 1000;
      };

      struct TrajectoryData
      {
        int proposed_lane;
        int current_lane;
        double avg_speed;
        double prop_closest_approach;
        double actual_closest_approach;
        collision collides;
      };

      typedef std::function<double(const Estimator&, vector<snapshot> trajectory,
        map<int, vector<prediction>> predictions, TrajectoryData data)> DelegateType;

      vector<DelegateType> delegates = { (DelegateType)&Estimator::inefficiency_cost,
        (DelegateType)&Estimator::collision_cost,
        (DelegateType)&Estimator::buffer_cost,
        (DelegateType)&Estimator::change_lane_cost,
        (DelegateType)&Estimator::free_line_cost
      };

      // priority levels for costs
      int const COLLISION = pow(10, 6);
      int const DANGER = 3*pow(10, 5);
      int const COMFORT = pow(10, 4);
      int const EFFICIENCY = pow(10, 3);

      double MAX_SPEED;
      double const DESIRED_BUFFER = Vehicle::SAFE_DISTANCE*2;

      int const PLANNING_HORIZON = 1;

      double const PREDICTION_INTERVAL = 0.15;
      double const MANOEUVRE = 4.0;
      double const OBSERVED_DISTANCE = 65;
      double const MAX_DISTANCE = 999999;

      double change_lane_cost(vector<snapshot> trajectory,
        map<int, vector<prediction>> predictions, TrajectoryData data) const;

      double inefficiency_cost(vector<snapshot> trajectory,
        map<int, vector<prediction>> predictions, TrajectoryData data) const;

      double collision_cost(vector<snapshot> trajectory,
        map<int, vector<prediction>> predictions, TrajectoryData data) const;

      double buffer_cost(vector<snapshot> trajectory,
        map<int, vector<prediction>> predictions, TrajectoryData data) const;

      double free_line_cost(vector<snapshot> trajectory,
        map<int, vector<prediction>> predictions, TrajectoryData data) const;

      TrajectoryData get_helper_data(double car_s, double ref_s, vector<snapshot> trajectory,
        map<int, vector<prediction>> predictions, CarState state);

      bool check_collision(double car_s, double ref_speed, snapshot snap, prediction s_now, 
        CarState checkstate, bool lack_of_space);

      map<int, vector<prediction>> filter_predictions_by_lane(
        map<int, vector<prediction>> predictions, int lane);
  };

}
#endif //ESTIMATOR_H_