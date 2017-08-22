#ifndef MAP_H
#define MAP_H

#include <vector>

namespace helpers {
  using namespace std;

  struct Frenet
  {
    double s;
    double d;
  };

  struct Coord {
    double x;
    double y;
  };

  class  Map
  {
  public:
    Map();
    ~Map();

    void add_waypoints(double x, double y, double s, double d_x, double d_y);

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    Frenet getFrenet(double x, double y, double theta);

    // Transform from Frenet s,d coordinates to Cartesian x,y
    Coord getXY(double s, double d);

  private:
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    int NextWaypoint(double x, double y, double theta);
    int ClosestWaypoint(double x, double y);
    double distance(double x1, double y1, double x2, double y2);
  };



}

#endif