#ifndef MAP_H
#define MAP_H
#include "libs/spline.h"

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

  class Map
  {
  public:

    static double MAX_S;

    static void add_waypoints(double x, double y, double s, double d_x, double d_y);
    static void init();

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    static Frenet getFrenet(double x, double y, double theta);

    // Transform from Frenet s,d coordinates to Cartesian x,y
    static Coord getXY(double s, double d);

  private:
    static tk::spline s_x, s_y, s_dx, s_dy;
    static vector<double> map_waypoints_x;
    static vector<double> map_waypoints_y;
    static vector<double> map_waypoints_s;
    static vector<double> map_waypoints_dx;
    static vector<double> map_waypoints_dy;

    static int NextWaypoint(double x, double y, double theta);
    static int ClosestWaypoint(double x, double y);
    static double distance(double x1, double y1, double x2, double y2);
    static int cyclic_index(int i);
  };

}

#endif