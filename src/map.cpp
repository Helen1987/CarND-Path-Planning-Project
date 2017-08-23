#include <math.h>
#include "helper_functions.h"
#include "map.h"

namespace helpers {

  vector<double> Map::map_waypoints_x = vector<double>();
  vector<double> Map::map_waypoints_y = vector<double>();
  vector<double> Map::map_waypoints_s = vector<double>() ;
  vector<double> Map::map_waypoints_dx = vector<double>();
  vector<double> Map::map_waypoints_dy = vector<double>();

  Map::Map()
  {
  }

  Map::~Map()
  {
  }

  void Map::add_waypoints(double x, double y, double s, double d_x, double d_y) {
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  double Map::distance(double x1, double y1, double x2, double y2)
  {
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
  }

  Frenet Map::getFrenet(double x, double y, double theta) {
    Frenet frenet;
    int next_wp = NextWaypoint(x, y, theta);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
      prev_wp = map_waypoints_x.size() - 1;
    }

    double n_x = map_waypoints_x[next_wp] - map_waypoints_x[prev_wp];
    double n_y = map_waypoints_y[next_wp] - map_waypoints_y[prev_wp];
    double x_x = x - map_waypoints_x[prev_wp];
    double x_y = y - map_waypoints_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    frenet.d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - map_waypoints_x[prev_wp];
    double center_y = 2000 - map_waypoints_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
    {
      frenet.d *= -1;
    }

    // calculate s value
    frenet.s = 0;
    for (int i = 0; i < prev_wp; i++)
    {
      frenet.s += distance(map_waypoints_x[i], map_waypoints_y[i], map_waypoints_x[i + 1], map_waypoints_y[i + 1]);
    }

    frenet.s += distance(0, 0, proj_x, proj_y);

    return frenet;
  }

  Coord Map::getXY(double s, double d)
  {
    Coord coord;
    int prev_wp = -1;

    while (s > map_waypoints_s[prev_wp + 1] && (prev_wp < (int)(map_waypoints_s.size() - 1)))
    {
      prev_wp++;
    }

    int wp2 = (prev_wp + 1) % map_waypoints_x.size();

    double heading = atan2((map_waypoints_y[wp2] - map_waypoints_y[prev_wp]), (map_waypoints_x[wp2] - map_waypoints_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - map_waypoints_s[prev_wp]);

    double seg_x = map_waypoints_x[prev_wp] + seg_s*cos(heading);
    double seg_y = map_waypoints_y[prev_wp] + seg_s*sin(heading);

    double perp_heading = heading - pi() / 2;

    coord.x = seg_x + d*cos(perp_heading);
    coord.y = seg_y + d*sin(perp_heading);

    return coord;

  }

  int Map::NextWaypoint(double x, double y, double theta) {
    int closestWaypoint = ClosestWaypoint(x, y);

    double map_x = map_waypoints_x[closestWaypoint];
    double map_y = map_waypoints_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = abs(theta - heading);

    if (angle > pi() / 4)
    {
      closestWaypoint++;
    }

    return closestWaypoint;
  }

  int Map::ClosestWaypoint(double x, double y)
  {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < map_waypoints_x.size(); i++)
    {
      double map_x = map_waypoints_x[i];
      double map_y = map_waypoints_y[i];
      double dist = distance(x, y, map_x, map_y);
      if (dist < closestLen)
      {
        closestLen = dist;
        closestWaypoint = i;
      }

    }

    return closestWaypoint;

  }
}