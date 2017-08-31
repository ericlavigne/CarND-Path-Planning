#ifndef PATH_PLANNING_TRACK_H
#define PATH_PLANNING_TRACK_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
#include "spline.h"

using namespace std;

class Track {
public:
    explicit Track(string map_file);
    Track(const Track &track);
    virtual ~Track();

    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> sd_to_xy(double s, double d);

    // Transform from Cartesian x,y to Frenet s,d
    vector<double> xy_to_sd(double x, double y);

    // Transform from Frenet s,d,vs,vd coordinates to Cartesian x,y,vx,vy
    vector<double> sd_to_xyv(double s, double d, double vs, double vd);

    // Transform from Cartesian x,y,vx,vy to Frenet s,d,vs,vd
    vector<double> xyv_to_sdv(double x, double y, double vx, double vy);

private:
    tk::spline s_x;
    tk::spline s_y;
    tk::spline s_dx;
    tk::spline s_dy;
    bool circular_track;
    double min_waypoint_s;
    double max_waypoint_s;
    double endpoint_distance;
};

#endif //PATH_PLANNING_TRACK_H
