#ifndef PATH_PLANNING_TRACK_H
#define PATH_PLANNING_TRACK_H

#include <vector>
#include <string>
#include "spline.h"

using namespace std;

class Track {
public:
    Track(string map_file);
    virtual ~Track();

    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> frenet_to_xy(double s, double d);

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
