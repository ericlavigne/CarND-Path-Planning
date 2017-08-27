#include "track.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>

Track::Track(string map_file) {
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    ifstream in_map(map_file.c_str(), ifstream::in);

    string line;
    while (getline(in_map, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        waypoints_x.push_back(x);
        waypoints_y.push_back(y);
        waypoints_s.push_back(s);
        waypoints_dx.push_back(d_x);
        waypoints_dy.push_back(d_y);
    }

    // Splines to support conversion from s,d to x,y.
    // Other direction is also possible but more difficult.
    s_x.set_points(waypoints_s,waypoints_x);
    s_y.set_points(waypoints_s,waypoints_y);
    s_dx.set_points(waypoints_s,waypoints_dx);
    s_dy.set_points(waypoints_s,waypoints_dy);

    // Connect the beginning and end of track if endpoints are close enough.
    min_waypoint_s = waypoints_s[0];
    max_waypoint_s = waypoints_s[waypoints_s.size()-1];
    double endpoint_distance_x = waypoints_x[0] - waypoints_x[waypoints_x.size()-1];
    double endpoint_distance_y = waypoints_y[0] - waypoints_y[waypoints_y.size()-1];
    endpoint_distance = sqrt(endpoint_distance_x*endpoint_distance_x + endpoint_distance_y*endpoint_distance_y);
    circular_track = (endpoint_distance < 100);
}

Track::~Track() {}

vector<double> Track::frenet_to_xy(double s, double d) {
    double path_x = s_x(s);
    double path_y = s_y(s);
    double dx = s_dx(s);
    double dy = s_dy(s);
    double x = path_x + d * dx;
    double y = path_y + d * dy;
    return {x,y};
}
