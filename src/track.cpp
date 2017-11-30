#include "track.h"

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
        double s;
        double d_x;
        double d_y;
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

    // Connect the beginning and end of track if endpoints are close enough.
    min_waypoint_s = waypoints_s[0];
    max_waypoint_s = waypoints_s[waypoints_s.size()-1];
    double endpoint_distance_x = waypoints_x[0] - waypoints_x[waypoints_x.size()-1];
    double endpoint_distance_y = waypoints_y[0] - waypoints_y[waypoints_y.size()-1];
    endpoint_distance = sqrt(endpoint_distance_x*endpoint_distance_x + endpoint_distance_y*endpoint_distance_y);
    circular_track = (endpoint_distance < 100);

    // If circular track, need to add beginning points to end and vice-versa to create overlap.
    // In-between beginning and end, take weighted average of the two overlapping sections.
    vector<double> aug_x;
    vector<double> aug_y;
    vector<double> aug_s;
    vector<double> aug_dx;
    vector<double> aug_dy;
    double track_distance = max_waypoint_s - min_waypoint_s + (circular_track ? endpoint_distance : 0.0);
    if(circular_track) {
        for(int i : {waypoints_s.size() - 2, waypoints_s.size() - 1}) {
            aug_x.push_back(waypoints_x[i]);
            aug_y.push_back(waypoints_y[i]);
            aug_s.push_back(waypoints_s[i] - track_distance);
            aug_dx.push_back(waypoints_dx[i]);
            aug_dy.push_back(waypoints_dy[i]);
        }
    }
    for(int i = 0; i < waypoints_s.size(); i++) {
        aug_x.push_back(waypoints_x[i]);
        aug_y.push_back(waypoints_y[i]);
        aug_s.push_back(waypoints_s[i]);
        aug_dx.push_back(waypoints_dx[i]);
        aug_dy.push_back(waypoints_dy[i]);
    }
    if(circular_track) {
        for(int i : {0, 1}) {
            aug_x.push_back(waypoints_x[i]);
            aug_y.push_back(waypoints_y[i]);
            aug_s.push_back(waypoints_s[i] + track_distance);
            aug_dx.push_back(waypoints_dx[i]);
            aug_dy.push_back(waypoints_dy[i]);
        }
    }

    // Splines to support conversion from s,d to x,y.
    // Other direction is also possible but more difficult.
    s_x.set_points(aug_s,aug_x);
    s_y.set_points(aug_s,aug_y);
    s_dx.set_points(aug_s,aug_dx);
    s_dy.set_points(aug_s,aug_dy);
}

Track::Track(const Track &track) {
    this->circular_track = track.circular_track;
    this->endpoint_distance = track.endpoint_distance;
    this->max_waypoint_s = track.max_waypoint_s;
    this->min_waypoint_s = track.min_waypoint_s;
    this->s_x = track.s_x;
    this->s_y = track.s_y;
    this->s_dx = track.s_dx;
    this->s_dy = track.s_dy;
}

Track::~Track() {}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Track::sd_to_xy(double s, double d) {
    double path_x = s_x(s);
    double path_y = s_y(s);
    double dx = s_dx(s);
    double dy = s_dy(s);
    double x = path_x + d * dx;
    double y = path_y + d * dy;
    return {x,y};
}

// Transform from Cartesian x,y to Frenet s,d
vector<double> Track::xy_to_sd(double x, double y) {
    double s_est = min_waypoint_s;
    double dist2_est = pow(s_x(s_est) - x, 2) + pow(s_y(s_est) - y, 2);
    for(int i = 1; i <= 20; i++) {
        double s = i * (max_waypoint_s - min_waypoint_s) / 20 + min_waypoint_s;
        double dist2 = pow(s_x(s) - x, 2) + pow(s_y(s) - y, 2);
        if(dist2 < dist2_est) {
            dist2_est = dist2;
            s_est = s;
        }
    }
    for(int i = 0; i < 20; i++) {
        double x_est = s_x(s_est);
        double y_est = s_y(s_est);
        double x_diff = x - x_est;
        double y_diff = y - y_est;
        double dx = s_dx(s_est);
        double dy = s_dy(s_est);
        // plus_s is unit vector in direction of +s. Rotated 90 degrees counterclockwise from the (dx,dy) vector.
        double plus_s_x = 0 - dy;
        double plus_s_y = dx;
        double plus_s_mag = sqrt(pow(plus_s_x,2)+pow(plus_s_y,2));
        plus_s_x = plus_s_x / plus_s_mag;
        plus_s_y = plus_s_y / plus_s_mag;
        double delta_s = x_diff * plus_s_x + y_diff * plus_s_y;
        s_est += delta_s;
        if(abs(delta_s) < 0.01) {
            break;
        }
    }
    double x_est = s_x(s_est);
    double y_est = s_y(s_est);
    double x_diff = x - x_est;
    double y_diff = y - y_est;
    double dx = s_dx(s_est);
    double dy = s_dy(s_est);
    double d_mag = sqrt(pow(dx,2)+pow(dy,2));
    dx = dx / d_mag;
    dy = dy / d_mag;
    double d_est = dx * x_diff + dy * y_diff;
    return {s_est,d_est};
}

// Transform from Frenet s,d,vs,vd coordinates to Cartesian x,y,vx,vy
vector<double> Track::sd_to_xyv(double s, double d, double vs, double vd) {
    double small = 0.01;
    vector<double> xy1 = sd_to_xy(s,d);
    double s2 = s + small * vs;
    double d2 = d + small * vd;
    vector<double> xy2 = sd_to_xy(s2,d2);
    double vx = (xy2[0] - xy1[0]) * (1 / small);
    double vy = (xy2[1] - xy1[1]) * (1 / small);
    return {xy1[0],xy1[1], vx, vy};
}

// Transform from Cartesian x,y,vx,vy to Frenet s,d,vs,vd
vector<double> Track::xyv_to_sdv(double x, double y, double vx, double vy) {
    double small = 0.01;
    vector<double> sd1 = xy_to_sd(x,y);
    double x2 = x + small * vx;
    double y2 = y + small * vy;
    vector<double> sd2 = xy_to_sd(x2,y2);
    double vs = (sd2[0] - sd1[0]) * (1 / small);
    double vd = (sd2[1] - sd1[1]) * (1 / small);
    return {sd1[0], sd1[1], vs, vd};
}

// TODO: Improve v accuracy by using dot product between v and d vector.
// Current approach has outward bias during a curve and predicts slow lane shift when car moves at constant d.
