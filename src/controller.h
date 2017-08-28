#ifndef PATH_PLANNING_CONTROLLER_H
#define PATH_PLANNING_CONTROLLER_H

#include <vector>
#include <string>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
#include "track.h"

using namespace std;

class Controller {
public:
    Controller(Track track, double car_x, double car_y, double car_yaw, double speed_limit, double acceleration_limit, double jerk_limit);
    virtual ~Controller();
    void updatePathHistory(vector<double> prev_x, vector<double> prev_y);
    void updateTrajectory(vector<double> traj_x, vector<double> traj_y, vector<double> traj_speed);
    vector<double> getPathX();
    vector<double> getPathY();
private:
    Track track;
    vector<double> prev_x, prev_y;
    vector<double> path_x, path_y;
    vector<double> traj_x, traj_y, traj_speed;
    double delta_t, path_length;
    double speed_limit, acceleration_limit, jerk_limit;
    void recalculatePath();
};

#endif //PATH_PLANNING_CONTROLLER_H
