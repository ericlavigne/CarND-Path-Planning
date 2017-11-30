#include <iostream>
#include <cmath>
#include "pid_controller.h"
#include "spline.h"

PIDController::PIDController(vector<double> carx, vector<double> cary,
                             vector<double> trajx, vector<double> trajy, vector<double> trajv,
                             double seconds_before_traj, double seconds_per_traj)
{
    double minv = 0.0015 / 0.02; // Ensure at least 0.01 meters per tick to avoid rounding problems.
    double v = minv;
    for(int i = 0; i < 10 && i < carx.size(); i++) {
        _pathx.push_back(carx[i]);
        _pathy.push_back(cary[i]);
    }
    int last = _pathx.size() - 1;
    double prev_x = _pathx[last], prev_y = _pathy[last];
    if(_pathx.size() >= 2) {
        double prev_vx = (prev_x - _pathx[last-1]) * 50.0;
        double prev_vy = (prev_y - _pathy[last-1]) * 50.0;
        v = max(minv, sqrt(pow(prev_vx,2) + pow(prev_vy,2)));
    }
    double lookahead_seconds = 1.0;
    int expected_waypoints = 150;
    double dt = 0.02;

    vector<double> spline_points_x, spline_points_y, spline_points_t;
    for(int i = max(0,int(_pathx.size() - 2)); i < _pathx.size(); i++) {
        spline_points_x.push_back(_pathx[i]);
        spline_points_y.push_back(_pathy[i]);
        spline_points_t.push_back(i * dt);
    }
    for(int i = 0; i <= ceil(expected_waypoints * dt / seconds_per_traj) && i < trajx.size(); i++) {
        double t = seconds_before_traj + i * seconds_per_traj;
        if(t > spline_points_t[spline_points_t.size()-1]) {
            spline_points_x.push_back(trajx[i]);
            spline_points_y.push_back(trajy[i]);
            spline_points_t.push_back(t);
        }
        if(t > expected_waypoints * dt / seconds_per_traj) {
            break;
        }
    }
    double s = 0.0, x = _pathx[0], y = _pathy[0];
    vector<double> spline_points_s;
    for(int i = 0; i < spline_points_x.size(); i++) {
        double ds = sqrt(pow(spline_points_x[i] - x, 2) + pow(spline_points_y[i] - y, 2));
        s += ds;
        x = spline_points_x[i];
        y = spline_points_y[i];
        spline_points_s.push_back(s);
    }
    tk::spline spline_x, spline_y;
    spline_x.set_points(spline_points_s,spline_points_x);
    spline_y.set_points(spline_points_s,spline_points_y);

    s = sqrt(pow(_pathx[_pathx.size()-1] - _pathx[0], 2) + pow(_pathy[_pathy.size()-1] - _pathy[0], 2));

    while(_pathx.size() < expected_waypoints) {
        double t = _pathx.size() * dt;
        double t_plus = t + lookahead_seconds;
        double t_plus_traj_passed = (t_plus - seconds_before_traj) / seconds_per_traj;
        double traj1 = max(0, int(t_plus_traj_passed));
        double traj2 = traj1 + 1;
        double trajweight2 = t_plus_traj_passed - traj1;
        double trajweight1 = 1 - trajweight2;
        double trajv_plus = trajv[traj1] * trajweight1 + trajv[traj2] * trajweight2;
        v = max(minv, v + (trajv_plus - v) * dt);

        s = s + v * dt;
        double x = spline_x(s);
        double y = spline_y(s);
        _pathx.push_back(x);
        _pathy.push_back(y);
    }
    /*
    cout << "Trajectory:" << endl;
    for(int i = 0; i < 3; i++) {
        cout << "   " << trajx[i] << "    " << trajy[i] << endl;
    }
    cout << "Path:" << endl;
    for(int i = 0; i < 50; i++) {
        cout << "   " << _pathx[i] << "    " << _pathy[i] << endl;
    }
     */
}

vector<double> PIDController::pathX() {
    return _pathx;
}

vector<double> PIDController::pathY() {
    return _pathy;
}
