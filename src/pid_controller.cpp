#include <iostream>
#include <cmath>
#include "pid_controller.h"

PIDController::PIDController(vector<double> carx, vector<double> cary,
                             vector<double> trajx, vector<double> trajy, vector<double> trajv,
                             double seconds_before_traj, double seconds_per_traj)
{
    double minv = 0.015 / 0.02; // Ensure at least 0.01 meters per tick to avoid rounding problems.
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
    while(_pathx.size() < 50) {
        double t = _pathx.size() * 0.02;
        double t_plus = t + lookahead_seconds;
        double t_plus_traj_passed = (t_plus - seconds_before_traj) / seconds_per_traj;
        double traj1 = max(0, int(t_plus_traj_passed));
        double traj2 = traj1 + 1;
        double trajweight2 = t_plus_traj_passed - traj1;
        double trajweight1 = 1 - trajweight2;
        double trajx_plus = trajx[traj1] * trajweight1 + trajx[traj2] * trajweight2;
        double trajy_plus = trajy[traj1] * trajweight1 + trajy[traj2] * trajweight2;
        double trajv_plus = trajv[traj1] * trajweight1 + trajv[traj2] * trajweight2;
        v = max(minv, v + (trajv_plus - v) * 0.02);
        double delta_x = trajx_plus - prev_x;
        double delta_y = trajy_plus - prev_y;
        double distance = sqrt(pow(delta_x,2) + pow(delta_y,2));
        double dx = delta_x * v * 0.02 / distance;
        double dy = delta_y * v * 0.02 / distance;
        prev_x += dx;
        prev_y += dy;
        _pathx.push_back(prev_x);
        _pathy.push_back(prev_y);
    }
}

vector<double> PIDController::pathX() {
    return _pathx;
}

vector<double> PIDController::pathY() {
    return _pathy;
}
