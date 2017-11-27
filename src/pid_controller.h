#ifndef PATH_PLANNING_PID_CONTROLLER_H
#define PATH_PLANNING_PID_CONTROLLER_H

#include <vector>

using namespace std;

class PIDController {
private:
    vector<double> _pathx, _pathy;
public:
    PIDController(vector<double> carx, vector<double> cary,
                  vector<double> trajx, vector<double> trajy, vector<double> trajv,
                  double seconds_before_traj, double seconds_per_traj);
    vector<double> pathX();
    vector<double> pathY();
};

#endif //PATH_PLANNING_PID_CONTROLLER_H
