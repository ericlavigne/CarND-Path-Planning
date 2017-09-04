#ifndef PATH_PLANNING_TRAJECTORY_PLANNER_H
#define PATH_PLANNING_TRAJECTORY_PLANNER_H

#include <unordered_map>
#include <set>
#include <queue>
#include "prediction.h"
#include "car_state.h"

using namespace std;

class TrajectoryPlanner {
public:
    TrajectoryPlanner(Prediction prediction, double s, double d, double vs, double vd,
                      double speed_limit, double lookahead_seconds, double delta_t);
    void calculate(double calc_time_limit_seconds);
    vector<CarState> path();
    vector<double> pathS();
    vector<double> pathD();
    vector<double> pathV();
private:
    double s, d, vs, vd;
    double speed_limit, lookahead_seconds, delta_t;
    unordered_map<string,CarState> closedStates;
    priority_queue<CarState> openStates; // priority queue allows lookup of largest element by default.
    CarState best;
    vector<Prediction> predictions;
};

#endif //PATH_PLANNING_TRAJECTORY_PLANNER_H
