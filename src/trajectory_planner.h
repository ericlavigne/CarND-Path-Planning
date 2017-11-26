#ifndef PATH_PLANNING_TRAJECTORY_PLANNER_H
#define PATH_PLANNING_TRAJECTORY_PLANNER_H

#include <unordered_map>
#include <set>
#include <queue>
#include "discrete_trajectory_planner.h"

using namespace std;

class TrajectoryPlanner {
public:
    TrajectoryPlanner(double s, double d, double vs, double vd,
                      vector<double> other_s, vector<double> other_d, vector<double> other_vs, vector<double> other_vd,
                      double max_vs, double max_as, double lookahead_seconds);
    ~TrajectoryPlanner();
    void calculate(double calc_time_limit_seconds);
    vector<double> pathS();
    vector<double> pathD();
    vector<double> pathV();
private:
    double _lane_width = 4;
    int _num_lanes = 3;
    double _s, _d, _vs, _vd;
    vector<double> _other_s, _other_d, _other_vs, _other_vd;
    double _max_vs, _max_as, _lookahead_seconds;
    int _discrete_max_v;
    DiscreteTrajectoryPlanner* _discrete_planner;
    int convert_s_to_discrete(double s);
    int convert_d_to_discrete(double d, double vd);
    int convert_v_to_discrete(double vs);
    double convert_s_to_continuous(int d);
    double convert_d_to_continuous(int d);
    double convert_v_to_continuous(int v);
};

#endif //PATH_PLANNING_TRAJECTORY_PLANNER_H
