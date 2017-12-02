#include <iostream>
#include "trajectory_planner.h"

TrajectoryPlanner::TrajectoryPlanner(double s, double d, double vs, double vd,
                                     vector<double> other_s, vector<double> other_d,
                                     vector<double> other_vs, vector<double> other_vd,
                                     double max_vs, double max_as, double lookahead_seconds)

        : _s(s), _d(d), _vs(vs), _vd(vd), _other_s(other_s), _other_d(other_d), _other_vs(other_vs), _other_vd(other_vd),
          _max_vs(max_vs), _max_as(max_as), _lookahead_seconds(lookahead_seconds),
          _discrete_max_v(max(1,(int) floor(max_vs+0.1)))
{
    int discrete_max_a = max(1, (int) floor(max_as+0.1));
    int discrete_s = convert_s_to_discrete(s);
    int discrete_d = convert_d_to_discrete(d, vd);
    int discrete_v = convert_v_to_discrete(vs);
    vector<int> discrete_other_s, discrete_other_d, discrete_other_v;
    for(int i = 0; i < other_s.size(); i++) {
        discrete_other_s.push_back(convert_s_to_discrete(other_s[i]));
        discrete_other_d.push_back(convert_d_to_discrete(other_d[i],other_vd[i]));
        discrete_other_v.push_back(convert_v_to_discrete(other_vs[i]));
    }
    int discrete_lookahead_seconds = (int) ceil(lookahead_seconds);
    int horizon = 10;
    int num_lanes = 3;
    int crash_distance = 3;
    int preferred_distance = 10;
    _discrete_planner =
            new DiscreteTrajectoryPlanner(discrete_s,discrete_d,discrete_v,
                                          discrete_other_s,discrete_other_d,discrete_other_v,
                                          discrete_lookahead_seconds, horizon,
                                          _discrete_max_v, discrete_max_a, num_lanes,
                                          crash_distance, preferred_distance);
}

TrajectoryPlanner::~TrajectoryPlanner() {
    delete _discrete_planner;
}

void TrajectoryPlanner::calculate(double calc_time_limit_seconds) {
    _discrete_planner->calculateSeconds(calc_time_limit_seconds);
}

vector<double> TrajectoryPlanner::pathS() {
    vector<double> result;
    vector<int> discrete_path_s = _discrete_planner->pathS();
    for(int i = 0; i < discrete_path_s.size(); i++) {
        if(i==0) {
            result.push_back(_s);
        } else {
            result.push_back(convert_s_to_continuous(discrete_path_s[i]));
        }
    }
    return result;
}

vector<double> TrajectoryPlanner::pathD() {
    vector<double> result;
    vector<int> discrete_path_d = _discrete_planner->pathD();
    cout << "Plan:  " << _d;
    for(int i = 0; i < discrete_path_d.size(); i++) {
        if(i==0) {
            result.push_back(_d);
        } else {
            result.push_back(convert_d_to_continuous(discrete_path_d[i]));
        }
        cout << "   " << discrete_path_d[i];
    }
    cout << endl;
    return result;
}

vector<double> TrajectoryPlanner::pathV() {
    vector<double> result;
    vector<int> discrete_path_v = _discrete_planner->pathV();
    for(int i = 0; i < discrete_path_v.size(); i++) {
        if(i==0) {
            result.push_back(_vs);
        } else {
            result.push_back(convert_v_to_continuous(discrete_path_v[i]));
        }
    }
    return result;
}

int TrajectoryPlanner::convert_s_to_discrete(double s) {
    return int(0.5 + s);
}

int TrajectoryPlanner::convert_d_to_discrete(double d, double vd) {
    int result = int(0.5 + (d + vd) * 2.0 / _lane_width - 1.0);
    result = max(0,result); // Can't be off-road to left
    result = min((_num_lanes-1)*2,result); // Can't be off-road to right
    return result;
}

int TrajectoryPlanner::convert_v_to_discrete(double vs) {
    int result = int(0.5 + vs * _discrete_max_v / _max_vs);
    result = min(_discrete_max_v, result);
    return result;
}

double TrajectoryPlanner::convert_s_to_continuous(int s) {
    return (double) s;
}

double TrajectoryPlanner::convert_d_to_continuous(int d) {
    return (d+1) * _lane_width * 0.5;
}

double TrajectoryPlanner::convert_v_to_continuous(int v) {
    double result = v * _max_vs / _discrete_max_v;
    result = min(result,_max_vs);
    return result;
}
