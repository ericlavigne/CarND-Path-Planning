#ifndef PATH_PLANNING_CAR_STATE_H
#define PATH_PLANNING_CAR_STATE_H

#include <vector>
#include <string>
#include "prediction.h"

using namespace std;

class CarState {
public:
    CarState(double s, double d, double vs, double vd,
             double speed_limit, double penalty, double lane_line_seconds, double time, double lookahead_time, string came_from_key);
    CarState(const CarState &car);
    CarState();
    double value_estimate() const;
    vector<CarState> next_states(Prediction prediction, double delta_t);
    string key();
    string show();
    bool operator<(const CarState& rhs) const;
    double s, d, vs, vd, speed_limit, penalty, lane_line_seconds, time, lookahead_time;
    string came_from_key;
};

#endif //PATH_PLANNING_CAR_STATE_H
