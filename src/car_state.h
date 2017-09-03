#ifndef PATH_PLANNING_CAR_STATE_H
#define PATH_PLANNING_CAR_STATE_H

#include <vector>
#include "prediction.h"

using namespace std;

class CarState {
public:
    CarState(double s, double d, double vs, double vd, double penalty, double lane_line_seconds, double time);
    double value_estimate(double total_time);
    vector<CarState> next_states(Prediction prediction, double delta_t);
    string key();
private:
    double s, d, vs, vd, penalty, lane_line_seconds, time;
};

#endif //PATH_PLANNING_CAR_STATE_H
