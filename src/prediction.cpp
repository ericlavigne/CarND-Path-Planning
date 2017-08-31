#include "prediction.h"
#include <cmath>

Prediction::Prediction(vector<double> cars_s_, vector<double> cars_d_, vector<double> cars_vs_, vector<double> cars_vd_)
: cars_s(cars_s_), cars_d(cars_d_), cars_vs(cars_vs_), cars_vd(cars_vd_)
{

}

Prediction Prediction::forward_seconds(double seconds) {
    vector<double> result_s;
    vector<double> result_d;
    result_s.clear();
    result_d.clear();
    for(int i = 0; i < cars_s.size(); i++) {
        result_s.push_back(cars_s[i] + seconds * cars_vs[i]);
        result_d.push_back(cars_d[i] + seconds * cars_vd[i]);
    }
    Prediction result(result_s,result_d,cars_vs,cars_vd);
    return result;
}

double Prediction::nearest_car_distance(double s, double d) {
    double nearest = 1000;
    double car_length = 4.47;
    double car_width = 2.43;
    for(int i = 0; i < cars_s.size(); i++) {
        double distance = min(abs(cars_s[i] - s) - car_length,
                              abs(cars_d[i] - d) - car_width);
        if(distance < nearest) {
            nearest = distance;
        }
    }
    return nearest;
}

bool Prediction::crashed(double s, double d) {
    return nearest_car_distance(s,d) < 0.1;
}

bool Prediction::touching_lane_line(double d) {
    double car_width = 2.43;
    double lane_width = 4;
    int num_lanes = 3;
    if(d < 0.5 * car_width) {
        return true;
    }
    if(d > num_lanes * lane_width - 0.5 * car_width) {
        return true;
    }
    for(int i = 1; i < num_lanes; i++) {
        if(abs(lane_width * i - d) < 0.5 * car_width) {
            return true;
        }
    }
    return false;
}
