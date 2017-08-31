#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include <vector>

using namespace std;

class Prediction {
public:
    Prediction(vector<double> cars_s, vector<double> cars_d, vector<double> cars_vs, vector<double> cars_vd);
    Prediction forward_seconds(double seconds);
    double nearest_car_distance(double s, double d);
    bool crashed(double s, double d);
private:
    vector<double> cars_s, cars_d, cars_vs, cars_vd;
};

#endif //PATH_PLANNING_PREDICTION_H
