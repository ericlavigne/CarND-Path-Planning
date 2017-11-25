#ifndef PATH_PLANNING_DISCRETE_PREDICTION_H
#define PATH_PLANNING_DISCRETE_PREDICTION_H

#include <vector>

using namespace std;

class DiscretePrediction {
private:
    vector<vector<int>> _cars_s, _cars_d, _cars_v;
    int _crash_distance, _preferred_distance;
    void predict(int t);
public:
    DiscretePrediction(vector<int> cars_s, vector<int> cars_d, vector<int> cars_v, int crash_distance, int preferred_distance);
    bool tooClose(int s, int d, int t);
    bool crashed(int s, int d, int t);
    int spaceAhead(int s, int d, int t, int horizon);
};

#endif
