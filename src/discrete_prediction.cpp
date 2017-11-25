#include "discrete_prediction.h"

DiscretePrediction::DiscretePrediction(vector<int> cars_s, vector<int> cars_d, vector<int> cars_v,
                                       int crash_distance, int preferred_distance)
{
    _cars_s.push_back(cars_s);
    _cars_d.push_back(cars_d);
    _cars_v.push_back(cars_v);
    _crash_distance = crash_distance;
    _preferred_distance = preferred_distance;
}

void DiscretePrediction::predict(int t_max) {
    for(int t = _cars_s.size(); t <= t_max; t++) {
        vector<int> new_s, new_d, new_v;
        for(int i = 0; i < _cars_s[t-1].size(); i++) {
            int v = _cars_v[t-1][i];
            int s = _cars_s[t-1][i] + v;
            int d = _cars_d[t-1][i];
            new_s.push_back(s);
            new_v.push_back(v);
            if((d % 2 == 1) && t >= 2) {
                // Car on lane line will be in lane within two seconds.
                // Not sure which lane. Consider both to be unavailable.
                new_s.push_back(s);
                new_v.push_back(v);
                new_d.push_back(d-1);
                new_d.push_back(d+1);
            } else {
                new_d.push_back(d);
            }
        }
        _cars_s.push_back(new_s);
        _cars_d.push_back(new_d);
        _cars_v.push_back(new_v);
    }
}

// Within 1 of d and within preferred_distance-1 of s
bool DiscretePrediction::tooClose(int s, int d, int t) {
    predict(t);
    for(int i = 0; i < _cars_s[t].size(); i++) {
        if((s < _cars_s[t][i] + _preferred_distance)
           && (s > _cars_s[t][i] - _preferred_distance)
           && (d <= _cars_d[t][i] + 1)
           && (d >= _cars_d[t][i] - 1))
        {
            return true;
        }
    }
    return false;
}

// Same d and within crash_distance of s
bool DiscretePrediction::crashed(int s, int d, int t) {
    predict(t);
    for(int i = 0; i < _cars_s[t].size(); i++) {
        if((s <= _cars_s[t][i] + _crash_distance)
           && (s >= _cars_s[t][i] - _crash_distance)
           && (d == _cars_d[t][i]))
        {
            return true;
        }
    }
    return false;
}

int DiscretePrediction::spaceAhead(int s, int d, int t, int horizon) {
    // How many spaces are available (not tooClose) directly ahead of this spot?
    // Do not look any further than horizon.
    predict(t);
    int min_dist = horizon;
    for(int i = 0; i < _cars_s[t].size(); i++) {
        if((d <= _cars_d[t][i] + 1) && (d >= _cars_d[t][i] - 1) && (s < _cars_s[t][i] + _preferred_distance)) {
            int dist = _cars_s[t][i] - s - _preferred_distance;
            min_dist = min(dist,min_dist);
            if(min_dist <= 0) {
                return 0;
            }
        }
    }
    return min_dist;
}
