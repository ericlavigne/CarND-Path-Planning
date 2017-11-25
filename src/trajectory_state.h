#ifndef PATH_PLANNING_TRAJECTORY_STATE_H
#define PATH_PLANNING_TRAJECTORY_STATE_H

#include <string>
#include <vector>
#include "discrete_prediction.h"

using namespace std;

class TrajectoryState {
private:
    // s is distance along the track in meters
    int _s, _d, _v, _t, _simulate_steps, _horizon_steps, _min_v, _max_v, _max_a, _num_lanes;
    double _penalty_so_far;
    bool _valid; // False if default constructor used.
    DiscretePrediction* _p;
public:
    TrajectoryState(int s, int d, int v, int t, int simulate_steps, int horizon_steps,
                    int min_v, int max_v, int max_a, double penalty_so_far, int num_lanes,
                    DiscretePrediction* p);
    TrajectoryState(const TrajectoryState &state);
    TrajectoryState(); // Default is not valid. Exists because some containers require it.
    int s();
    int d();
    int v();
    double scoreEstimate() const; // Higher is better. Must be greater than or equal to true score.
    vector<TrajectoryState> nextStates() const;
    string key() const; // Must be unique. Used to determine if we've seen this state before.
    string show() const;
    bool final() const; // True if this state is candidate to be the end of a path.
    bool operator<(const TrajectoryState& rhs) const; // Based on score_estimate, needed for priority_queue
};

#endif
