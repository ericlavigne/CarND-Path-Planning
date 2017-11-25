#ifndef PATH_PLANNING_DISCRETE_TRAJECTORY_PLANNER_H
#define PATH_PLANNING_DISCRETE_TRAJECTORY_PLANNER_H

#include <vector>
#include "discrete_prediction.h"
#include "astar.h"
#include "trajectory_state.h"

using namespace std;

class DiscreteTrajectoryPlanner {
private:
    DiscretePrediction* _prediction;
    AStar<TrajectoryState>* _optimizer;
    int _start_s, _start_d, _start_v;
protected:
    ~DiscreteTrajectoryPlanner();
public:
    DiscreteTrajectoryPlanner(int ego_s, int ego_d, int ego_v,
                              vector<int> other_s, vector<int> other_d, vector<int> other_v,
                              int simulate_steps, int horizon_steps,
                              int max_v, int max_a, int num_lanes,
                              int crash_distance, int preferred_distance);
    bool finished();
    double score();
    vector<int> pathS();
    vector<int> pathD();
    vector<int> pathV();
    void calculateSeconds(double seconds);
    void calculateSteps(int steps);
};

#endif
