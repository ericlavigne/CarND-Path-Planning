#include "discrete_trajectory_planner.h"

DiscreteTrajectoryPlanner::DiscreteTrajectoryPlanner(int ego_s, int ego_d, int ego_v,
                                                     vector<int> other_s, vector<int> other_d, vector<int> other_v,
                                                     int simulate_steps, int horizon_steps,
                                                     int max_v, int max_a, int num_lanes,
                                                     int crash_distance, int preferred_distance)
{
    // TODO: Create relative versions of other_X
    _prediction = new DiscretePrediction(other_s,other_d,other_v,crash_distance,preferred_distance);
    TrajectoryState initialState(ego_s,ego_d,0,0,simulate_steps,horizon_steps,-ego_v,max_v-ego_v,max_a,0.0,num_lanes,_prediction);
    _optimizer = new AStar<TrajectoryState>(initialState);
}

DiscreteTrajectoryPlanner::~DiscreteTrajectoryPlanner() {
    delete _optimizer;
    delete _prediction;
}

bool DiscreteTrajectoryPlanner::finished() {
    return _optimizer->finished();
}

double DiscreteTrajectoryPlanner::score() {
    return _optimizer->score();
}

vector<int> DiscreteTrajectoryPlanner::pathS() {
    vector<TrajectoryState> relativePath = _optimizer->path();
    vector<int> result;
    for(int t = 0; t < relativePath.size(); t++) {
        result.push_back(relativePath[t].s() + t * _start_v);
    }
    return result;
}

vector<int> DiscreteTrajectoryPlanner::pathD() {
    vector<TrajectoryState> path = _optimizer->path();
    vector<int> result;
    for(int t = 0; t < path.size(); t++) {
        result.push_back(path[t].d());
    }
    return result;

}

vector<int> DiscreteTrajectoryPlanner::pathV() {
    vector<TrajectoryState> relativePath = _optimizer->path();
    vector<int> result;
    for(int t = 0; t < relativePath.size(); t++) {
        result.push_back(relativePath[t].v() + _start_v);
    }
    return result;
}

void DiscreteTrajectoryPlanner::calculateSeconds(double maxSeconds) {
    _optimizer->calculateSeconds(maxSeconds);
}

void DiscreteTrajectoryPlanner::calculateSteps(int maxSteps) {
    _optimizer->calculateSteps(maxSteps);
}
