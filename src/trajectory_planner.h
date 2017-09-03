#ifndef PATH_PLANNING_TRAJECTORY_PLANNER_H
#define PATH_PLANNING_TRAJECTORY_PLANNER_H

#include "prediction.h"

using namespace std;

class TrajectoryPlanner {
public:
    TrajectoryPlanner(Prediction prediction_, double s, double d, double vs, double vd, double speed_limit);
};

#endif //PATH_PLANNING_TRAJECTORY_PLANNER_H
