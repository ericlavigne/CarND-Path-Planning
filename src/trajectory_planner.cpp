#include <iostream>
#include "trajectory_planner.h"
#include "car_state.h"

TrajectoryPlanner::TrajectoryPlanner(Prediction prediction, double s, double d, double vs, double vd) {
    CarState current_car(s,d,vs,vd,0,0,0);
    vector<CarState> next_cars = current_car.next_states(prediction, 0.5);
    cout << endl << "Current car: " << current_car.key() << endl;
    for(CarState car : next_cars) {
        cout << "    " << car.key() << endl;
    }
    cout << endl;
}
