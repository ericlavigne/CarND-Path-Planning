#include <iostream>
#include "trajectory_planner.h"
#include "car_state.h"

TrajectoryPlanner::TrajectoryPlanner(Prediction prediction, double s, double d, double vs, double vd, double speed_limit) {
    CarState current_car(s,d,vs,vd,speed_limit,0,0,0);
    vector<CarState> next_cars = current_car.next_states(prediction, 0.5);
    cout << endl << "Current car: " << current_car.show() << endl;
    for(CarState car : next_cars) {
        cout << "    " << car.show() << "    " << car.key() << endl;
    }
    cout << endl;
}
