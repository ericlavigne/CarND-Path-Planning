#include <iostream>
#include "trajectory_planner.h"
#include "car_state.h"
#include <ctime>

TrajectoryPlanner::TrajectoryPlanner(Prediction prediction_, double s_, double d_, double vs_, double vd_,
                                     double speed_limit_, double lookahead_seconds_, double delta_t_)
        : s(s_), d(d_), vs(vs_), vd(vd_),
          speed_limit(speed_limit_), lookahead_seconds(lookahead_seconds_), delta_t(delta_t_),
          best(s_,d_,max(vs_, speed_limit_ / 2),vd_,speed_limit_,0,0,0,lookahead_seconds_,"")
{
    cout << endl << "TrajectoryPlanner constructor shows speed limit " << speed_limit << " and car speed " << best.vs << endl << endl;
    CarState current_car(s,d,vs,vd,speed_limit,0,0,0,lookahead_seconds_,"");
    vector<CarState> next_cars = current_car.next_states(prediction_, 0.5);
    cout << endl << "Current car: " << current_car.show() << endl;
    for(CarState car : next_cars) {
        cout << "    " << car.show() << "    " << car.key() << endl;
    }
    cout << endl;

    openStates.push(best);
    for(int i = 0; (i - 1) * delta_t_ < lookahead_seconds_; i++) {
        predictions.push_back(prediction_.forward_seconds(i*delta_t_));
    }
    cout << "Finished creating TrajectoryPlanner" << endl;
}

void TrajectoryPlanner::calculate(double calc_time_limit_seconds) {
    cout << "Started TrajectoryPlanner::calculate" << endl;
    clock_t start_clock = clock();
    while(! openStates.empty()) {
        CarState current_state = openStates.top();
        cout << "    " << current_state.show() << "  ";
        openStates.pop();
        string current_key = current_state.key();
        if(closedStates.count(current_key) > 0) {
            // This state (or very similar) already closed. Skip it.
            cout << "dup" << endl;
        } else if((best.time > lookahead_seconds - delta_t / 2) &&
                (best.value_estimate() > current_state.value_estimate())) {
            cout << "worse than terminal" << endl;
            // Already found good end state, and the best candidate open state is worse. We're done.
            cout << "Trajectory Planner reached objective. Clearing open states." << endl;
            while(! openStates.empty()) {
                openStates.pop();
            }
        } else {
            closedStates[current_state.key()] = current_state;
            int current_time_step = (int) lround(current_state.time / delta_t);
            Prediction prediction = predictions[current_time_step + 1];
            vector<CarState> next_states = current_state.next_states(prediction, delta_t);
            cout << "spawn " << next_states.size() << " new states" << endl;
            for(CarState next_state : next_states) {
                cout << "        " << next_state.show() << "  ";
                int best_time_step = (int) lround(best.time / delta_t);
                int next_time_step = (int) lround(next_state.time / delta_t);
                int terminal_time_step = (int) lround(lookahead_seconds / delta_t);
                bool terminal_found = (best_time_step == terminal_time_step);
                double best_score = best.value_estimate();
                double next_score = next_state.value_estimate();
                string next_key = next_state.key();
                if(next_time_step > best_time_step) {
                    // Always prefer states that are further in the future.
                    cout << "**best+t** ";
                    best = next_state;
                } else if(next_time_step == best_time_step && next_score > best_score) {
                    // Same time as best. Better score. This is the new best.
                    cout << "**best+v** ";
                    best = next_state;
                }
                if(terminal_found && next_score < best_score) {
                    // Current best is not estimate, and this state's optimistic estimate is worse. Skip.
                    cout << "worse-than-best";
                } else if(closedStates.count(next_key) > 0) {
                    // Already closed this state (or very similar). Skip.
                    cout << "dup";
                } else if(next_time_step >= terminal_time_step) {
                    closedStates[next_state.key()] = next_state;
                    cout << "terminal";
                } else {
                    cout << "open";
                    openStates.push(next_state);
                }
                cout << endl;
            }
        }
        clock_t current_clock = clock();
        double elapsed_seconds = double(current_clock - start_clock) / CLOCKS_PER_SEC;
        if(elapsed_seconds > calc_time_limit_seconds) {
            cout << "Timeout in TrajectoryPlanner::calculate" << endl;
            return;
        }
        //cout << "TrajectoryPlanner::calculate - tick open:" << openStates.size() << " closed:" << closedStates.size() << endl;
    }
    cout << "Best: " << best.show() << "    " << best.key() << endl;
    cout << "Finished TrajectoryPlanner::calculate with " << closedStates.size() << " closed and "
         << openStates.size() << " open." << endl;
}

vector<CarState> TrajectoryPlanner::path() {
    //cout << "Started TrajectoryPlanner::path" << endl;
    vector<CarState> result;
    CarState current = best;
    while(true) {
        result.push_back(current);
        if(closedStates.count(current.came_from_key) > 0) {
            current = closedStates[current.came_from_key];
        } else {
            reverse(result.begin(), result.end());
            //cout << "Finished TrajectoryPlanner::path" << endl;
            return result;
        }
    }
}

vector<double> TrajectoryPlanner::pathS() {
    vector<double> result;
    for(CarState car : path()) {
        result.push_back(car.s);
    }
    return result;
}

vector<double> TrajectoryPlanner::pathD() {
    vector<double> result;
    for(CarState car : path()) {
        result.push_back(car.d);
    }
    return result;
}

vector<double> TrajectoryPlanner::pathV() {
    vector<double> result;
    for(CarState car : path()) {
        result.push_back(car.vs);
    }
    return result;
}
