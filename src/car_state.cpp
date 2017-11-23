#include "car_state.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <iostream>

CarState::CarState(double s_, double d_, double vs_, double vd_,
                   double speed_limit_, double penalty_, double lane_line_seconds_,
                   double time_, double lookahead_time_, string came_from_key_)
        : s(s_), d(d_), vs(vs_), vd(vd_), speed_limit(speed_limit_),
          penalty(penalty_), lane_line_seconds(lane_line_seconds_),
          time(time_), lookahead_time(lookahead_time_), came_from_key(came_from_key_) {}

CarState::CarState(const CarState &car)
        : s(car.s), d(car.d), vs(car.vs), vd(car.vd), speed_limit(car.speed_limit),
          penalty(car.penalty), lane_line_seconds(car.lane_line_seconds),
          time(car.time), lookahead_time(car.lookahead_time), came_from_key(car.came_from_key) {}

CarState::CarState() {}

double CarState::value_estimate() const {
    return s + (lookahead_time - time) * speed_limit - penalty;
}

vector<CarState> CarState::next_states(Prediction prediction, double delta_t) {
    double speed_limit_d = 1 * delta_t; // 1 m/s max, but will apply penalty if actually go that fast
    double min_d = d - speed_limit_d;
    double max_d = d + speed_limit_d;
    double left_side_of_road = 1;
    double right_side_of_road = 3 * 4 - 1;
    if(min_d < left_side_of_road) {
        min_d = left_side_of_road;
    }
    if(max_d > right_side_of_road) {
        max_d = right_side_of_road;
    }
    if(d < left_side_of_road) {
        min_d = left_side_of_road;
        max_d = left_side_of_road + speed_limit_d;
    }
    if(d > right_side_of_road) {
        min_d = right_side_of_road - speed_limit_d;
        max_d = right_side_of_road;
    }
    if(max_d < min_d) {
        cout << "CarState no options min_d:" << min_d << " max_d:" << max_d << endl;
        throw 0;
    }

    // List d,vd pairs for next states
    vector<double> next_d_values(0);
    for(int i = 0; i < 7; i++) {
        double new_d = min_d + i * (max_d - min_d) / 6.0;
        next_d_values.push_back(new_d);
    }
    // If lane center in range, make sure it is included.
    for(double lane_center : {2,6,10}) {
        if(lane_center <= max_d && lane_center >= min_d) {
            next_d_values.push_back(lane_center);
        }
    }

    // List vs options for next states
    double max_acceleration = 10;
    double min_v = max(vs,1.0) - max_acceleration * delta_t;
    double max_v = max(vs,1.0) + max_acceleration * delta_t;
    if(max_v > speed_limit) {
        max_v = speed_limit;
    }
    if(min_v < 1) {
        min_v = 1;
    }
    if(max_v < min_v) {
        min_v = max_v;
    }
    vector<double> next_vs_values(0);
    for(int i = 0; i < 5; i++) {
        double new_vs = min_v + i * (max_v - min_v) / 4.0;
        next_vs_values.push_back(new_vs);
    }

    // Create new car states for each combination of d,vd and vs
    vector<CarState> result;
    string old_key = key();
    for(double next_vs : next_vs_values) {
        double next_s = s + 0.5 * delta_t * (vs + next_vs);
        for(int i = 0; i < next_d_values.size(); i++) {
            double next_d = next_d_values[i];
            double next_vd = (next_d - d) / delta_t;
            double next_lane_line_seconds = lane_line_seconds;
            if(prediction.touching_lane_line(next_d)) {
                next_lane_line_seconds += delta_t;
            } else {
                next_lane_line_seconds = 0;
            }
            double next_penalty = penalty;
            if(prediction.crashed(next_s, next_d)) {
                next_penalty += 1000;
            }
            next_penalty += 0.1 * next_lane_line_seconds * delta_t;
            if(next_lane_line_seconds > 2) {
                next_penalty += 0.2 * delta_t;
            }
            if(next_lane_line_seconds > 2.8) {
                next_penalty += 2.0 * delta_t;
            }
            double off_road_distance = prediction.distance_from_drivable_region(next_d);
            if(off_road_distance > 0.01) {
                next_penalty += 200 + off_road_distance * 20;
            }
            //double distance_to_crash = prediction.nearest_car_distance(next_s,next_d);
            //if(distance_to_crash < 20) {
            //    next_penalty += pow(20 - distance_to_crash, 2);
            //}
            CarState car(next_s, next_d, next_vs, next_vd, speed_limit, next_penalty,
                         next_lane_line_seconds, time + delta_t, lookahead_time, old_key);
            result.push_back(car);
        }
    }
    return result;
}

// Unique key to represent this car state as node in A*.
// Deliberately reduce information content so that very similar car states will not both be traversed by A*.
string CarState::key() {
    // Forward speed almost always in 40-50 range. Small forward speed differences not interesting.
    int approx_vs = ((int) (vs / 5 + 0.5)) * 5;
    // Distance the car can travel in half a second should be sufficiently fine-grained to measure progress.
    int s_resolution = 5;
    int approx_s = s_resolution * ((int) (s / s_resolution + 0.5));
    // Position within lane is very important. Use resolution of 1/10 meter to represent 8 positions within each lane.
    int d10 = ((int) ((d * 10) + 0.5));
    // Movement left and right is very important. Resolution of 1/10 m/s when typical lane change is 1 m/s.
    int vd10 = ((int) ((vd * 10) + 0.5));
    int abs_vd10 = (int) abs(vd10 + 0.5);
    // Turn it all into string. Might be worth using format function for speed.
    return "s:" + to_string(approx_s)
           + " d:" + to_string(d10 / 10) + "." + to_string(d10 % 10)
           + " vs:" + to_string(approx_vs)
           + " vd:" + (vd10 < 0 ? "-" : "") + to_string(abs_vd10 / 10) + "." + to_string(abs_vd10 % 10)
           + " t:" + to_string(time) + " p:" + to_string(10 * int(penalty / 10.0));
}

string CarState::show() {
    ostringstream stream;
    stream << "<CarState |";
    stream << setprecision(4) << " s:" << s;
    stream << setprecision(3) << " d:" << d << " vs:" << vs << " vd:" << vd;
    stream << setprecision(2) << " lane_sec:" << lane_line_seconds << " value:" << value_estimate();
    stream << " >";
    return stream.str();
}

bool CarState::operator<(const CarState& rhs) const
{
    return value_estimate() < rhs.value_estimate();
}
