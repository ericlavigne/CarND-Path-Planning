#include "car_state.h"
#include <string>
#include <cmath>
#include <sstream>
#include <iomanip>

CarState::CarState(double s_, double d_, double vs_, double vd_,
                   double speed_limit_, double penalty_, double lane_line_seconds_, double time_)
        : s(s_), d(d_), vs(vs_), vd(vd_), speed_limit(speed_limit_),
          penalty(penalty_), lane_line_seconds(lane_line_seconds_), time(time_) {}

double CarState::value_estimate(double total_time) {
    return s + (total_time - time) * speed_limit - penalty;
}

vector<CarState> CarState::next_states(Prediction prediction, double delta_t) {
    // List d,vd pairs for next states
    vector<double> next_d_values(0);
    vector<double> next_vd_values(0);
    // Aim for each of the lane centers with varying maximum acceleration
    for (double max_acceleration : {1.0, 2.0}) {
        // Determine range of d values at which we can have vd=0 in next tick.
        double min_stopped_pos, max_stopped_pos;
        if(abs(vd) < 0.1) {
            min_stopped_pos = d - max_acceleration * delta_t / 4;
            max_stopped_pos = d + max_acceleration * delta_t / 4;
        } else if(vd > 0) {
            min_stopped_pos = d + 0.5 / max_acceleration * pow(vd,2)
                              - 0.25 * max_acceleration * pow((delta_t - vd / max_acceleration),2);
            max_stopped_pos = d + 0.5 / max_acceleration * pow(vd,2)
                              + 0.25 * (delta_t - vd / max_acceleration) * (3 * vd + delta_t * max_acceleration);
        } else {
            max_stopped_pos = d - 0.5 / max_acceleration * pow(vd,2)
                              + 0.25 * max_acceleration * pow((delta_t + vd / max_acceleration),2);
            min_stopped_pos = d - 0.5 / max_acceleration * pow(vd,2)
                              - 0.25 * (delta_t + vd / max_acceleration) * (-3 * vd + delta_t * max_acceleration);
        }
        // Determine our closest stopping point after one tick accelerating left or right.
        // If goal outside this range then use max acceleration.
        // If goal inside this range use proportional acceleration based on where in this range.
        double left_accel_v = vd - max_acceleration * delta_t;
        double left_accel_d = d + vd * delta_t - 0.5 * max_acceleration * pow(delta_t,2);
        double left_accel_stop_time = abs(left_accel_v) / max_acceleration;
        double left_accel_stop_d = left_accel_d + 0.5 * left_accel_v * left_accel_stop_time;
        double right_accel_v = vd + max_acceleration * delta_t;
        double right_accel_d = d + vd * delta_t + 0.5 * max_acceleration * pow(delta_t,2);
        double right_accel_stop_time = abs(right_accel_v) / max_acceleration;
        double right_accel_stop_d = right_accel_d + 0.5 * right_accel_v * right_accel_stop_time;
        for(double lane_center : {2.0, 6.0, 10.0}) {
            if(lane_center > min_stopped_pos && lane_center < max_stopped_pos) {
                next_d_values.push_back(lane_center);
                next_vd_values.push_back(0.0);
            } else {
                double acceleration = 0;
                if (lane_center < left_accel_stop_d) {
                    acceleration = 0 - max_acceleration;
                } else if (lane_center > right_accel_stop_d) {
                    acceleration = max_acceleration;
                } else {
                    acceleration = 2 * max_acceleration
                                   * ((lane_center - left_accel_stop_d) / (right_accel_stop_d - left_accel_stop_d)
                                      - 0.5);
                }
                next_d_values.push_back(d + 0.5 * acceleration * pow(delta_t,2));
                next_vd_values.push_back(vd + acceleration * delta_t);
            }
        }
    }

    // Forward acceleration options: +10, 0, or -10 m/s2 limited by max speed and min speed (max/2)
    vector<double> next_vs_values(0);
    next_vs_values.push_back(vs);
    if(speed_limit > vs + 0.1) {
        next_vs_values.push_back(min(speed_limit, vs + delta_t * 10));
    }
    if(speed_limit / 2 < vs - 0.1) {
        next_vs_values.push_back(max(speed_limit / 2, vs - delta_t * 10));
    }
    // Create new car states for each combination of d,vd and vs
    vector<CarState> result;
    for(double next_vs : next_vs_values) {
        double next_s = s + 0.5 * delta_t * (vs + next_vs);
        for(int i = 0; i < next_d_values.size(); i++) {
            double next_d = next_d_values[i];
            double next_vd = next_vd_values[i];
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
            if(next_lane_line_seconds > 1) {
                next_penalty += 0.1 * delta_t;
            }
            if(next_lane_line_seconds > 2) {
                next_penalty += 0.2 * delta_t;
            }
            if(next_lane_line_seconds > 2.8) {
                next_penalty += 2.0 * delta_t;
            }
            CarState car(next_s, next_d, next_vs, next_vd, speed_limit, next_penalty, next_lane_line_seconds, time + delta_t);
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
           + " vd:" + (vd10 < 0 ? "-" : "") + to_string(abs_vd10 / 10) + "." + to_string(abs_vd10 % 10);
}

string CarState::show() {
    ostringstream stream;
    stream << "<CarState |";
    stream << setprecision(4) << " s:" << s;
    stream << setprecision(3) << " d:" << d << " vs:" << vs << " vd:" << vd;
    stream << setprecision(2) << " lane_sec:" << lane_line_seconds;
    stream << " >";
    return stream.str();
}
