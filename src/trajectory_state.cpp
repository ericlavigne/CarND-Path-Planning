#include "trajectory_state.h"
#include <sstream>
#include <iomanip>

TrajectoryState::TrajectoryState(int s, int d, int v, int t, int simulate_steps, int horizon_steps,
                                 int min_v, int max_v, int max_a, double penalty_so_far, int num_lanes,
                                 DiscretePrediction* p)
: _s(s), _d(d), _v(v), _t(t), _simulate_steps(simulate_steps), _horizon_steps(horizon_steps),
  _min_v(min_v), _max_v(max_v), _max_a(max_a), _penalty_so_far(penalty_so_far), _num_lanes(num_lanes), _p(p)
{
    if(p->crashed(s,d,t)) {
        _penalty_so_far += (simulate_steps + horizon_steps) * 1000;
    } else if(p->tooClose(s,d,t)) {
        _penalty_so_far += (simulate_steps + horizon_steps) * 100;
    }
    _valid = true;
}

TrajectoryState::TrajectoryState(const TrajectoryState &state)
: _s(state._s), _d(state._d), _v(state._v), _simulate_steps(state._simulate_steps), _horizon_steps(state._horizon_steps),
  _min_v(state._min_v), _max_v(state._max_v), _max_a(state._max_a), _penalty_so_far(state._penalty_so_far), _p(state._p)
{
    _valid = true;
}

TrajectoryState::TrajectoryState() {
    _valid = false;
}

int TrajectoryState::s() {
    return _s;
}

int TrajectoryState::d() {
    return _d;
}

int TrajectoryState::v() {
    return _v;
}

double TrajectoryState::scoreEstimate() const {
    if(!_valid) {
        return 0.0;
    }
    int steps_remaining = _simulate_steps + _horizon_steps;
    int max_avg_speed = min(_max_v, _v + ((steps_remaining + 1) / 2));
    int max_distance_remaining = steps_remaining * max_avg_speed;
    int min_distance_remaining = _p->spaceAhead(_s,_d,_t,max_distance_remaining);
    int max_avg_speed_with_lane_change = min(_max_v, _v + ((steps_remaining - 1) / 2));
    if(max_avg_speed_with_lane_change < _v) {
        max_avg_speed_with_lane_change = _v;
    }
    int max_distance_remaining_with_lane_change = steps_remaining * max_avg_speed_with_lane_change;
    if(max_distance_remaining_with_lane_change < 0) {
        max_distance_remaining_with_lane_change = 0;
    }
    int optimistic_distance_remaining = max(max_distance_remaining_with_lane_change, min_distance_remaining);
    return _s + optimistic_distance_remaining - _penalty_so_far;
}

vector<TrajectoryState> TrajectoryState::nextStates() const {
    vector<TrajectoryState> res;
    if(!_valid) {
        return res;
    }
    if(_d % 2 == 1) {
        // Between lanes. Only options are right or left.
        for(int vd = -1; vd <= 1; vd += 2) {
            res.push_back(TrajectoryState(_s+_v,_d+vd,_v,_t+1,_simulate_steps-1,
                                          _horizon_steps,_min_v,_max_v,_max_a,_penalty_so_far,_num_lanes,_p));
        }
    } else {
        for (int a = -_max_a; a <= _max_a; a++) {
            int v = _v + a;
            int v_ave = (_v + v) / 2;
            if (v >= _min_v && v <= _max_v) {
                res.push_back(TrajectoryState(_s+v_ave,_d,_v+a,_t+1,_simulate_steps-1,
                                              _horizon_steps,_min_v,_max_v,_max_a,_penalty_so_far,_num_lanes,_p));
            }
        }
        // Shifting lanes and accelerating are not allowed at the same time.
        for(int vd = -1; vd <= 1; vd++) {
            int d = _d + vd;
            if(d >= 0 && d <= 2*(_num_lanes-1)) {
                res.push_back(TrajectoryState(_s + _v, _d + vd, _v, _t + 1, _simulate_steps - 1,
                                              _horizon_steps, _min_v, _max_v, _max_a, _penalty_so_far, _num_lanes, _p));
            }
        }
    }
    return res;
}

string TrajectoryState::key() const {
    ostringstream stream;
    stream << "<TrjState |" << " s:" << _s << " d:" << _d << " v:" << _v << " t:" << _t << " penalty:" << int(_penalty_so_far) << " >";
    return stream.str();
}

string TrajectoryState::show() const{
    return key();
}

bool TrajectoryState::final() const {
    return _simulate_steps == 0;
}

bool TrajectoryState::operator<(const TrajectoryState& rhs) const {
    return scoreEstimate() < rhs.scoreEstimate();
}
