#include "controller.h"

Controller::Controller(Track track_, double car_x, double car_y, double car_yaw_radians,
                       double speed_limit, double acceleration_limit, double jerk_limit) : track(track_)
{
    cout << "Controller constructor: carx:" << car_x << " cary:" << car_y << " yawradians:" << car_yaw_radians
         << " speedlimit:" << speed_limit << " acclimit:" << acceleration_limit << " jerklimit:" << jerk_limit << endl;

    this->delta_t = 0.02;
    this->path_length = 50;
    this->speed_limit = speed_limit;
    this->acceleration_limit = acceleration_limit;
    this->jerk_limit = jerk_limit;

    double speed = 0.0;
    double acceleration = 0.0;
    double prev_x = car_x;
    double prev_y = car_y;
    double dir_x = cos(car_yaw_radians);
    double dir_y = sin(car_yaw_radians);
    for(int i = 0; i < path_length; i++) {
        acceleration += jerk_limit * delta_t;
        if(acceleration > acceleration_limit) {
            acceleration = acceleration_limit;
        }
        speed += acceleration * delta_t;
        if(speed > speed_limit) {
            speed = speed_limit;
        }
        prev_x += speed * delta_t * dir_x;
        prev_y += speed * delta_t * dir_y;
        this->prev_x.push_back(prev_x);
        this->prev_y.push_back(prev_y);
        this->path_x.push_back(prev_x);
        this->path_y.push_back(prev_y);
        this->traj_x.push_back(prev_x);
        this->traj_y.push_back(prev_y);
        this->traj_speed.push_back(speed);
        cout << "    x:" << prev_x << " y:" << prev_y << " v:" << speed << " a:" << acceleration << endl;
    }
    cout << "Controller constructor finished" << endl;
}

Controller::~Controller() {}

void Controller::updatePathHistory(vector<double> prev_x, vector<double> prev_y) {
    cout << "Controller::updatePathHistory with " << prev_x.size() << " path history points" << endl;
    if(prev_x.size() > 2) {
        this->prev_x = prev_x;
        this->prev_y = prev_y;
        recalculatePath();
    } else {
        cout << "Called Controller::updatePathHistory with path size of " << prev_x.size() << endl;
    }
}

void Controller::updateTrajectory(vector<double> traj_x, vector<double> traj_y, vector<double> traj_speed) {
    cout << "Controller::updateTrajectory with " << traj_x.size() << " trajectory points" << endl;
    this->traj_x = traj_x;
    this->traj_y = traj_y;
    this->traj_speed = traj_speed;
    recalculatePath();
}

vector<double> Controller::getPathX() {
    return this->path_x;
}

vector<double> Controller::getPathY() {
    return this->path_y;
}

void Controller::recalculatePath() {
    int prev_last = prev_x.size() - 1;
    double ref_x = prev_x[prev_last];
    double ref_y = prev_y[prev_last];
    double ref_yaw = atan2(ref_y - prev_y[prev_last-1], ref_x - prev_x[prev_last-1]);
    cout << "Controller::recalculatePath start ref x:" << ref_x << " y:" << ref_y << " yaw:" << ref_yaw << endl;

    // Fill in path_x and path_y with car-local conversions of prev_x and prev_y (will convert back to map coords later)
    path_x.clear();
    path_y.clear();
    cout << "Convert previous points from global to car coordinates" << endl;
    for(int i = 0; i < prev_x.size(); i++) {
        double shift_x = prev_x[i] - ref_x;
        double shift_y = prev_y[i] - ref_y;
        path_x.push_back((shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw)));
        path_y.push_back((shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw)));
        cout << "    global x:" << prev_x[i] << " y:" << prev_y[i] << "  car x:" << path_x[i] << " y:" << path_y[i] << endl;
    }
    // Determine velocity and acceleration for previous points, which are helpful for speed, acceleration, and jerk constraints.
    vector<double> path_vx, path_vy, path_ax, path_ay;
    path_vx.clear();
    path_vy.clear();
    path_ax.clear();
    path_ay.clear();
    cout << "Augment previous with speed and acceleration" << endl;
    for(int i = 0; i < path_x.size(); i++) {
        if(i == 0) {
            path_vx.push_back((path_x[1] - path_x[0]) / delta_t);
            path_vy.push_back((path_y[1] - path_y[1]) / delta_t);
        } else {
            path_vx.push_back((path_x[i] - path_x[i-1]) / delta_t);
            path_vy.push_back((path_y[i] - path_y[i-1]) / delta_t);
        }
    }
    for(int i = 0; i < path_x.size(); i++) {
        if(i == 0) {
            path_ax.push_back((path_vx[1] - path_vx[0]) / delta_t);
            path_ay.push_back((path_vy[1] - path_vy[1]) / delta_t);
        } else {
            path_ax.push_back((path_vx[i] - path_vx[i-1]) / delta_t);
            path_ay.push_back((path_vy[i] - path_vy[i-1]) / delta_t);
        }
        cout << "    x:" << path_x[i] << " vx:" << path_vx[i] << " ax:" << path_ax[i]
             << " y:" << path_y[i] << " vy:" << path_vy[i] << " ay:" << path_ay[i] << endl;
    }
    // Transform trajectory relative to car's current location and orientation
    cout << "Convert trajectory from global to car coordinates" << endl;
    vector<double> traj_car_x;
    vector<double> traj_car_y;
    traj_car_x.clear();
    traj_car_y.clear();
    for(int i = 0; i < traj_x.size(); i++) {
        double shift_x = traj_x[i] - ref_x;
        double shift_y = traj_y[i] - ref_y;
        traj_car_x.push_back((shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw)));
        traj_car_y.push_back((shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw)));
        cout << "    global x:" << traj_x[i] << " y:" << traj_y[i] << "  car x:" << traj_car_x[i] << " y:" << traj_car_y[i] << endl;
    }
    //cout << "Building splines with " << traj_car_x.size() << " x values" << endl;

    //cout << "Contents of traj_car_x:" << endl;
    //for(int i = 0; i < traj_car_x.size(); i++) {
    //    cout << "  " << traj_car_x[i];
    //    if(i % 50 == 49) {
    //        cout << endl;
    //    }
    //}
    //cout << endl << endl;

    tk::spline spline_x_to_y;
    tk::spline spline_x_to_speed;
    spline_x_to_y.set_points(traj_car_x,traj_car_y);
    spline_x_to_speed.set_points(traj_car_x,traj_speed);
    //cout << "Splines built" << endl;
    double lookahead_seconds = 1;
    cout << "Augment path from " << path_x.size() << " to " << path_length << " elements." << endl;
    for(int i = path_x.size(); i < path_length; i++) {
        double last_x = path_x[i-1];
        double last_y = path_y[i-1];
        double last_vx = path_vx[i-1];
        double last_vy = path_vy[i-1];
        double last_ax = path_ax[i-1];
        double last_ay = path_ay[i-1];
        double future_x = last_x + last_vx * lookahead_seconds + 0.5 * last_ax * pow(lookahead_seconds,2);
        double future_y = last_y + last_vy * lookahead_seconds + 0.5 * last_ay * pow(lookahead_seconds,2);
        double future_des_y = spline_x_to_y(future_x);
        double future_des_vy = (future_y - last_y) / lookahead_seconds; // Average vy between now and future
        future_des_vy += (future_des_vy - last_vy); // Should reach average halfway between now and future
        double ay_for_des_y = 2 * (future_des_y - future_y) / pow(lookahead_seconds,2);
        double ay_for_des_vy = (future_des_vy - last_vy) / lookahead_seconds;
        double ay = 0.5 * (ay_for_des_y + ay_for_des_vy);
        // Enforce acceleration and jerk limits on turning. Reserve at least 10% for necessary speed changes.
        if(abs(ay) > 0.9 * acceleration_limit) {
            ay = copysign(0.9 * acceleration_limit, ay);
        }
        if(abs(ay - last_ay) > 0.9 * jerk_limit * delta_t) {
            ay = copysign(0.9 * jerk_limit * delta_t, ay - last_ay) + last_ay;
        }
        double future_des_speed = max(speed_limit, spline_x_to_speed(future_x));
        double future_des_vx = sqrt(future_des_speed * future_des_speed - 2 * max(last_vy * last_vy, future_des_vy * future_des_vy));
        double ax = (future_des_vx - last_vx) / lookahead_seconds;
        // Enforce acceleration and jerk limits on changes to car speed. Reserve at least 1% for margin of error.
        if(sqrt(ay * ay + ax * ax) > 0.99 * acceleration_limit) {
            ax = copysign(sqrt(0.98 * pow(acceleration_limit,2) - pow(ay,2)), ax);
        }
        if(sqrt(pow(ay - last_ay, 2) + pow(ax - last_ax, 2)) > 0.99 * jerk_limit * delta_t) {
            ax = copysign(sqrt(0.98 * pow(jerk_limit*delta_t,2) - pow(ay - last_ay, 2)), ax - last_ax) + last_ax;
        }
        double vx = last_vx + ax * delta_t;
        double vy = last_vy + ay * delta_t;
        double x = last_x + vx * delta_t;
        double y = last_y + vy * delta_t;
        path_x.push_back(x);
        path_y.push_back(y);
        path_vx.push_back(vx);
        path_vy.push_back(vy);
        path_ax.push_back(ax);
        path_ay.push_back(ay);

        cout << "    x:" << path_x[i] << " vx:" << path_vx[i] << " ax:" << path_ax[i]
             << " y:" << path_y[i] << " vy:" << path_vy[i] << " ay:" << path_ay[i] << endl;
    }

    // Convert path_x and path_y back to map coordinates.
    cout << "Convert path from car coordinates back to global coordinates" << endl;
    for(int i = 0; i < path_x.size(); i++) {
        double x = path_x[i];
        double y = path_y[i];
        path_x[i] = x * cos(ref_yaw) - y * sin(ref_yaw) + ref_x;
        path_y[i] = x * sin(ref_yaw) + y * cos(ref_yaw) + ref_y;

        cout << "    car x:" << x << " y:" << y << "  global x:" << path_x[i] << " y:" << path_y[i] << endl;
    }
    cout << "Controller::recalculatePath end" << endl;
}
