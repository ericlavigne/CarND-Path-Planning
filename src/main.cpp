#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "track.h"
#include "trajectory_planner.h"
#include "controller.h"

using namespace std;

// for convenience
using json = nlohmann::json;

double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main() {
    uWS::Hub h;

    string map_file_ = "../data/highway_map.csv";
    Track track(map_file_);

    double speed_limit_mph = 47; // mph
    double speed_limit = speed_limit_mph / 2.24;
    double acceleration_limit = 9.5;
    double jerk_limit = 10.0;

    h.onMessage([&track, &speed_limit, &acceleration_limit, &jerk_limit, &speed_limit_mph](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {

        clock_t startClock = clock();

        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;

        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            //cout << endl << "Receiving message: " << s << endl << endl;

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    vector<double> previous_path_x = j[1]["previous_path_x"];
                    vector<double> previous_path_y = j[1]["previous_path_y"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    vector<double> cars_s(0), cars_d(0), cars_vs(0), cars_vd(0);

                    for(int i = 0; i < sensor_fusion.size(); i++) {
                        double d = sensor_fusion[i][6];
                        double sens_x = sensor_fusion[i][1];
                        double sens_y = sensor_fusion[i][2];
                        double sens_vx = sensor_fusion[i][3];
                        double sens_vy = sensor_fusion[i][4];
                        double sens_s = sensor_fusion[i][5];
                        double sens_d = sensor_fusion[i][6];
                        vector<double> sens_calc_sdv = track.xyv_to_sdv(sens_x,sens_y,sens_vx,sens_vy);
                        cars_s.push_back(sens_calc_sdv[0]);
                        cars_d.push_back(sens_calc_sdv[1]);
                        cars_vs.push_back(sens_calc_sdv[2]);
                        cars_vd.push_back(sens_calc_sdv[3]);
                    }

                    while(previous_path_x.size() > 50) {
                        previous_path_x.pop_back();
                        previous_path_y.pop_back();
                    }

                    // Predict where other cars will be at end of previous_path (assuming vs and vd are constant)
                    double pred_time = previous_path_x.size() * 0.02;
                    vector<double> pred_cars_s, pred_cars_d;
                    for(int i = 0; i < cars_s.size(); i++) {
                        pred_cars_s.push_back(cars_s[i] + pred_time * cars_vs[i]);
                        pred_cars_d.push_back(cars_d[i] + pred_time * cars_vd[i]);
                    }

                    // Initialize Trajectory
                    double end_of_prev_x = car_x;
                    double end_of_prev_y = car_y;
                    double end_of_prev_vx = 0;
                    double end_of_prev_vy = 0;
                    if(previous_path_x.size() > 2) {
                        end_of_prev_x = previous_path_x[previous_path_x.size() - 1];
                        end_of_prev_y = previous_path_y[previous_path_x.size() - 1];
                        double before_end_of_prev_x = previous_path_x[previous_path_x.size() - 2];
                        double before_end_of_prev_y = previous_path_y[previous_path_x.size() - 2];
                        end_of_prev_vx = (end_of_prev_x - before_end_of_prev_x) / 0.02;
                        end_of_prev_vy = (end_of_prev_y - before_end_of_prev_y) / 0.02;
                    }

                    vector<double> end_of_prev_sdv = track.xyv_to_sdv(end_of_prev_x,end_of_prev_y,end_of_prev_vx,end_of_prev_vy);

                    // Create and use trajectory planner
                    double lookahead_seconds = 10;
                    TrajectoryPlanner planner(end_of_prev_sdv[0], end_of_prev_sdv[1], end_of_prev_sdv[2], 0.0,
                                              pred_cars_s, pred_cars_d, cars_vs, cars_vd,
                                              speed_limit, acceleration_limit, lookahead_seconds);
                    planner.calculate(0.01);
                    vector<double> next_s_vals = planner.pathS();
                    vector<double> next_d_vals = planner.pathD();
                    vector<double> next_speed_vals = planner.pathV();

                    // Print out trajectory
                    //cout << "Trajectory s/d/v:" << endl;
                    //for(int i = 0; i < next_s_vals.size(); i++) {
                    //    cout << "    " << next_s_vals[i] << "    " << next_d_vals[i] << "    " << next_speed_vals[i] << endl;
                    //}
                    //cout << "EndOfPrev x:" << end_of_prev_x << " y:" << end_of_prev_y << endl;

                    // Convert trajectory from s,d to x,y
                    vector<double> next_x_vals(0), next_y_vals(0);
                    //cout << "Trajectory x/y:" << endl;
                    for(int i = 0; i < next_s_vals.size(); i++) {
                        vector<double> next_xy = track.sd_to_xy(next_s_vals[i], next_d_vals[i]);
                        next_x_vals.push_back(next_xy[0]);
                        next_y_vals.push_back(next_xy[1]);
                        //cout << "    " << next_xy[0] << "    " << next_xy[1] << endl;
                    }

                    double seconds_before_traj = previous_path_x.size() * 0.02;
                    double seconds_per_traj = 1.0;

                    if(previous_path_x.size() < 1) {
                        previous_path_x = {car_x};
                        previous_path_y = {car_y};
                    }
                    
                    Controller pid(previous_path_x, previous_path_y, next_x_vals, next_y_vals, next_speed_vals,
                                   seconds_before_traj, seconds_per_traj);

                    json msgJson;
                    msgJson["next_x"] = pid.pathX();
                    msgJson["next_y"] = pid.pathY();
                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //cout << endl << "Sending message: " << msg << endl << endl;

                    clock_t endClock = clock();
                    double elapsedSeconds = double(endClock - startClock) / CLOCKS_PER_SEC;
                    //cout << "Callback seconds: " << elapsedSeconds << endl;

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
