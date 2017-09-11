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
#include "spline.h"
#include "track.h"
#include "controller.h"
#include "prediction.h"
#include "trajectory_planner.h"

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

    // Start in lane 1 (0 is left, 1 is middle, 2 is right)
    int lane = 1;
    double speed_limit_mph = 49.5; // mph
    double speed_limit = speed_limit_mph / 2.24;
    double acceleration_limit = 10.0;
    double jerk_limit = 10.0;

    h.onMessage([&track, &lane, &speed_limit, &acceleration_limit, &jerk_limit, &speed_limit_mph](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {
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
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    int prev_size = previous_path_x.size();

                    if(prev_size > 0) {
                        car_s = end_path_s;
                        car_d = end_path_d;
                    }

                    vector<double> cars_s(0), cars_d(0), cars_vs(0), cars_vd(0);

                    for(int i = 0; i < sensor_fusion.size(); i++) {
                        double d = sensor_fusion[i][6];
                        double sens_x = sensor_fusion[i][1];
                        double sens_y = sensor_fusion[i][2];
                        double sens_vx = sensor_fusion[i][3];
                        double sens_vy = sensor_fusion[i][4];
                        double sens_s = sensor_fusion[i][5];
                        double sens_d = sensor_fusion[i][6];
                        //vector<double> sens_calc_sd = track.xy_to_sd(sens_x,sens_y);
                        //cout << "x=" << sens_x << " y=" << sens_y << " s=" << sens_s << " d=" << sens_d
                        //     << " :: calc s=" << sens_calc_sd[0] << " d=" << sens_calc_sd[1] << endl;
                        vector<double> sens_calc_sdv = track.xyv_to_sdv(sens_x,sens_y,sens_vx,sens_vy);
                        //cout << "x=" << sens_x << " y=" << sens_y << " s=" << sens_s << " d=" << sens_d
                        //     << " :: calc s=" << sens_calc_sdv[0] << " d=" << sens_calc_sdv[1]
                        //     << " vs=" << sens_calc_sdv[2] << " vd=" << sens_calc_sdv[3] << endl;
                        cars_s.push_back(sens_calc_sdv[0]);
                        cars_d.push_back(sens_calc_sdv[1]);
                        cars_vs.push_back(sens_calc_sdv[2]);
                        cars_vd.push_back(sens_calc_sdv[3]);
                    }

                    // Initialize Prediction
                    Prediction current_prediction(cars_s,cars_d,cars_vs,cars_vd);

                    // Move prediction further in time to match end of previous_path
                    Prediction end_of_previous_path_prediction = current_prediction.forward_seconds(previous_path_x.size() * 0.02);

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
                    //cout << "End of prev (" << previous_path_x.size() << ") x:" << end_of_prev_x
                    //     << " y:" << end_of_prev_y << " vx:" << end_of_prev_vx << " vy:" << end_of_prev_vy << endl;

                    vector<double> end_of_prev_sdv = track.xyv_to_sdv(end_of_prev_x,end_of_prev_y,end_of_prev_vx,end_of_prev_vy);
                    //cout << "End of prev s:" << end_of_prev_sdv[0] << " d:" << end_of_prev_sdv[1]
                    //     << " vs:" << end_of_prev_sdv[2] << " vd:" << end_of_prev_sdv[3] << endl;

                    // Create and use trajectory planner
                    double lookahead_seconds = 2;
                    double delta_t = 0.5;
                    TrajectoryPlanner trajectory_planner(end_of_previous_path_prediction,
                                                         end_of_prev_sdv[0], end_of_prev_sdv[1],
                                                         end_of_prev_sdv[2], end_of_prev_sdv[3],
                                                         speed_limit, lookahead_seconds, delta_t);
                    trajectory_planner.calculate(0.1);
                    vector<double> next_s_vals = trajectory_planner.pathS();
                    vector<double> next_d_vals = trajectory_planner.pathD();
                    vector<double> next_speed_vals = trajectory_planner.pathV();

                    // Print out trajectory
                    cout << "Trajectory s/d/v:" << endl;
                    for(int i = 0; i < next_s_vals.size(); i++) {
                        cout << "    " << next_s_vals[i] << "    " << next_d_vals[i] << "    " << next_speed_vals[i] << endl;
                    }
                    cout << "EndOfPrev x:" << end_of_prev_x << " y:" << end_of_prev_y << endl;

                    // Convert trajectory from s,d to x,y
                    vector<double> next_x_vals(0), next_y_vals(0);
                    cout << "Trajectory x/y:" << endl;
                    for(int i = 0; i < next_s_vals.size(); i++) {
                        vector<double> next_xy = track.sd_to_xy(next_s_vals[i], next_d_vals[i]);
                        next_x_vals.push_back(next_xy[0]);
                        next_y_vals.push_back(next_xy[1]);
                        cout << "    " << next_xy[0] << "    " << next_xy[1] << endl;
                    }

                    //cout << "Creating controller" << endl;
                    Controller controller(track, car_x, car_y, deg2rad(car_yaw), speed_limit, acceleration_limit, jerk_limit);
                    //cout << "Updating path history" << endl;
                    controller.updatePathHistory(previous_path_x,previous_path_y);
                    //cout << "Updating trajectory" << endl;
                    controller.updateTrajectory(next_x_vals, next_y_vals, next_speed_vals);
                    //cout << "Finished with controller" << endl;

                    json msgJson;
                    msgJson["next_x"] = controller.getPathX();
                    msgJson["next_y"] = controller.getPathY();
                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    cout << endl << "Sending message: " << msg << endl << endl;

                    //this_thread::sleep_for(chrono::milliseconds(1000));
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
    // program
    // doesn't compile :-(
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
