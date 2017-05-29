#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
// LU module is needed for inverse() to work.
// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1089
#include "Eigen-3.3/Eigen/LU"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Convert from global to local positions.
Eigen::Vector2f global2local(float ptx, float pty,
  float rootx, float rooty, float rooto) {

  Eigen::Vector2f pt;
  pt << ptx, pty;

  Eigen::Vector2f root;
  root << rootx, rooty;

  Eigen::MatrixXf rot(2,2);
  rot << cos(rooto), -sin(rooto), sin(rooto), cos(rooto);
  rot = rot.inverse();

  Eigen::Vector2f pt_local = rot * (pt - root);
  return pt_local;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  int iters = 3;

  h.onMessage([&mpc, &iters](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          std::cout << j << std::endl;
          // j[1] is the data JSON object
          vector<double> ptsx_v = j[1]["ptsx"];
          vector<double> ptsy_v = j[1]["ptsy"];

          double x = j[1]["x"];
          double y = j[1]["y"];
          double psi = j[1]["psi"];
          double psi_unity = j[1]["psi_unity"];
          double v = j[1]["speed"];

          double steer_value;
          double throttle_value;

          for(std::vector<int>::size_type i = 0; i != ptsx_v.size(); i++) {
            Eigen::Vector2f localpt = global2local(
              ptsx_v[i], ptsy_v[i], x, y, psi);
            ptsx_v[i] = localpt[0];
            ptsy_v[i] = localpt[1];
          }
          Eigen::VectorXd ptsx = Eigen::VectorXd::Map(
            ptsx_v.data(), ptsx_v.size());
          Eigen::VectorXd ptsy = Eigen::VectorXd::Map(
            ptsy_v.data(), ptsy_v.size());

          auto coeffs = polyfit(ptsx, ptsy, 3);
          double cte = polyeval(coeffs, 0.0);
          double epsi = psi-atan(coeffs[1]);
          // std::cout << "cte: " << cte << std::endl;

          Eigen::VectorXd state(6);
          // Interestingly, setting x to 4.0 gave better result,
          // possibly due to the latency, that the current position is
          // no longer at 0.0.
          state << 4.0, 0.0, 0.0, v, cte, epsi;

          std::vector<double> x_vals = {state[0]};
          std::vector<double> y_vals = {state[1]};
          std::vector<double> psi_vals = {state[2]};
          std::vector<double> v_vals = {state[3]};
          std::vector<double> cte_vals = {state[4]};
          std::vector<double> epsi_vals = {state[5]};
          std::vector<double> delta_vals = {};
          std::vector<double> a_vals = {};

          // Solve with MPC
          for (size_t i = 0; i < iters; i++) {
            // std::cout << "Iteration " << i << std::endl;

            auto vars = mpc.Solve(state, coeffs);

            x_vals.push_back(vars[0]);
            y_vals.push_back(vars[1]);
            psi_vals.push_back(vars[2]);
            v_vals.push_back(vars[3]);
            cte_vals.push_back(vars[4]);
            epsi_vals.push_back(vars[5]);

            delta_vals.push_back(vars[6]);
            a_vals.push_back(vars[7]);

            state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
          }

          steer_value = delta_vals[0];
          throttle_value = a_vals[0];

          json msgJson;

          msgJson["throttle"] = throttle_value;
          msgJson["steering_angle"] = -steer_value;

          // std::cout << "throttle: " << throttle_value << std::endl;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for(std::vector<int>::size_type i = 0; i != x_vals.size(); i++) {
            mpc_x_vals.push_back(x_vals[i]);
            mpc_y_vals.push_back(y_vals[i]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          next_x_vals = ptsx_v;
          next_y_vals = ptsy_v;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
