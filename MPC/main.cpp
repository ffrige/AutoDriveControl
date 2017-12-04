#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180.0; }
double rad2deg(double x) { return x * 180.0 / pi(); }

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

int main(int argc, char *argv[]) {

  uWS::Hub h;
	
	//define weights for cost function of MPC
	double w0,w1,w2,w3,w4,w5;
	
	const double Lf = 2.67;

	if (argc != 7) {
		std::cout << "Not enough weights provided, using standard values! " << std::endl;
		w0 = 10; //cte
		w1 = 1000; //epsi
		w2 = 0.01; //speed
		w3 = 10; //steering
		w4 = 100; //steering'
		w5 = 1000; //steering''
	} else {
		w0 = atof(argv[1]);
		w1 = atof(argv[2]);
		w2 = atof(argv[3]);
		w3 = atof(argv[4]);
		w4 = atof(argv[5]);
		w5 = atof(argv[6]);
	}


	Eigen::VectorXd weights(6);
	weights << w0,w1,w2,w3,w4,w5;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc, &weights, &Lf](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
					
					//waypoints of track - ideal path
					//6 points in global coordinate system
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
					
					//current car status
					//position in m
					//orientation in rad
					//speed in mph!
					double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
					v*= 0.44704; //convert speed from mph to m/s
					
          double steering_act = j[1]["steering_angle"];
					double throttle_act = j[1]["throttle"];

					//convert path points into car coordinate system
					Eigen::VectorXd pointsX(int(ptsx.size())), pointsY(int(ptsy.size()));
					for (unsigned int i=0;i<ptsx.size();++i) {
						double x_loc = ptsx[i] - px;
						double y_loc = ptsy[i] - py;
						ptsx[i] = x_loc * cos(psi) + y_loc* sin(psi);
						ptsy[i] = - x_loc * sin(psi) + y_loc * cos(psi);
						pointsX[i] = ptsx[i]; 
						pointsY[i] = ptsy[i];
					}
					
					//fit a cubic polynomial through the path points
					Eigen::VectorXd coeffs = polyfit(pointsX, pointsY, 3);
          					
					//calculate actual error
					double cte = coeffs[0];
					double epsi = -atan(coeffs[1]);

					//add latency to actual state
					//assuming car moves at constant speed along x axis  
					double latency = 0.1; //in seconds
					px = v * latency;
					py = 0;
					psi = -v * steering_act / Lf * latency;
					v += throttle_act * latency;
					cte += v * sin(epsi) * latency;
					epsi += psi;
					
					//build actual state vector in car coordinate system
					Eigen::VectorXd state(6);
					state << px, py, psi, v, cte, epsi;
					
					//use MPC to calculate actions (steering angle and throttle)
					vector<double> result = mpc.Solve(state, coeffs, weights);
					
          double steer_value = -result[0]/ deg2rad(25); //negative because simulator uses different angle orientation
          double throttle_value = result[1];

          json msgJson;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

					int N = (int)((result.size()-2)/2);

					for(int i=2; i<2+N*2;i+=2) {
						mpc_x_vals.push_back(result[i]);
						mpc_y_vals.push_back(result[i+1]);
					}
					
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          
					//Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
					
					next_x_vals = ptsx;
					next_y_vals = ptsy;
					
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(int(latency*1000.0)));
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
