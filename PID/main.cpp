#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid1, pid2, pid3, pid_sp;

  //Initialize the PID instances

	/*
	if (argc != 13) {
		std::cout << "Not enough parameters given!!! " << std::endl;
		return 0;
	}

	double Kp1 = atof(argv[1]);
	double Ki1 = atof(argv[2]);
	double Kd1 = atof(argv[3]);
	double Kp2 = atof(argv[4]);
	double Ki2 = atof(argv[5]);
	double Kd2 = atof(argv[6]);
	double Kp3 = atof(argv[7]);
	double Ki3 = atof(argv[8]);
	double Kd3 = atof(argv[9]);
	double KpSp = atof(argv[10]);
	double KiSp = atof(argv[11]);
	double KdSp = atof(argv[12]);

	pid1.Init(Kp1,Ki1,Kd1);
	pid2.Init(Kp2,Ki2,Kd2);
	pid3.Init(Kp3,Ki3,Kd3);
	pid_sp.Init(KpSp,KiSp,KdSp);

	*/
	
	pid1.Init(0.15,0.005,0.1);
	pid2.Init(0.25,0.005,0.1);
	pid3.Init(2,0.005,0.1);
	pid_sp.Init(5,0.5,0);
	
	double cte_old = 0;
	double cte_old_old = 0;
	
  h.onMessage([&pid1, &pid2, &pid3, &pid_sp, &cte_old, &cte_old_old](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
										
					/*
					* use cascaded PID controller to calculate new steering value according to current error
					*/
					
					pid1.UpdateError(cte);
					
					//add gain scheduling for pid1
					if (fabs(pid1.p_error) > 0.9) {
						pid1.gs = 1.5;
					} else {
						pid1.gs = 1.0;
					}
				
					double pid1_error = pid1.TotalError();
					pid2.UpdateError((cte-cte_old)-pid1_error);
					double pid2_error = pid2.TotalError();
					pid3.UpdateError((cte-2.0*cte_old+cte_old_old)/2.0-pid2_error);
					steer_value = pid3.TotalError();

					//clamp steer_value between -1,1
					if (steer_value > 1) steer_value = 1;
					if (steer_value < -1) steer_value = -1;					

					cte_old_old = cte_old;
					cte_old = cte;



					/*
					* use simple PID controller to calculate new throttle value according to current steering angle
					*/
					
					double throttle_value;
					//input with dead zone
					double pid_sp_in = -fabs(angle)/25.0;
					if (fabs(pid_sp_in) < 0.25) pid_sp_in = 0;
					pid_sp.UpdateError(pid_sp_in);
					throttle_value = 1 - pid_sp.TotalError();
					
					//clamp throttle_value between -0.1,1
					if (throttle_value < -0.1) throttle_value = -0.1;
					if (throttle_value > 1) throttle_value = 1;
					//do not brake at low speed
					if (speed < 40 && throttle_value < 0) throttle_value = 0;
					
					//std::cout << "angle = " << angle << "throttle_value = " << throttle_value << std::endl;




										   										
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
