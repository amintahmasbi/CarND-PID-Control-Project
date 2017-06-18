#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */

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

int main()
{
  uWS::Hub h;

  PID steer_pid;
  PID speed_pid;

  // To run Twiddle for parameter calibration of PID controller
  // Keep this as TRUE if your controller is already calibrated
  bool is_steer_calibrated = false;
  bool is_speed_calibrated = true; //TODO: should be implemented

  // Initialize the pid variable.
  // To find initial values, run calibration only once -> is_steer_calibrated = false
  if (is_steer_calibrated && is_speed_calibrated)
  {
    steer_pid.Init(0.0,0.0,0.0,0.0,is_steer_calibrated);
    speed_pid.Init(0.0,0.0,0.0,0.0,is_speed_calibrated);
  }
  else //First step of calibration: Set Initial values to zero
  {
    steer_pid.Init(0.0,0.0,0.0,0.0,is_steer_calibrated);
    speed_pid.Init(0.0,0.0,0.0,0.0,is_speed_calibrated);
  }

  //This two variables are used by controller for delta_t
  clock_t current_time = clock();
  clock_t previous_time = clock();





  h.onMessage([&steer_pid, &speed_pid, &current_time, &previous_time]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          //calculate delta_t between the last two data received from simulator
          current_time = clock();
          double diff_time = double(current_time - previous_time);
          double dt = (diff_time <= 0)? 1.0/CLOCKS_PER_SEC:diff_time/CLOCKS_PER_SEC;
          previous_time = current_time;
          /*
           * Calcuating steering value here, remember the steering value is
           * [-1, 1].
           */
          double steer_value;

          //Update error and calculate steering
          if(steer_pid.isCalibrated)
          {
            steer_pid.UpdateError(cte,dt);
            steer_value = steer_pid.TotalError();
          }
          else
          {
            steer_pid.UpdateError(cte,dt);
            steer_value = steer_pid.TotalError();

            if (abs(cte) > 2.3 || steer_pid.succSteps >= 1000) // Passed the Lane margin in simulator
            {
              // DEBUG
              std::cout << "P=" << steer_pid.Kp << " I=" << steer_pid.Ki << " D=" << steer_pid.Kd << " steps= " << steer_pid.succSteps<< std::endl;
              //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " deltaT: " << dt << std::endl;
              //Restart the simulator and run Twiddle to calibrate
              steer_pid.Restart(ws);
            }
          }
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
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
