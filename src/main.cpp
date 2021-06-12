#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <algorithm>

//BEST CTE: 0.0857032 Twiddle Stage: 0 Twiddle VAR: 2 dp: 0.00428238 0.000471013 0.00213227 -0.174496 0 -0.0283478 

//BEST CTE: 0.0840189 Twiddle Stage: 0 Twiddle VAR: 1 dp: 0.00108852 0.000133028 0.000736042 -0.174496 0 -0.0274299 dpi = [0.01, 0.01, 0.01] and K = [0,0,0] tolerance 0.002

//BEST CTE: 0.0369817 Twiddle Stage: 0 Twiddle VAR: 2 dp: 0.00428238 0.000471013 0.00142739 -0.185312 0 -3.0286 dpi = [0.01, 0.01, 0.01] and K = [0,0,-3] tolerance 0.002

// Kp : -0.189594 Ki : 0 Kd : -3.02859 final params


// for convenience
using nlohmann::json;
using std::string;

bool twiddle = false;
bool twiddle_complete = false;
int n = 0;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}
double crosstrack_error = 0;
int main() {
  uWS::Hub h;

  PID pid;
  // multiple runs were needed to complete twiddle
  //pid.Init(-0.091,-0.0005,-1.693);
  //pid.Init(0,0,-3);
  //pid.Init(-0.174496,0,-0.0283478);
  //pid.Init(-0.174496,0,-3); //0.0274299);
  //pid.Init(-0.225,-0.0004,-4);
  //pid.Init(-0.185312 ,0 ,-3.0286);
  // Initialize PID
  pid.Init(-0.189594, 0, -3.02859);
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle = 0.4;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          if(fabs(cte) > 0.5){
            throttle = 0.2;
          }
          if(fabs(cte) > 1.0){
            throttle = 0.1;
          }
          if(fabs(cte) > 1.5){
            throttle = 0.05;
          }


          if(twiddle){
            if(!pid.twiddle_complete){
              if(n==0){
                n+=1;
              }else{
                if(n>=500){
                  crosstrack_error += pow(cte, 2);
                }
              }
              n+=1;
              if(n>1000){
                pid.ContinueTwiddle(crosstrack_error/500);
                crosstrack_error = 0;
                n = 0;
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
            }else if(!twiddle_complete){
              twiddle_complete = true;
              std::cout << "Kp : " << pid.Kp << " Ki : " << pid.Ki << " Kd : " << pid.Kd << std::endl;
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            }
          }

          
          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " throttle " << throttle 
          //           << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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
