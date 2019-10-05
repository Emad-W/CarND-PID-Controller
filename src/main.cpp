#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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

int main() {
  uWS::Hub h;
  
  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  /*Creates robot and initializes location/orientation to 0, 0, 0*/
//   P_init = 0.0;
//   I_init = 0.0;
//   D_init = 0.0;
//   pid.Init(0.0, 0.0, 0.0);
//   pid.Init(0.2, 0.004, 3.0);
  
  
  bool twiddle = false;
  double dp_p = 0.0138456;
  double dp_i = 0.000021687;
  double dp_d = 0.076287;
  int n = 0;
  int n_max = 100;
  int it = 0;
  int id =0;
  int flag = 0;
  double err = 0.0;
  double total_err = 0.0;
  double best_err = 9999.0;
  double PID_value = 0.0;
  double PID_error = 0.0;
  double tol = 0.01;  
  
  if(twiddle == true) {
    pid.Init(0.141191, 0.00070935, 4.13358);
  }
  else 
  {
//     pid.Init(0.06, 0.00031, 1.29);
    pid.Init(0.159952, 0.000675757, 4.33827);
  }


  h.onMessage([&pid, &twiddle, &dp_p, &dp_i, &dp_d, &n, &n_max, &it, &id, &flag, &err, &total_err, &best_err, &PID_value, &PID_error, &tol ](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
//           double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle_value = 0.4;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
//           std::cout << "PID Values   " << " Kp: " << pid.Kp << " Ki: " << pid.Ki << " Kd: " << pid.Kd << std::endl;
//           std::cout << "PID error   " << " p_error: " << dp_p << " i_error: " << dp_i << " d_error: " << dp_d << std::endl;
          
          
          if (twiddle == true)
          {
//             std::cout << "PID Init Values   " << " Kp: " << pid.Kp << " Ki: " << pid.Ki << " Kd: " << pid.Kd << std::endl;
//             std::cout << "PID Init error   " << " p_error: " << dp_p << " i_error: " << dp_i << " d_error: " << dp_d << std::endl;

            if(n == 0)
            {
//               std::cout << " N = 0 " << std::endl;
              
              pid.Init(pid.Kp, pid.Ki, pid.Kd);
              total_err = 0;
            }
            
            // Update error values with cte
            pid.UpdateError(cte);

            // Calculate steering value (if reasonable error, returns between [-1, 1])
            steer_value = pid.TotalError();
            
//             if (speed < 15)
//             throttle_value = (1 - std::abs(steer_value)) * 0.2 + 0.2;
//             else
//               throttle_value = 0;

            n ++;
            if (n > n_max)
              total_err += pow(cte,2);
            
            
            
//             std::cout << "N value" << n << std::endl;
            if (n > 2*n_max)
            {

              std::cout << "Optimize" << std::endl;

              
              err = total_err/n_max;
              if (id > 2)
                id = 0;
              int id_prev = id;
              switch (id_prev)
              {
                case 0:
                  std::cout << "case P" << std::endl;
                  PID_value = pid.Kp;
                  PID_error = dp_p;
                  break;
                  
                case 1:
                  std::cout << "case I" << std::endl;
                  PID_value = pid.Ki;
                  PID_error = dp_i;
                  break;
                  
                case 2:
                  std::cout << "case D" << std::endl;
                  PID_value = pid.Kd;
                  PID_error = dp_d;
                  break;
                  
                //default:

              }
              
              std::cout << "PID Values   " << " Kp: " << pid.Kp << " Ki: " << pid.Ki << " Kd: " << pid.Kd << std::endl;
              std::cout << "PID error   " << " p_error: " << dp_p << " i_error: " << dp_i << " d_error: " << dp_d << std::endl;
              
              
              if (flag == 0)
              {
                PID_value += PID_error;
                flag = 1;
                std::cout << "1st Case best error " << best_err << std::endl;
              }

              else
              {
                if ((err < best_err) && (flag ==1))
                {
                  
                  best_err = err;
                  PID_error *= 1.1;
                  flag = 0; // we are going in the correct direction and error is decreasing
                  id ++;
                  std::cout << "2nd Case best error " << best_err << std::endl;
                }
                else  // wrong direction due to p_error
                {
                  if (flag == 1)
                  {
                    flag = 2;
                    PID_value -= 2*PID_error;
                    std::cout << "3rd Case best error " << best_err << std::endl;
                  }
                  else if ((err < best_err) && (flag == 2))
                  {
                    best_err = err;
                    PID_error *= 1.1;
                    flag = 0;
                    id ++;
                    std::cout << "4th Case best error " << best_err << std::endl;
                  }
                  else
                  {
                    PID_value += PID_error;
                    PID_error *= 0.9;
                    flag = 0;
                    id ++;

                  }

                }
              }
              
              switch (id_prev)
              {
                case 0:
                  pid.Kp = PID_value;
                  dp_p = PID_error;
                  break;
                  
                case 1:
                  pid.Ki = PID_value;
                  dp_i = PID_error;
                  break;
                  
                case 2:
                  pid.Kd = PID_value;
                  dp_d = PID_error;
                  break;
                  
                //default:

              }
              
              std::cout << "PID updated Values   " << " Kp: " << pid.Kp << " Ki: " << pid.Ki << " Kd: " << pid.Kd << std::endl;
              std::cout << "PID updated error   " << " p_error: " << dp_p << " i_error: " << dp_i << " d_error: " << dp_d << std::endl;
              
              n = 0;
              it ++;

              std::cout << "current error " << err << " Best error " << best_err << std::endl;
              double sum_dp;  
              sum_dp = dp_p+dp_i+dp_d;
              
              if(sum_dp < tol) 
              {
              
                std::cout << "Best PID: " << pid.Kp << " " << pid.Ki << " " << pid.Kd << " ";
//                 ws.close();
                std::cout << "Disconnected" << std::endl;
              } 
              else 
              {
                std::cout << "reset" << std::endl;
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
              
            } 
            else 
            {
              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
           
          } 
   
          
          else //twiddle if
          { 
            // Update error values with cte
            pid.UpdateError(cte);

            // Calculate steering value (if reasonable error, returns between [-1, 1])
            steer_value = pid.TotalError();
                        
            // Formula below switches to between [0, 1], larger steering angle means less throttle
            if (speed < 40)
              throttle_value = (1 - std::abs(steer_value)) * 0.2 + 0.2;
            else
              throttle_value = 0;
            // DEBUG

            std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                      << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          } // end of twiddle else
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
  } 
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}