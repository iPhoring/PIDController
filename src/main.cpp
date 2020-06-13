#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"


// for convenience 
using nlohmann::json;
using std::string;

//Blobal variable 
bool runTwiddle=true;
int itCounter=0;

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
  } else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


int main(void){
    uWS::Hub h;
    PID pid;
    PID speedPID;
    
    double init_Kp=0.08; //0.09;//0.08;
    double init_Ki=0.0005;//0.0005;//0.0005;
    double init_Kd=7.5;//7.22648; //7.5;

    pid.Init(init_Kp,init_Ki,init_Kd);
    speedPID.Init(0.1,0.002,0.0);

    h.onMessage([&pid,&speedPID](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

        static unsigned int frames = 0; // variable will only be initialized(memory allocation) once in the program.
        static double total_error = 0.0; // value will only be initialized(memory allocation) once in the program.

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
                    double setSpeed = 25.0;
                    double throttle_value=0.3;

                    pid.UpdateError(cte);
                    steer_value = pid.TotalError();
                    if(steer_value > 1.0) steer_value = 1.0; // handling boundary conditions and removing unknown noise
                    if(steer_value < -1.0) steer_value = -1.0; // handling boundary conditions and removing unknown noise

                    //std::cout<<"Value of runTwiddle:"<<runTwiddle<<std::endl;
                    if(runTwiddle){ //only tuning the steering value PID
                        if (frames> 500){
                            std::cout<<"-----Starting Twiddle cycle-----\n\n"<<cte<<std::endl;
                            std::cout<<"Entering Twiddle with cte:"<<total_error<<std::endl;
                            std::cout<<"Kp:[" << pid.Kp << "] Ki:[" << pid.Ki << "] Kd:[" << pid.Kd <<"]"<<std::endl;
                            std::cout<<"p_error:["<<pid.p_error<<"] i_error:["<<pid.i_error<<"] d_error:["<<pid.d_error<<"]"<<std::endl;
                            pid.Twiddle(total_error,pid);
                            itCounter +=1;
                            if (itCounter>4){runTwiddle=false;} // controlling how many times you want to run the twiddle algo
                            std::cout<<"-----Completed Twiddle cycle-----\n\n"<<std::endl;
                        } else {
                            total_error += pow(cte,1);
                        }
                        frames++; //letting the car move few frames before we start tuning the PID
                    }
                    if (!runTwiddle){ //checking PID values after running twiddle for 4 times or when it is off 
                        std::cout<<"=========================================="<<std::endl;
                        std::cout<<"Total cte:["<<cte<<"] Frame count:["<<frames<<"]"<<std::endl;
                        std::cout<<"Kp after Twiddle:[" << pid.Kp << "] Ki after Twiddle:[" << pid.Ki << "] Kd after Twiddle:[" << pid.Kd <<"]"<<std::endl;
                        std::cout<<"Steering Value:[" << steer_value <<"] ";
                        frames++;
                        //std::cout<<"==========================================\n"<<std::endl;
                    }
                    double err_speed = speed - setSpeed;
                    speedPID.UpdateError(err_speed);
                    throttle_value = speedPID.TotalError();
                    if(throttle_value > 1.0) throttle_value = 1.0; // handling boundary conditions and removing unknown noise
                    if(throttle_value < -1.0) throttle_value = -1.0; // handling boundary conditions and removing unknown noise

                    if (!runTwiddle){
                        std::cout<<"Speed Value:[" << throttle_value <<"]" <<std::endl;
                        std::cout<<"==========================================\n"<<std::endl;
                    }
                    
                    // DEBUG
                    //std::cout << "PID: " << pid.Kp <<":" <<pid.Ki <<":"<< pid.Kd << std::endl;
                    //std::cout<<"p_error:"<<pid.p_error<<" i_error:"<<pid.i_error<<" d_error:"<<pid.d_error<<std::endl;
                    //std::cout<<"-----Completed main cycel---------------------------\n\n"<<std::endl;
                    //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value; //0.3;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }// telemetry
            } else {
                // Manual driving 
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            } // if s!="" data checking
        } // checking length of data is 4 or 2 websocket checking
    }); // h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req){
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length){
        ws.close();std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port)){
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl; 
        return -1;
    }

    h.run();
    return 0; //returing main
} // end of main