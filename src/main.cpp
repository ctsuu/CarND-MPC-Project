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

#include <ctime>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// Timer
std::clock_t current_time;
std::clock_t previous_time;
double delta_t;

// States
double cte;
double epsi;

double previous_s = 0; // Initial steering value
double previous_a = 0; // Initial acceleration

int counter = 0; // frame counter
int gear; // Num of frames looking ahead, range from 1 to 24
int high_gear = 20; //The highest gear
double limit; // speed limit


bool isBrake = false;

//double average_psi = 0; // Initialize average car heading

//int linewidth;

// Create speed limits

std::vector<double> sx = {-32.16173, -123.3917,122.3383, 158.3417, -146.6317, 107.3083, 102.9283, 156.5083,165.5735, 103.8935,77.04827,98.34827,47.20827};
std::vector<double> sy = {113.361, 33.37102,-79.97897, -17.42898, -141.329, -134.209, 29.88102, 59.27102, 127.2894,158.4294,-1.338982,-42.02898,-148.129};
std::vector<double> sl = {100,  50,    30,      30,        80,    30,       40,       35,       40, 100,28,40,40}; 


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





int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  gear = 5; // look ahead frames



  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          
          

          int current_time = clock();
          delta_t = (current_time-previous_time)*1.0/(CLOCKS_PER_SEC); // sec
          std::cout <<"Counter"<<counter<<"\t"<<"Delta_t = "<<delta_t<< std::endl;

          
          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = 0;
          double throttle_value = 0;
       

          // Up shift, increase smoothness as speed increase
          if (gear< high_gear){
            if (counter % 5 == 0) {
              gear += 1;
            }
          }


          
          // Apply brake and shift down
          if (isBrake){
            if (gear > 2){ 
              gear -= gear-1; // shift to gear 1
              //speed_value = -1.0;
              isBrake = false;
            }
          }
          std::cout << "Gear "<< gear << std::endl;
          
          // Create state vector [x, y, psi, v]
          Eigen::VectorXd state(4);
          // At vehicle coordinate, the vehicle always at (0,0)
 
          state << px, py, psi, v;
          // std::cout << "state: " << std::endl;
          // std::cout << state << std::endl;

          
          // Create actuators vector [steer_value, throttle_value]
          Eigen::VectorXd actuators(2);
           
          actuators << previous_s, previous_a;
          // std::cout << "Actuators: " << std::endl;
          // std::cout << actuators << std::endl;

          // Generate next state, set 25 steps
          int Num = 25;
          
	  //auto next_state = mpc.Model(state, actuators, Num, 0.05);
	  //auto next_state = mpc.Model(state, actuators, Num, delta_t);
          //std::cout << "Predicted Next State "<<Num <<" steps " << std::endl;
          //std::cout << next_state << std::endl;


          // Transform Global Map Way points into vehicle coordinate Matrix
          //Eigen:: MatrixXd car(3, ptsx.size());
          auto tvcm = mpc.Transform(ptsx, ptsy, state);
          //std::cout << "Transformed Vehicle Coordinate Matrix:" << std::endl;
          //std::cout << tvcm << std::endl;

          Eigen::VectorXd next_x(ptsx.size());
          Eigen::VectorXd next_y(ptsx.size());
          
          json msgJson;
         

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int w = 0; w < ptsx.size(); w ++){
            //next_x_vals.push_back(tvcm(0,w));
            //next_y_vals.push_back(tvcm(1,w));
            next_x(w) = tvcm(0,w); // Eigen Vector
            next_y(w) = tvcm(1,w); // Eigen Vector
          }    
          
          // read speed limit with in 10m  
          for (int z = 0; z < sx.size(); z++){
            if (sqrt((px-sx[z])*(px-sx[z])+(py-sy[z])*(py-sy[z]))< 10){
              limit = sl[z];
            }
          }
            
          
          std::cout << "Speed limit: " << limit << std::endl;  




          // Fit polynomial with order 3
                   
          auto coeffs = Eigen::VectorXd(4);

          
          coeffs = polyfit(next_x, next_y, 3);
          //std::cout << "Polynomial Coeffs" << std::endl;
          //std::cout << coeffs << std::endl;
          
          
          next_x_vals.clear();
          next_y_vals.clear();
           
          for (double x = -10; x <= 80; x += 4.0) {
            // use `polyeval` to evaluate the x values.
    	    auto ref = polyeval(coeffs, x);
            next_x_vals.push_back(x);
            next_y_vals.push_back(ref);
            //std::cout << ref << std::endl; 
          }   
          
         

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.

	  double cte = polyeval(coeffs, 0) - 0;
          //std::cout << "CTE at x = 0 point" << std::endl;
          //std::cout << cte << std::endl;

 
	  // Due to the sign starting at 0, the orientation error is -f'(x).
	  // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
	  double epsi = -atan(coeffs[1]);
          // std::cout << "epsi: " << std::endl;
          // std::cout << epsi << std::endl;

          // Six elements car state
          Eigen::VectorXd car_state(6); 
          // px, py, psi, speed, cte, epsi
          car_state << 0, 0, 0, v, cte, epsi; 
          
          // Solve the path and actuation commands
          auto act = mpc.Solve(car_state, coeffs, limit);

          std::vector<double> steering;
          std::vector<double> throttle;
          std::vector<double> mpc_x_vals;
          std::vector<double> mpc_y_vals;

 
          for (int i = 0; i < Num-1; i++){
            steering.push_back(act(0,i));
            throttle.push_back(act(1,i));
            mpc_x_vals.push_back(act(2,i));
            mpc_y_vals.push_back(act(3,i));
          }

          steer_value = -steering[gear];
          throttle_value = throttle[gear];

          if (throttle_value <0 or cte*cte >100){
            isBrake = true;
          }

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          
 
          previous_s = steer_value;
          previous_a = throttle_value; 
                
   

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //        std::cout << msg << std::endl;
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


          previous_time = current_time;
          counter += 1; 


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
