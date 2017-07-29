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

enum TWIDDLE_STATE_E{
    POSITIVE_TEST_NEEDED = 0,
    TESTING_POSITIVE_STEP,
    TESTING_NEGATIVE_STEP,
    ITERATION_DONE
};

int main()
{
    uWS::Hub h;

    PID pid;
    TWIDDLE_STATE_E twi_state = POSITIVE_TEST_NEEDED;
    auto RUN_TWIDDLE = false;
    // Initialize Kp, Ki, Kd
    // Manually tuned: 0.8, 0.0004, 4.5
    // 500steps optimized : 0.860978 0.000360201 4.14125 (30.4354 best error)
    // 1500steps optimized:
    std::vector<double> K_vec{0.860978,0.000360201,4.14125};
    std::vector<double> K_steps{0.04,0.0001,0.2};
    pid.Init(K_vec);
    unsigned int K_idx = 0;
    double best_error = 10E10;

    h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
                      * TODO: Calcuate steering value here, remember the steering value is
                      * [-1, 1].
                      * NOTE: Feel free to play around with the throttle and speed. Maybe use
                      * another PID controller to control the speed!
                      */
                    steer_value = pid.ControlSteering(cte);
                    // DEBUG
                    static int step = -1; // init training case
                    ++step;
                    //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
                    //std::cout << "Step Number: " << step << std::endl;

                    // Train only over the first steps and send a reset
                    if(RUN_TWIDDLE and (step == 500 or step == 0)){
                        std::string reset_msg = "42[\"reset\",{}]";
                        ws.send(reset_msg.data(),reset_msg.length(),uWS::OpCode::TEXT);
                        // Check if training shall be stopped
                        //auto sum_step = std::accumulate(K_steps.begin(),K_steps.end(),0);
                        if(true){
                            auto new_state = twi_state;
                            switch (twi_state) {
                            case POSITIVE_TEST_NEEDED:
                                std::cout << "start pos. step" << std::endl;
                                K_vec[K_idx] += K_steps[K_idx];
                                new_state = TESTING_POSITIVE_STEP;
                                break;
                            case TESTING_POSITIVE_STEP:
                                std::cout << "Total error: " << pid.TotalError()
                                          << " Best: "<< best_error <<std::endl;
                                if(pid.TotalError() < best_error){
                                    std::cout << "pos. step improved " << std::endl;
                                    // Improved so accelerate search step
                                    best_error = pid.TotalError();
                                    K_steps[K_idx] *= 1.1;
                                    // No need to search in negative step
                                    new_state = ITERATION_DONE;
                                }else{
                                    std::cout << "neg. step needed " << std::endl;
                                    // Didn't imrove: do negative step
                                    K_vec[K_idx] -= 2*K_steps[K_idx];
                                    new_state = TESTING_NEGATIVE_STEP;
                                    // Reset for negative step
                                    pid.Init(K_vec);
                                    step = 0;
                                }
                                break;
                            case TESTING_NEGATIVE_STEP:
                                std::cout << "Total error: " << pid.TotalError()
                                          << " Best: "<< best_error <<std::endl;
                                // 3. Check negative step result
                                if(pid.TotalError() < best_error){
                                    // Improved so accelerate search step
                                    std::cout << "neg. step improved perf " << std::endl;
                                    best_error = pid.TotalError();
                                    K_steps[K_idx] *= 1.1;
                                }else{
                                    // Reset previous value and reduce step size as both positive
                                    // and negative didn't improve performances
                                    std::cout << "neg. step didn't improve: reduce step size" << std::endl;
                                    K_vec[K_idx] += K_steps[K_idx];
                                    K_steps[K_idx] *= 0.9;
                                }
                                new_state = ITERATION_DONE;
                                break;
                            case ITERATION_DONE:
                                // Handled below
                                break;
                            default:
                                break;
                            }
                            twi_state = new_state;

                            // Reset
                            if(twi_state == ITERATION_DONE){
                                std::cout << "Current K " << K_vec[0] <<" "<< K_vec[1] <<" "<< K_vec[2] << std::endl;
                                std::cout << "Current Step " << K_steps[0] <<" "<< K_steps[1] <<" "<< K_steps[2] << std::endl;
                                if(K_idx == 2){
                                    K_idx = 0;
                                }else{
                                    K_idx++;
                                }
                                // Get ready for another try
                                std::cout << "Training K index " << K_idx << std::endl;
                                twi_state = POSITIVE_TEST_NEEDED;
                                pid.Init(K_vec);
                                step = -1;
                            }

                        }else{
                            std::cout << "Final K: " << K_vec[0] <<" "<< K_vec[1] <<" "<< K_vec[2] << std::endl;
                            // shutdown as optimzation is over
                            std::terminate();
                        }

                    }else{

                        json msgJson;
                        msgJson["steering_angle"] = steer_value;
                        msgJson["throttle"] = 0.3;
                        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }

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
