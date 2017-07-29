#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Kd_ = Kd;
    Ki_ = Ki;
    prev_cte = 0;
    int_cte = 0;
    error = 0;
}

void PID::Init(std::vector<double> K_vec){
    Init(K_vec[0],K_vec[1],K_vec[2]);
}

double PID::ControlSteering(double cte){
    UpdateError(cte);
    return -Kp_*cte - Kd_*diff_cte - Ki_*int_cte;
}

void PID::UpdateError(double cte) {
    diff_cte = cte - prev_cte;
    prev_cte = cte;
    int_cte += cte;
    error += cte*cte;
}

double PID::TotalError() {
    return error;
}

