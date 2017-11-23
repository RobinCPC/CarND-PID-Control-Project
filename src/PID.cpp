#include "PID.h"

//using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    // initialize PID gain and error
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0;
    i_error = 0;
    d_error = 0;
    return;
}

void PID::UpdateError(double cte) {
    // update CTE (cross-track error) diff_CTE, int_CTE
    d_error = cte - p_error; // cte - prevouse cte
    p_error = cte;
    i_error += cte;
    return;
}

double PID::TotalError() {
    // calculate PID contorl ouptut for steering angle
    double total = -Kp * p_error -Kd * d_error -Ki * i_error;
    return total;
}

