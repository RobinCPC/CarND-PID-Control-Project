#include "PID.h"
#include <limits>

//using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
    best_err = std::numeric_limits<double>::max(); // initial as negative
    dp.push_back(0.1);    // kp term
    dp.push_back(0.005);    // ki term
    dp.push_back(1.0);      // kd term
    n_gain = 0;
    n_tune = 0;
    max_step = 25;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool tune) {
    // initialize PID gain and error
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0;
    i_error = 0;
    d_error = 0;
    // twiddle parameer
    isTwiddle = tune;
    step  = 0;
    err = 0;
    return;
}

void PID::UpdateError(double cte) {
    // update CTE (cross-track error) diff_CTE, int_CTE
    d_error = cte - p_error; // cte - prevouse cte
    p_error = cte;
    i_error += cte;
    ++step;
    if (step > max_step && isTwiddle)
        err += cte * cte;
    return;
}

double PID::TotalError() {
    // calculate PID contorl ouptut for steering angle
    double total = -Kp * p_error -Kd * d_error -Ki * i_error;
    return total;
}

std::vector<double> PID::DoTwiddle(){
    std::vector<double> gain{Kp, Ki, Kd};
    if(best_err == std::numeric_limits<double>::max()){
      best_err = err;
      err = 0;
      n_gain = 0;
      n_tune = 0;
    }
    // start to tune depend on error n_tune
    while(dp[0]+dp[1]+dp[2] > 0.02){
      if(n_tune == 0){
        // start to tune gain
        gain[n_gain] += dp[n_gain];
        err = 0;
        n_tune = 1;
        return gain;

      }else if(n_tune == 1){
        // after firt tune
        if( err < best_err){
          best_err = err;
          dp[n_gain] *= 1.1;
          err = 0;
          // change gain to tune
          n_tune = 0;
          n_gain = (n_gain + 1) % 3;
        }else{
          gain[n_gain] -= 2*dp[n_gain];
          err = 0;
          n_tune = 2;
          return gain;
        } 

      }else if(n_tune == 2){
        // after second tune
        if( err < best_err){
          best_err = err;
          dp[n_gain] *= 1.1;
          err = 0;
          // change gain to tune
          n_tune = 0;
          n_gain = (n_gain + 1) % 3;
        }else{
          // tuning step too large
          gain[n_gain] += dp[n_gain];
          dp[n_gain] *= 0.9;
          err = 0;
          // change gain to tune
          n_tune = 0;
          n_gain = (n_gain + 1) % 3;
        }
        
      }
    }
    std::cout << "=====Finish Tuning !!!=====" << std::endl;
    return gain;
}
