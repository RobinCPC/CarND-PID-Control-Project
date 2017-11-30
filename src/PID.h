#ifndef PID_H
#define PID_H
#include <vector>
#include <iostream>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * Parameters for twiddle 
   */
  bool isTwiddle;
  int step;
  int max_step;
  const int buffer_steps = 5; // may have couple steps delay before connecting

  double err;
  double best_err;
  std::vector<double> dp;

  int n_gain;   // indicate which gain to tune
  int n_tune;   // indicate the time of tuning

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, bool tune=false);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Calculate twiddle. 
  */
  std::vector<double> DoTwiddle();
};

#endif /* PID_H */
