#include <iostream>
#include <limits>
#include "PID.h"
using namespace std;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  prev_cte = 0.0;
  int_cte = 0.0;
  first = true;
  twiddle = false;
  twiddle_steps = 0;
  max_twiddle_steps = 2000;
  tol = min(Kp, min(Ki, Kd)) * 0.0001;
  p = {Kp, Ki, Kd};
  dp = {0.1*Kp, 0.1*Ki, 0.1*Kd};
  
  best_error = std::numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  if(first){
    prev_cte = cte;
    first = false;
  }
  double diff_cte = cte - prev_cte;
  int_cte += cte;
  prev_cte = cte;
  double error;
  
  if(twiddle && twiddle_steps < max_twiddle_steps){
    twiddle_steps += 1;
    if(dp[0] + dp[1] + dp[2] > tol){
      for(int i=0; i<3; i++){
        p[i] += dp[i];
        error = -p[0]*cte - p[1]*int_cte - p[2]*diff_cte;
        if(error < best_error){
          cout << "Step: " << twiddle_steps << ": best_error: " << best_error << "; " << dp[0] << ", " << dp[1] << ", " << dp[2] << endl;
          best_error = error;
          dp[i] *= 1.1;
        }
        else{
          p[i] -= 2*dp[i];
          error = -p[0]*cte - p[1]*int_cte - p[2]*diff_cte;
          if(error < best_error){
            cout << "Step: " << twiddle_steps << ": best_error: " << best_error << "; " << dp[0] << ", " << dp[1] << ", " << dp[2] << endl;
            best_error = error;
            dp[i] *= 1.1;
          }
          else{
            p[i] += dp[i];
            dp[i] *= 0.9;
          }
        }
      }
    }
  }
  
  //p_error = -Kp * cte;
  //i_error = -Ki * int_cte;
  //d_error = -Kp * diff_cte;
  p_error = -p[0] * cte;
  i_error = -p[1] * int_cte;
  d_error = -p[2] * diff_cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return p_error + i_error + d_error;  // TODO: Add your total error calc here!
}

void PID::SetTwiddle(bool b_twiddle){
  twiddle = b_twiddle;
}