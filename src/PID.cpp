#include "PID.h"
#include <iostream>
#include <vector>
#include <limits>

// P and D keep on increasing with 1 and dosen't converge
//BEST CTE: 0.00928177 Twiddle Stage: 0 Twiddle VAR: 0 dp: 0.0847332 0.0254187 0.0847332 K -0.70091 0 -0.727505 dpi = [0.1, 0.1, 0.1] and K = [0,0,0] tolerance 0.2 

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  tolerance = 0.002;
  //tolerance = 0.2;
  twiddle_complete = false;

  twiddle_stage = 0;
  twiddle_var_num = 0;
  minimum_crosstrack = std::numeric_limits<double>::max();
  //minimum_crosstrack = 0.0857032;
}

void PID::UpdateError(double cte) {
  i_error += cte;
  d_error = cte - p_error;
  p_error = cte;
}

double PID::TotalError() {
  return ((Kp * p_error) + (Ki * i_error) + (Kd * d_error));  // total error calc here
}

void PID::ContinueTwiddle(double error){
  std::cout << "BEST CTE: " << minimum_crosstrack << " Twiddle Stage: " << twiddle_stage << " Twiddle VAR: " << twiddle_var_num << " dp: " << dp[0] << " " << dp[1] << " " << dp[2] <<  " " << Kp << " " << Ki << " " << Kd
  << std::endl;
  double sum = 0;
  for(double i : dp){
    sum += i;
  }
  if(sum <= tolerance){
    twiddle_complete = true;
    return;
  }
  std::vector<double> p = {Kp, Ki, Kd};

  if(twiddle_stage==0){
    if(twiddle_var_num == 0)
      Kp -= dp[twiddle_var_num];
    else if(twiddle_var_num == 1)
      Ki -= dp[twiddle_var_num];
    else
      Kd -= dp[twiddle_var_num];
    twiddle_stage += 1;
  }else if(twiddle_stage==1){
    if(error < minimum_crosstrack){
      minimum_crosstrack = error;
      dp[twiddle_var_num] *= 1.1;
      twiddle_var_num += 1;
      twiddle_var_num = twiddle_var_num%3;
      twiddle_stage = 0;
    }else{
      if(twiddle_var_num == 0)
        Kp += 2 * dp[twiddle_var_num];
      else if(twiddle_var_num == 1)
        Ki += 2 * dp[twiddle_var_num];
      else
        Kd += 2 * dp[twiddle_var_num];
      twiddle_stage = 2;
    }
  }else if(twiddle_stage==2){
    if(error < minimum_crosstrack){
      minimum_crosstrack = error;
      dp[twiddle_var_num] *= 1.1;
      twiddle_var_num += 1;
      twiddle_var_num = twiddle_var_num%3;
      twiddle_stage = 0;
    }else{
      if(twiddle_var_num == 0)
        Kp -= dp[twiddle_var_num];
      else if(twiddle_var_num == 1)
        Ki -= dp[twiddle_var_num];
      else
        Kd -= dp[twiddle_var_num];
      dp[twiddle_var_num] *= 0.9;
      twiddle_stage = 0;
      twiddle_var_num += 1;
      twiddle_var_num = twiddle_var_num%3;
      
    }
  }
}