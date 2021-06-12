#ifndef PID_H
#define PID_H

#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void ContinueTwiddle(double error);

  bool twiddle_complete;

    double Kp;
    double Ki;
    double Kd;

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 



  double tolerance;
  int twiddle_stage;
  int twiddle_var_num;
  double minimum_crosstrack;
  //std::vector<double> dp = {0.01,0.01,0.01};
  //std::vector<double> dp = {0.00428238,0.000471013,0.00213227};
  //std::vector<double> dp = {0.0225,0.00004,0.4};
  //std::vector<double> dp = {0.01, 0.01, 0.01};
  std::vector<double> dp = {0.00428238 ,0.000471013 ,0.00142739};
};

#endif  // PID_H