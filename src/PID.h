#ifndef PID_H
#define PID_H
#include <uWS/uWS.h>

class PID {
 public:
  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double previous_cte;
  double filtered_total_error;
  /*
   * Coefficients
   */
  double Kp;
  double Ki;
  double Kd;
  double Tf;
  double dt;

  //Calibration knobs
  double dKp;
  double dKi;
  double dKd;
  /*
   * run Twiddle to calibrate parameters if false
   */
  bool isCalibrated;
  double calibrationTolerance;
  //Number of successful steps for each run
  unsigned int succSteps;
//Private variables for Twiddle, should not be manipulated form outside
 private:

  //Total error of each run for calibration
  double calError;
  double speed,calSpeed;

  //Checks for different parameters calibration
  bool p_check,i_check; //d_check;
  int p_step, i_step, d_step;
  //Best error so far
  double best_error;

 public:
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
  void Init(double Kp, double Ki, double Kd, double Tf, bool isCalibrated);

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte, double dt, double speed);

  /*
   * Calculate the total PID error.
   */
  double TotalError();

  /*
   * Twiddle algorithm for parameter tuning of controller
   */
  void Twiddle();

  /*
   * Restart controller to start another step of twiddle
   */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);
};

#endif /* PID_H */
