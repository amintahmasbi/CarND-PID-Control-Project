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
  double filtered_total_error;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  double Tf;


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
  void Init(double Kp, double Ki, double Kd, double Tf);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Restart(uWS::WebSocket<uWS::SERVER> ws);
};

#endif /* PID_H */
