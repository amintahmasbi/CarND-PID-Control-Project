#include "PID.h"
#include <uWS/uWS.h>
#include <cmath>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double Tf)
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->Tf = Tf;
  p_error = 0;
  d_error = 0;
  i_error = 0;
  previous_cte = 0;
  filtered_total_error = 0;

}

void PID::UpdateError(double cte, double dt) {

  p_error = cte;
  d_error = (cte - previous_cte)/dt;
  i_error += cte*dt;
  this->dt = dt;
  previous_cte = cte;

}

double PID::TotalError() {
  double err = - Kp*p_error - Ki*i_error - Kd*d_error;

  if(abs(Tf) < 0.00001)
  {
    filtered_total_error = err;
  }
  else
  {
    filtered_total_error = filtered_total_error + (dt/Tf)*(err - filtered_total_error);
  }

  filtered_total_error = (filtered_total_error > 1) ? 1 : filtered_total_error;
  filtered_total_error = (filtered_total_error < -1) ? -1 : filtered_total_error;

  return  filtered_total_error;
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
