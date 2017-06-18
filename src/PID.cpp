#include "PID.h"
#include <uWS/uWS.h>
#include <cmath>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double Tf, bool isCalibrated)
{

  this->isCalibrated = isCalibrated;

  //Reset all values for calibration
  if(!this->isCalibrated)
  {
    this->Kp = 0.0;
    this->Ki = 0.0;
    this->Kd = 0.0;
    this->Tf = 0.0;

    calibrationTolerance = 0.1;
    calError = 0.0;
    succSteps = 0;
    dKp = 1.0;
    dKi = 1.0;
    dKd = 1.0;

    p_check = false;
    i_check = false;
//    d_check = false;
    p_step = 0;
    i_step = 0;
    d_step = 0;

    best_error = std::numeric_limits<double>::max();

  }
  //Set values if already calibrated
  else
  {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->Tf = Tf;
  }

  p_error = 0;
  i_error = 0;
  d_error = 0;

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

  //Low-pass filter to smooth the output
  if(abs(Tf) < 0.00001)
  {
    filtered_total_error = err;
  }
  else
  {
    filtered_total_error = filtered_total_error + (dt/Tf)*(err - filtered_total_error);
  }

  //Bound the control signals to its limits
  filtered_total_error = (filtered_total_error > 1) ? 1 : filtered_total_error;
  filtered_total_error = (filtered_total_error < -1) ? -1 : filtered_total_error;

  if(!this->isCalibrated)
  {
     calError += p_error*p_error;
     succSteps++;
  }

  return  filtered_total_error;
}

void PID::Twiddle()
{
  double tol = dKp+dKi+dKd;
  calError /= succSteps;
  //Termination condition
  if(tol < calibrationTolerance)
  {
    isCalibrated = true;
    cout << "Best Coeffs found as: P=" << Kp << " I=" << Ki << " D=" << Kd << endl;
  }
  else
  {
    // Tuning P coeff
    if(!p_check)
    {

      if(p_step == 0)
      {
        this->Kp += dKp;
        p_step = 1;
      }
      else if(p_step == 1 && calError < best_error)
      {
        dKp *= 1.1;
        best_error = calError;
        p_step = 0;
        p_check = true;
      }
      else if (p_step == 1 && calError >= best_error)
      {
        this->Kp -= 2*dKp;
        p_step = 2;
      }
      else if(p_step == 2 && calError < best_error)
      {
        dKp *= 1.1;
        best_error = calError;
        p_step = 0;
        p_check = true;
      }
      else if (p_step == 2 && calError >= best_error)
      {
        this->Kp += dKp;
        dKp *= 0.9;
        p_step = 0;
        p_check = true;
      }
    }
    // Tuning I coeff
    else if(!i_check)
    {

      if(i_step == 0)
      {
        this->Ki += dKi;
        i_step = 1;
      }
      else if(i_step == 1 && calError < best_error)
      {
        dKi *= 1.1;
        best_error = calError;
        i_step = 0;
        i_check = true;
      }
      else if (i_step == 1 && calError >= best_error)
      {
        this->Ki -= 2*dKi;
        i_step = 2;
      }
      else if(i_step == 2 && calError < best_error)
      {
        dKi *= 1.1;
        best_error = calError;
        i_step = 0;
        i_check = true;
      }
      else if (i_step == 2 && calError >= best_error)
      {
        this->Ki += dKi;
        dKi *= 0.9;
        i_step = 0;
        i_check = true;
      }
    }
    // Tuning D coeff
    else// if(!d_check)
    {

      if(d_step == 0)
      {
        this->Kd += dKd;
        d_step = 1;
      }
      else if(d_step == 1 && calError < best_error)
      {
        dKd *= 1.1;
        best_error = calError;
        d_step = 0;
        p_check = false;
        i_check = false;
//        d_check = true;
      }
      else if (d_step == 1 && calError >= best_error)
      {
        this->Kd -= 2*dKd;
        d_step = 2;
      }
      else if(d_step == 2 && calError < best_error)
      {
        dKd *= 1.1;
        best_error = calError;
        d_step = 0;
        p_check = false;
        i_check = false;
//        d_check = true;
      }
      else if (d_step == 2 && calError >= best_error)
      {
        this->Kd += dKd;
        dKd *= 0.9;
        d_step = 0;
        p_check = false;
        i_check = false;
//        d_check = true;
      }
    }


    //this->Tf = 0.0; //TODO: smooth factor calibration

    calError = 0.0;
    succSteps = 0;

  }

}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){

  Twiddle();
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
