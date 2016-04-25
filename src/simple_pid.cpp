#include "fcu_common/simple_pid.h"

namespace fcu_common
{

//
// Basic initialization
//
SimplePID::SimplePID()
{
  kp_ = 0.0;
  ki_ = 0.0;
  kd_ = 0.0;
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_state_ = 0.0;
  tau_ = 0.0;
}

//
// Initialize the controller
//
SimplePID::SimplePID(double p, double i, double d, double tau):
    kp_(p),ki_(i),kd_(d),tau_(tau)
{
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_state_ = 0.0;
}

//
// Compute the control;
//
double SimplePID::computePID(double desired, double current, double dt)
{
  double error = desired - current;
  if(dt == 0.0 || std::abs(error) > 9999999)
  {
    last_error_ = error;
    last_state_ = current;
    return 0.0;
  }

  integrator_ += dt/2*(error + last_error_);

  // derivative
  if(dt > 0.0)
  {  
    // Noise reduction (See "Small Unmanned Aircraft". Chapter 6. Slide 31/33)
      // d/dx w.r.t. error:: differentiator_ = (2*tau_ - dt)/(2*tau_ + dt)*differentiator_ + 2/(2*tau_ + dt)*(error - last_error_);
      differentiator_ = (2*tau_ - dt)/(2*tau_ + dt)*differentiator_ + 2/(2*tau_ + dt)*(current - last_state_);
  }

  last_error_ = error;
  last_state_ = current;

  // Note the negative der. term.  This is because now the differentiator is in the feedback loop rather than the forward loop
  return kp_*error + ki_*integrator_ - kd_*differentiator_;
}

double SimplePID::computePIDDirect(double x_c, double x, double x_dot, double dt)
{
  double error = x_c -x;
  if(dt == 0.0 || std::abs(error) > 9999999)
  {
    last_error_ = error;
    last_state_ = x;
    return 0.0;
  }

  integrator_ += dt/2*(error + last_error_);

  last_error_ = error;
  last_state_ = x;

  return kp_*error + ki_*integrator_ - kd_*x_dot;
}

//
// Late initialization or redo
//
void SimplePID::setGains(double p, double i, double d, double tau)
{
  //! \todo Should we really be zeroing this while we are gain tuning?
  kp_ = p;
  ki_ = i;
  kd_ = d;
  //integrator_ = 0.0;
  //differentiator_ = 0.0;
  //last_error_ = 0.0;
  //last_state_ = 0.0;
  tau_ = tau;
}

} // namespace relative_nav
