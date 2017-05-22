/*
 * Copyright (c) 2017 Robert Leishman, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rosflight_utils/simple_pid.h>

namespace rosflight_utils
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
SimplePID::SimplePID(double p, double i, double d, double max, double min, double tau) :
  kp_(p), ki_(i), kd_(d), max_(max), min_(min), tau_(tau)
{
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_state_ = 0.0;
}

//
// Compute the control;
//
double SimplePID::computePID(double desired, double current, double dt, double x_dot)
{
  double error = desired - current;

  // Don't do stupid things (like divide by nearly zero, gigantic control jumps)
  if (dt < 0.00001 || std::abs(error) > 9999999)
  {
    return 0.0;
  }

  if (dt > 1.0)
  {
    // This means that this is a ''stale'' controller and needs to be reset.
    // This would happen if we have been operating in a different mode for a while
    // and will result in some enormous integrator.
    // Or, it means we are disarmed and shouldn't integrate
    // Setting dt for this loop will mean that the integrator and dirty derivative
    // doesn't do anything this time but will keep it from exploding.
    dt = 0.0;
    differentiator_ = 0.0;
  }

  double p_term = error*kp_;
  double i_term = 0.0;
  double d_term = 0.0;


  // Calculate Derivative Term
  if (kd_ > 0.0)
  {
    if (std::isfinite(x_dot))
    {
      d_term = kd_ * x_dot;
    }
    else if (dt > 0.0)
    {
      // Noise reduction (See "Small Unmanned Aircraft". Chapter 6. Slide 31/33)
      // d/dx w.r.t. error:: differentiator_ = (2*tau_ - dt)/(2*tau_ + dt)*differentiator_ + 2/(2*tau_ + dt)*(error -
      // last_error_);
      differentiator_ =
          (2 * tau_ - dt) / (2 * tau_ + dt) * differentiator_ + 2 / (2 * tau_ + dt) * (current - last_state_);
      d_term = kd_* differentiator_;
    }
  }

  // Calculate Integrator Term
  if (ki_ > 0.0)
  {
      integrator_ += dt / 2 * (error + last_error_); // (trapezoidal rule)
      i_term = ki_ * integrator_;
  }

  // Save off this state for next loop
  last_error_ = error;
  last_state_ = current;

  // Sum three terms
  double u = p_term + i_term - d_term;

  return u;

  // Integrator anti-windup
//  double u_sat = saturate(u, min_, max_);
//  if (u != u_sat && std::fabs(i_term) > fabs(u - p_term + d_term))
//  {
//    // If we are at the saturation limits, then make sure the integrator doesn't get
//    // bigger if it won't do anything (except take longer to unwind).  Just set it to the
//    // largest value it could be to max out the control
//    integrator_ = (u_sat - p_term + d_term) / ki_;
//  }

//  return u_sat;
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
  tau_ = tau;
}

}  // namespace relative_nav
