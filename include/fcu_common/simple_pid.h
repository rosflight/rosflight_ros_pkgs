/*!
 *  \brief This file defines a simple PID controller to be used by other classes to implement a PID control loop
 *  \author Robert Leishman
 *  \date Dec. 2013
*/

#ifndef ROTOR_CONTROLLER_SIMPLE_PID_H
#define ROTOR_CONTROLLER_SIMPLE_PID_H

#include <cmath>
#include <ros/ros.h> // included temporarily for debug statements

namespace fcu_common
{

/*!
 * \brief The simplePID class is a basic, tried and true PID controller.  Only P (proportional) gains are
 *  necessary, the I (integral) and D (derivative) default to zero. The I control is computed using a
 *  first-order numerical integral of the error, the derivative is the numerical first-order derivative
 *  of the error.  Due to these crude integration techniques, it is best if the control be computed fast
 *  (i.e. small dt).
 */
class SimplePID
{
public:
  /*!
   * \brief SimplePID is the basic initializer;
   */
  SimplePID();


  /*!
   * \brief SimplePID initializes the class.
   * \param p the proportional controller gain (required)
   * \param i the integral controller gain (defaults to zero)
   * \param d the derivative controller gain (defaults to zero)
   * \param imin the min value accepted in the output of the integral control
   * \param imax the max value accepted in the output of the integral control (saturation for integrator windup)
   * \param tau band limited differentiator to reduce noise
   */
  SimplePID(double p, double i = 0.0, double d = 0.0, double tau = 0.15);


  /*!
   * \brief computePID computes the PID control for the given error and timestep (since the last control was computed!)
   * \param p_error is the "position" error (or whatever variable you are controlling)
   * \param dt is the timestep since the last control was computed.
   * \return the control command
   */
  double computePID(double desired, double current, double dt);

  /*!
   * \brief computePIDDirect computes PID control when given the derivative of the state
   * \param x_c commanded state
   * \param x current state
   * \param x_dot derivative of current state
   * \param dt timestep since last control
   * \return the control command
   */
  double computePIDDirect(double x_c, double x, double x_dot, double dt);


  /*!
   * \brief setgains is used to set the gains for a controller after it's been initialized.  It will rewrite
   *  whatever is already there!
   * \param p the proportional controller gain (required)
   * \param i the integral controller gain (defaults to zero)
   * \param d the derivative controller gain (defaults to zero)
   * \param imin the min value accepted in the output of the integral control
   * \param imax the max value accepted in the output of the integral control (saturation for integrator windup)
   * \param tau band limited differentiator to reduce noise
   */
  void setGains(double p, double i = 0.0, double d = 0.0, double tau = 0.15);



  /*!
   * \brief clearIntegrator allows you to clear the integrator, in case of integrator windup.
   */
  void clearIntegrator()
  {
      integrator_ = 0.0;
  }

protected:
  double kp_;  //!< the proportional gain
  double ki_;  //!< the integral gain (zero if you don't want integral control)
  double kd_;  //!< the derivative gain (zero if you don't want derivative control)
  double integrator_; //!< the integral of p_error
  double differentiator_; //!< used for noise reduced differentiation
  double last_error_;  //!< the last p_error, for computing the derivative;
  double last_state_;  //!< the last state, for computing the derivative;
  double tau_;  //!< the noise reduction term for the derivative

  /*!
   * \brief saturate saturates the variable val
   * \param val the parameter to saturate (makes a copy)
   * \param min the minimum value
   * \param max the max value
   * \return the saturated (if necessary) value
   */
  inline double saturate(double val, double &min, double &max)
  {
    if(val > max)
      val = max;
    else if(val < min)
      val = min;
    return val;
  }
};

}

#endif // ROTOR_CONTROLLER_SIMPLE_PID_H