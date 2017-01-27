/**
 * \file param_listener_interface.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_PARAM_LISTENER_INTERFACE_H
#define MAVROSFLIGHT_PARAM_LISTENER_INTERFACE_H

#include <string>

namespace mavrosflight
{
/**
 * \brief Describes an interface classes can implement to receive and handle mavlink messages
 */
class ParamListenerInterface
{
public:
  /**
   * \brief Called when a parameter is received from the fcu for the first time
   * \param name The name of the parameter
   * \param value The value of the parameter
   */
  virtual void on_new_param_received(std::string name, double value) = 0;

  /**
   * \brief Called when an updated value is received for a parameter
   * \param name The name of the parameter
   * \param value The updated value of the parameter
   */
  virtual void on_param_value_updated(std::string name, double value) = 0;

  /**
   * \brief Called when the status of whether there are unsaved parameters changes
   * \param unsaved_changes True if there are parameters that have been set but not saved on the fcu
   */
  virtual void on_params_saved_change(bool unsaved_changes) = 0;
};

}  // namespace mavrosflight

#endif  // MAVROSFLIGHT_PARAM_LISTENER_INTERFACE_H
