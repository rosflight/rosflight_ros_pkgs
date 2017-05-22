/*
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
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
   * \brief Called when a parameter is received from the autopilot for the first time
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
   * \param unsaved_changes True if there are parameters that have been set but not saved on the autopilot
   */
  virtual void on_params_saved_change(bool unsaved_changes) = 0;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_PARAM_LISTENER_INTERFACE_H
