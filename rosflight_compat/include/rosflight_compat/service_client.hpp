/*
 * Copyright (c) 2026 BYU MAGICC Lab.
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

/*
 * This file was created to provide backwards compatiblitiy for ROS 2 Humble.
 * Humble uses deprecated C types (rmw_qos_profile_services_default) while
 * later versions of ROS 2 migrated to C++ types: ServicesQoS().
 *
 * TODO: Once Humble is not longer supported, this compatiblitiy layer can be
 * deleted and all files using it should change to create the clients directly
 * using the modern ServicesQoS() type.
 */

#ifndef ROSFLIGHT_COMPAT_SERVICE_CLIENT_HPP
#define ROSFLIGHT_COMPAT_SERVICE_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <rmw/qos_profiles.h>

namespace rosflight_compat
{

template<typename ServiceT>
typename rclcpp::Client<ServiceT>::SharedPtr
create_service_client(rclcpp::Node & node, const std::string & service_name,
                      rclcpp::CallbackGroup::SharedPtr callback_group)
{
#if RCLCPP_VERSION_MAJOR < 21
  // legacy version for ROS 2 Humble on Ubuntu 22.04
  return node.create_client<ServiceT>(service_name, rmw_qos_profile_services_default,
                                      callback_group);
#else
  // modern version
  return node.create_client<ServiceT>(service_name, rclcpp::ServicesQoS(), callback_group);
#endif
}

} // namespace rosflight_compat

#endif // ROSFLIGHT_COMPAT_SERVICE_CLIENT_HPP
