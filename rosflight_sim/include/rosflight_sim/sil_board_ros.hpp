/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
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

#ifndef ROSFLIGHT_SIM_SIL_BOARD_ROS_H
#define ROSFLIGHT_SIM_SIL_BOARD_ROS_H

#include <cmath>
#include <cstdbool>
#include <cstddef>
#include <cstdint>

#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>

#include "rosflight_msgs/msg/rc_raw.hpp"
#include "rosflight_sim/udp_board.hpp"
#include "sensor_interface.hpp"
#include "sensors.h"

namespace rosflight_sim
{
/**
 * @brief ROSflight firmware board implementation for simulator. This class handles sensors,
 * actuators, and FCU clock and memory for the firmware. It also adds a simulated serial delay. It
 * inherits from UDP board, which establishes a communication link over UDP.
 */
class SILBoardROS : public rclcpp::Node
{
public:
  SILBoardROS();

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr firmware_run_srvs_;

  /**
   * @brief Calls firmware.run()
   */
  bool run_firmware(const std_srvs::srv:Trigger::Request::SharedPtr & req,
                    const std_srvs::srv:Trigger::Response::SharedPtr & res);

  /**
   * @brief Initializes the board and the firmware. Since the SIL board needs a reference to the 
   * containing node, the node must be instantiated before passing the reference to the board. This is
   * why this initialization code is not done in the construtor.
   */
  void initialize_members();


  // Components of the firmware
  std::shared_ptr<SilBoard> board_;
  std::shared_ptr<rosflight_firmware::Mavlink> comm_;
  std::shared_ptr<rosflight_firmware::ROSflight> firmware_;

  bool is_initialized = false;
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_SIL_BOARD_ROS_H

