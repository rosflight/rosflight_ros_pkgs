/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
 * Copyright (c) 2024 Ian Reid, BYU MAGICC Lab.
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

#include "rosflight_sim/sil_board.hpp"
#include "rosflight_sim/sil_board_ros.hpp"

namespace rosflight_sim
{

SILBoardROS::SILBoardROS()
  : rclcpp::Node("sil_board")
{}

void SILBoardROS::initialize_members()
{
  auto node_ptr = shared_from_this();
  board_ = std::make_shared<SILBoard>(node_ptr);

  comm_ = std::make_shared<rosflight_firmware::Mavlink>(*board_);
  firmware_ = std::make_shared<rosflight_firmware::ROSflight>(*board_, *comm_);

  // Initialize the firmware
  firmware_->init();
}

bool SILBoardROS::run_firmware(const std_srvs::srv::Trigger::Request::SharedPtr & req,  
                               const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  if (!is_initialized) {
    initialize_members();
    is_initialized = true;
  }

  // TODO: In the original rosflight_sil, they called run twice "to make sure that functions that don't get run when we have imu get run".
  // What functions are these, and do we still need to call it twice?
  firmware_->run();
  firmware_->run();

  // Package response and return
  res.success = true;
  return true;
}

} // namespace rosflight_sim

// TODO: If we want to do zero copy transfer, remove this main function. For now it will remain
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SILBoardROS>();

  rclcpp::spin(node);

  return 0;
}

