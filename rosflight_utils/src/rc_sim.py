#!/usr/bin/env python3

# Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
# Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
ROS node for emulating RC controller for SIL simulation

This script implements a ROS node that emulates an RC receiver
for software-in-the-loop simulation with the ROSflight Gazebo
simulator. Receiver commands are controlled with ROS2 service calls.

Authors: James Jackson, Daniel Koch, Brandon Sutherland, Ian Reid
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from rosflight_msgs.msg import RCRaw


class DummyRC(Node):
    def __init__(self):
        update_freq = 50  # hz

        self.THROTTLE_CHANNEL = 2
        self.ARM_SWITCH_CHANNEL = 4
        self.OVERRIDE_CHANNEL = 5
        self.CHANNEL_MIN = 1000
        self.CHANNEL_MID = 1500
        self.CHANNEL_MAX = 2000

        # Initialize rc message with null values
        self.rc_message = RCRaw()
        for i in range(8):
            self.rc_message.values[i] = self.CHANNEL_MID
        self.rc_message.values[self.THROTTLE_CHANNEL] = self.CHANNEL_MIN
        self.rc_message.values[self.ARM_SWITCH_CHANNEL] = self.CHANNEL_MIN
        self.rc_message.values[self.OVERRIDE_CHANNEL] = self.CHANNEL_MAX

        # Initialize ROS components
        super().__init__('dummy_rc')
        self.arm_service = self.create_service(Trigger, 'arm', self.arm_callback)
        self.disarm_service = self.create_service(Trigger, 'disarm', self.disarm_callback)
        self.disable_override_service = self.create_service(Trigger, 'enable_override', self.enable_override_callback)
        self.disable_override_service = self.create_service(Trigger, 'disable_override', self.disable_override_callback)
        self.rc_publisher = self.create_publisher(RCRaw, 'RC', 10)
        self.timer = self.create_timer(1 / update_freq, self.timer_callback)

    def arm_callback(self, request, response):
        self.rc_message.values[self.ARM_SWITCH_CHANNEL] = self.CHANNEL_MAX
        response.success = True
        response.message = 'Arm switch enabled!'
        return response

    def disarm_callback(self, request, response):
        self.rc_message.values[self.ARM_SWITCH_CHANNEL] = self.CHANNEL_MIN
        response.success = True
        response.message = 'Arm switch disabled!'
        return response

    def enable_override_callback(self, request, response):
        self.rc_message.values[self.OVERRIDE_CHANNEL] = self.CHANNEL_MAX
        response.success = True
        response.message = 'Override switch enabled!'
        return response

    def disable_override_callback(self, request, response):
        self.rc_message.values[self.OVERRIDE_CHANNEL] = self.CHANNEL_MIN
        response.success = True
        response.message = 'Override switch disabled!'
        return response

    def timer_callback(self):
        self.rc_message.header.stamp = self.get_clock().now().to_msg()
        self.rc_publisher.publish(self.rc_message)


def main(args=None):
    rclpy.init(args=args)
    dummy_rc = DummyRC()
    rclpy.spin(dummy_rc)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
