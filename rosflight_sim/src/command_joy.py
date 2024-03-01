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
ROS node for publishing /command topics with a controller.

This script implements a ROS node that uses a joystick controller to
publish commands on the /command topic.

Currently supported joystick controllers are:
    * Taranis Q-X7 transmitter connected over USB
    * XBox controller
    * RealFlight InterLink controller
    * RadioMaster TX16S transmitter connected over USB

To add support for a new controller, simply add its configuration to
the config dictionary.

Authors: James Jackson, Daniel Koch, Brandon Sutherland
"""

from enum import Enum

import pygame
import rclpy
from rclpy.node import Node

from rosflight_msgs.msg import Command


# define output RC channels
class Mapping(Enum):
    x = 0
    y = 1
    z = 2
    f = 3
    xsign = 4
    ysign = 5
    zsign = 6
    fsign = 7


# Joystick configuration. The configuration for each joystick consists
# of the following:
#    - List of keywords to identify the joystick/controller type
#    - A lambda function for each channel that returns an appropriate
#      value between -1 and 1, given a joystick object
config = {}

config['Taranis'] = {}
config['Taranis']['keys'] = ['Taranis']
config['Taranis'][Mapping.x] = 0
config['Taranis'][Mapping.y] = 1
config['Taranis'][Mapping.z] = 3
config['Taranis'][Mapping.f] = 2
config['Taranis'][Mapping.xsign] = 1
config['Taranis'][Mapping.ysign] = 1
config['Taranis'][Mapping.zsign] = 1
config['Taranis'][Mapping.fsign] = 1

config['XBox'] = {}
config['XBox']['keys'] = ['Xbox', 'X-Box']
config['XBox'][Mapping.x] = 3
config['XBox'][Mapping.y] = 4
config['XBox'][Mapping.z] = 0
config['XBox'][Mapping.f] = 1
config['XBox'][Mapping.xsign] = 1
config['XBox'][Mapping.ysign] = 1
config['XBox'][Mapping.zsign] = 1
config['XBox'][Mapping.fsign] = -1

config['RealFlight'] = {}
config['RealFlight']['keys'] = ['GREAT PLANES']
config['RealFlight'][Mapping.x] = 1
config['RealFlight'][Mapping.y] = 2
config['RealFlight'][Mapping.z] = 4
config['RealFlight'][Mapping.f] = 0
config['RealFlight'][Mapping.xsign] = 1
config['RealFlight'][Mapping.ysign] = 1
config['RealFlight'][Mapping.zsign] = 1
config['RealFlight'][Mapping.fsign] = 1

config['TX16S'] = {}
config['TX16S']['keys'] = ['OpenTX RM TX16S Joystick']
config['TX16S'][Mapping.x] = 0
config['TX16S'][Mapping.y] = 1
config['TX16S'][Mapping.z] = 3
config['TX16S'][Mapping.f] = 2
config['TX16S'][Mapping.xsign] = 1
config['TX16S'][Mapping.ysign] = 1
config['TX16S'][Mapping.zsign] = 1
config['TX16S'][Mapping.fsign] = 1


class CommandJoy(Node):
    def __init__(self):
        update_freq = 50  # hz

        super().__init__('command_joy')
        self.rc_publisher = self.create_publisher(Command, 'command', 10)
        self.timer = self.create_timer(1 / update_freq, self.timer_callback)

        pygame.display.init()
        pygame.joystick.init()

        try:
            self.joy = pygame.joystick.Joystick(0)
        except:
            self.get_logger().fatal('Failed to open joystick device')
            quit()

        self.joy.init()

        # detect joystick/controller type
        self.get_logger().info('Joystick: %s' % self.joy.get_name())

        self.joy_type = None
        for k in config.keys():
            for key in config[k]['keys']:
                if key in self.joy.get_name():
                    self.joy_type = k
                    break

        if self.joy_type is None:
            self.get_logger().fatal('Unsupported joystick device')
            quit()

        self.get_logger().info('Using %s config' % self.joy_type)

    def timer_callback(self):
        pygame.event.pump()

        msg = Command()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.x = self.joy.get_axis(config[self.joy_type][Mapping.x]) * config[self.joy_type][Mapping.xsign]
        msg.y = self.joy.get_axis(config[self.joy_type][Mapping.y]) * config[self.joy_type][Mapping.ysign]
        msg.z = self.joy.get_axis(config[self.joy_type][Mapping.z]) * config[self.joy_type][Mapping.zsign]
        msg.f = self.joy.get_axis(config[self.joy_type][Mapping.f]) * config[self.joy_type][Mapping.fsign]

        self.rc_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rc_joy = CommandJoy()
    rclpy.spin(rc_joy)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
