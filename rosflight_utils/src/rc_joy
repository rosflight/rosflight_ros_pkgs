#!/usr/bin/env python

# Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
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

"""ROS node for emulating RC controller for SIL simulation

This script implements a ROS node that uses a joystick controller to
emulate an RC receiver for software-in-the-loop simulation with the
ROSflight Gazebo simulator.

Currently supported joystick controllers are:
    * Taranis Q-X7 transmitter connected over USB
    * XBox controller
    * RealFlight InterLink controller

To add support for a new controller, simply add its configuration to
the config dictionary.

Authors: James Jackson, Daniel Koch
"""

import pygame
import rospy
from rosflight_msgs.msg import RCRaw

from enum import Enum


# define output RC channels
class Channel(Enum):
    AIL = 0
    ELV = 1
    THR = 2
    RUD = 3
    SW1 = 4
    SW2 = 5
    SW3 = 6
    SW4 = 7

# Joystick configuration. The configuration for each joystick consists
# of the following:
#    - List of keywords to identify the joystick/controller type
#    - A lambda function for each channel that returns an appropriate
#      value between -1 and 1, given a joystick object
config = {}

config['Taranis'] = {}
config['Taranis']['keys'] = ['Taranis']
config['Taranis'][Channel.AIL] = lambda j: j.get_axis(0)
config['Taranis'][Channel.ELV] = lambda j: j.get_axis(1)
config['Taranis'][Channel.THR] = lambda j: j.get_axis(2)
config['Taranis'][Channel.RUD] = lambda j: j.get_axis(3)
config['Taranis'][Channel.SW1] = lambda j: j.get_axis(4)
config['Taranis'][Channel.SW2] = lambda j: j.get_axis(5)
config['Taranis'][Channel.SW3] = lambda j: j.get_axis(6)
config['Taranis'][Channel.SW4] = lambda j: j.get_axis(7)

config['XBox'] = {}
config['XBox']['keys'] = ['Xbox', 'X-Box']
config['XBox'][Channel.AIL] = lambda j: j.get_axis(3)
config['XBox'][Channel.ELV] = lambda j: j.get_axis(4)
config['XBox'][Channel.THR] = lambda j: -j.get_axis(1)
config['XBox'][Channel.RUD] = lambda j: j.get_axis(0)
config['XBox'][Channel.SW1] = lambda j: j.get_button(0)
config['XBox'][Channel.SW2] = lambda j: j.get_button(1)
config['XBox'][Channel.SW3] = lambda j: j.get_button(2)
config['XBox'][Channel.SW4] = lambda j: j.get_button(3)

config['RealFlight'] = {}
config['RealFlight']['keys'] = ['GREAT PLANES']
config['RealFlight'][Channel.AIL] = lambda j: j.get_axis(0)
config['RealFlight'][Channel.ELV] = lambda j: j.get_axis(1)
config['RealFlight'][Channel.THR] = lambda j: -j.get_axis(2)
config['RealFlight'][Channel.RUD] = lambda j: j.get_axis(4)
config['RealFlight'][Channel.SW1] = lambda j: 2*j.get_button(4)-1
config['RealFlight'][Channel.SW2] = lambda j: 2*j.get_button(0)-1
config['RealFlight'][Channel.SW3] = lambda j: 2*j.get_button(1)-1
config['RealFlight'][Channel.SW4] = lambda j: 2*j.get_button(2)-1


if __name__ == '__main__':

    # initialize
    rospy.init_node('rc_joy')
    rc_pub = rospy.Publisher('RC', RCRaw, queue_size=10)

    pygame.display.init()
    pygame.joystick.init()

    try:
        joy = pygame.joystick.Joystick(0)
    except:
        rospy.logfatal("Failed to open joystick device")
        rospy.signal_shutdown("Failed to open joystick device")
        quit()

    joy.init()

    # detect joystick/controller type
    rospy.loginfo("Joystick: %s" % joy.get_name())

    joy_type = None
    for k in config.keys():
        for key in config[k]['keys']:
            if key in joy.get_name():
                joy_type = k
                break

    if joy_type is None:
        rospy.logfatal('Unsupported joystick device')
        rospy.signal_shutdown('Unsupported joystick device')
        quit()

    rospy.loginfo("Using %s config" % joy_type)

    # main loop
    update_freq = 50 # Hz
    rate = rospy.Rate(update_freq)
    while not rospy.is_shutdown():
        pygame.event.pump()

        msg = RCRaw()
        msg.header.stamp = rospy.Time.now()

        for chan in Channel:
            msg.values[chan.value] = round(config[joy_type][chan](joy) * 500 + 1500)

        rc_pub.publish(msg)

        rate.sleep()
