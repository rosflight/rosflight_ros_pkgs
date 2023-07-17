#!/usr/bin/env python3

# author: James Jackson

import pygame
import rclpy
from rclpy.node import Node
from rosflight_msgs.msg import Command

from enum import Enum


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
        self.timer = self.create_timer(1/update_freq, self.timer_callback)

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
