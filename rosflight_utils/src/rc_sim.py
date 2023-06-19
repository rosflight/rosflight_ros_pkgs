#!/usr/bin/python

# author: James Jackson

import rclpy
from rclpy.node import Node
from rosflight_msgs.msg import RCRaw
from std_srvs.srv import Trigger


class DummyRC(Node):
    def __init__(self):
        self.THROTTLE_CHANNEL = 2
        self.ARM_SWITCH_CHANNEL = 4
        self.CHANNEL_MIN = 1000
        self.CHANNEL_MID = 1500
        self.CHANNEL_MAX = 2000

        # Initialize rc message with null values
        self.rc_message = RCRaw()
        for i in range(8):
            self.rc_message.values[i] = self.CHANNEL_MID
        self.rc_message.values[self.THROTTLE_CHANNEL] = self.CHANNEL_MIN
        self.rc_message.values[self.ARM_SWITCH_CHANNEL] = self.CHANNEL_MIN

        # Initialize ROS components
        super().__init__('dummy_rc')
        self.arm_service = self.create_service(Trigger, 'arm', self.arm_callback)
        self.disarm_service = self.create_service(Trigger, 'disarm', self.disarm_callback)
        self.rc_publisher = self.create_publisher(RCRaw, 'RC', 10)
        self.timer = self.create_timer(1/50, self.timer_callback)

    def arm_callback(self, request, response):
        self.rc_message.values[self.ARM_SWITCH_CHANNEL] = self.CHANNEL_MAX
        response.success = True
        response.message = 'Armed!'
        return response

    def disarm_callback(self, request, response):
        self.rc_message.values[self.ARM_SWITCH_CHANNEL] = self.CHANNEL_MIN
        response.success = True
        response.message = 'Disarmed!'
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
