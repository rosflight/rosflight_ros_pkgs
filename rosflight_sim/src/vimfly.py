#!/usr/bin/env python3
"""
vimfly - vim keybindings for your multirotor!

Teleoperated flying from your keyboard.
Keys are mapped to each channel of the RCRaw message.

"""

import time

import pygame
import rclpy
from rclpy.node import Node
from rosflight_msgs.msg import RCRaw


class VimFly:
    def __init__(self, node=None):

        if node is None:
            self.node = rclpy.create_node('vimfly')
        elif isinstance(node, Node):
            self.node = node
        else:
            raise TypeError('node must be an rclpy.node.Node')

        # initialize pygame display
        pygame.init()
        pygame.display.set_caption('vimfly')
        self.screen = pygame.display.set_mode((550, 200))
        self.font = pygame.font.SysFont('monospace', 18)

        # create publisher for RC commands.
        self.rc_pub = self.node.create_publisher(RCRaw, 'sim/RC', 10)

        # TODO: swap these hardcoded vals to params
        self.rate = 30
        self.timer_period_sec = 1 / self.rate

        self.thrust_command = 1000
        self.thrust_step = 10
        self.THRUST_DEBOUNCE_THRESHOLD = 0.02
        self.thrust_debouncing = [False]
        self.thrust_start_time = time.perf_counter()

        self.armed = 1000  # Unarmed
        self.arm_debouncing = [False]

        self.RC_override = 2000  # Start under manual control
        self.RC_override_debouncing = [False]

        self.angle_mode = 1000  # Start in rate mode
        self.angle_mode_debouncing = [False]

        self.pressed_keys = set()

        self.running = True

        # Use a ROS timer so VimFly does not block rclpy.spin()
        self.timer = self.node.create_timer(self.timer_period_sec, self.run_once)

    def run_once(self):
        self.process_pygame_events()
        if not self.running:
            return
        msg = self.populate_rc_msg()
        self.rc_pub.publish(msg)
        self.update_display(msg)

    def process_pygame_events(self):
        for event in pygame.event.get():
            # check if window was closed
            if event.type == pygame.QUIT:
                self.stop_running()
                return

            # reset state if focus is lost
            elif event.type == pygame.WINDOWFOCUSLOST:
                self.reset_input_state()

            # register key presses
            elif event.type == pygame.KEYDOWN:
                if event.key not in self.pressed_keys:
                    self.pressed_keys.add(event.key)
                    if event.key in (pygame.K_a, pygame.K_s):
                        self.increment_thrust()
                        self.thrust_start_time = time.perf_counter()
                        self.thrust_debouncing[0] = True

            # register key releases
            elif event.type == pygame.KEYUP:
                self.pressed_keys.discard(event.key)
                if event.key in (pygame.K_a, pygame.K_s):
                    self.thrust_debouncing[0] = False

    def populate_rc_msg(self):

        # initialize command message
        msg = RCRaw()
        msg.header.stamp = self.node.get_clock().now().to_msg()

        if pygame.K_h in self.pressed_keys:  # LEFT -- h
            msg.values[0] = 2000
        elif pygame.K_l in self.pressed_keys:  # RIGHT -- l
            msg.values[0] = 1000
        else:
            msg.values[0] = 1500

        if pygame.K_k in self.pressed_keys:  # FORWARD -- k
            msg.values[1] = 2000
        elif pygame.K_j in self.pressed_keys:  # BACKWARD -- j
            msg.values[1] = 1000
        else:
            msg.values[1] = 1500

        if pygame.K_d in self.pressed_keys:  # CCW -- d
            msg.values[3] = 2000
        elif pygame.K_f in self.pressed_keys:  # CW -- f
            msg.values[3] = 1000
        else:
            msg.values[3] = 1500

        # THRUST LOWER -- s  //  THRUST HIGHER -- a
        if pygame.K_a in self.pressed_keys or pygame.K_s in self.pressed_keys:
            self.thrust_start_time = self.debounce(
                self.thrust_debouncing,
                self.thrust_start_time,
                self.THRUST_DEBOUNCE_THRESHOLD,
                self.increment_thrust,
            )
        else:
            self.thrust_debouncing[0] = False

        # Always send an thrust command -- we don't want to drop like a brick!
        msg.values[2] = self.thrust_command

        # Send arm commands
        if pygame.K_t in self.pressed_keys:
            self.toggle_once(self.arm_debouncing, self.toggle_arm)
        else:
            self.arm_debouncing[0] = False
        msg.values[4] = self.armed

        # Send RC override commands
        if pygame.K_r in self.pressed_keys:
            self.toggle_once(self.RC_override_debouncing, self.toggle_RC_override)
        else:
            self.RC_override_debouncing[0] = False
        msg.values[5] = self.RC_override

        # Send angle mode commands
        if pygame.K_m in self.pressed_keys:
            self.toggle_once(self.angle_mode_debouncing, self.toggle_angle_mode)
        else:
            self.angle_mode_debouncing[0] = False
        msg.values[6] = self.angle_mode

        # Pad the message with remaining channels.
        msg.values[7] = 1500

        return msg

    def update_display(self, msg):
        self.display_help()

        msgText = 'roll: {}, pitch: {}, yaw: {}, thrust: {}'.format(
            msg.values[0], msg.values[1], msg.values[3], msg.values[2]
        )

        status_info = 'armed: {}, RC_override: {}, angle_mode: {}'.format(
            msg.values[4], msg.values[5], msg.values[6]
        )
        self.render(msgText, (0, 140))
        self.render(status_info, (0, 160))

        pygame.display.flip()

    def display_help(self):
        self.screen.fill((0, 0, 0))

        line = 20
        col1 = 0
        col2 = 250

        # left column
        self.render('vimfly keybindings:', (col1, 0))
        self.render('- a: higher thrust', (col1, 1 * line))
        self.render('- s: lower thrust', (col1, 2 * line))
        self.render('- d: yaw CCW', (col1, 3 * line))
        self.render('- f: yaw CW', (col1, 4 * line))
        self.render('- t: arm/disarm', (col1, 5 * line))
        self.render('- m: angle mode', (col1, 6 * line))

        # right column
        self.render('- h: Roll Left', (col2, 1 * line))
        self.render('- j: Pitch Backward', (col2, 2 * line))
        self.render('- k: Pitch Forward', (col2, 3 * line))
        self.render('- l: Roll Right', (col2, 4 * line))
        self.render('- r: RC override', (col2, 5 * line))

    def render(self, text, loc):
        txt = self.font.render(text, 1, (255, 255, 255))
        self.screen.blit(txt, loc)

    def debounce(self, debounced, debounce_start_time, threshold, key_action):
        now = time.perf_counter()

        if not debounced[0]:
            debounced[0] = True
            debounce_start_time = now

        if now - debounce_start_time > threshold:
            # The key has been debounced once, start the process over!
            debounced[0] = False

            key_action()

        return debounce_start_time

    def toggle_once(self, debounced, key_action):
        if not debounced[0]:
            debounced[0] = True
            key_action()

    def increment_thrust(self):
        # Increment the commanded altitude
        if pygame.K_a in self.pressed_keys and self.thrust_command < 2000:
            self.thrust_command += self.thrust_step

        elif pygame.K_s in self.pressed_keys and self.thrust_command > 1000:
            self.thrust_command -= self.thrust_step

    def toggle_arm(self):
        if self.armed == 1000:
            self.armed = 2000
        else:
            self.armed = 1000

    def toggle_RC_override(self):
        if self.RC_override == 1000:
            self.RC_override = 2000
        else:
            self.RC_override = 1000

    def toggle_angle_mode(self):
        if self.angle_mode == 1000:
            self.angle_mode = 2000
        else:
            self.angle_mode = 1000

    def reset_input_state(self):
        self.pressed_keys.clear()
        self.thrust_debouncing[0] = False
        self.arm_debouncing[0] = False
        self.RC_override_debouncing[0] = False
        self.angle_mode_debouncing[0] = False

    def stop_running(self):
        self.running = False
        self.reset_input_state()
        self.timer.cancel()
        pygame.quit()
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()
    teleop = VimFly()
    rclpy.spin(teleop.node)
    if rclpy.ok():
        rclpy.shutdown()
