#!/usr/bin/env python
"""
vimfly - vim keybindings for your multirotor!

Teleoperated flying from your keyboard.
Keys are mapped to each channel of the RCRaw message.

"""
import pygame
import rclpy
import time
from rosflight_msgs.msg import RCRaw


class VimFly:
    def __init__(self):

        self.node = rclpy.create_node('vimfly')

        # initialize pygame display
        pygame.init()
        pygame.display.set_caption('vimfly')
        self.screen = pygame.display.set_mode((550, 200))
        self.font = pygame.font.SysFont("monospace", 18)

        # create publisher for RC commands.
        self.rc_pub = self.node.create_publisher(RCRaw, 'RC', 10)
        
        # TODO: swap these hardcoded vals to params
        self.rate = 30

        # self.timer = self.node.create_timer(1/self.rate, self.run)

        self.thrust_command = 1000
        self.thrust_step = 10
        self.THRUST_DEBOUNCE_THRESHOLD = 0.100
        self.thrust_debouncing = [False]
        self.thrust_start_time = self.node.get_clock().now()

        self.armed = 1000 # Unarmed
        self.ARM_DEBOUNCE_THRESHOLD = 0.250
        self.arm_debouncing = [False]
        self.arm_start_time = self.node.get_clock().now()

        self.RC_override = 2000 # Start under manual control
        self.RC_OVERRIDE_DEBOUNCE_THRESHOLD = 0.250
        self.RC_override_debouncing = [False]
        self.RC_override_start_time = self.node.get_clock().now()

        while(True):
            self.keys = pygame.key.get_pressed()
            self.run()
            time.sleep(1/self.rate)


    def run(self):

        # initialize command message
        msg = RCRaw()
        msg.header.stamp = self.node.get_clock().now().to_msg()

        # LEFT -- h
        if self.keys[pygame.K_h]:
            msg.values[0] = 2000

        # RIGHT -- l
        elif self.keys[pygame.K_l]:
            msg.values[0] = 1000

        else:
            msg.values[0] = 1500

        # FORWARD -- k
        if self.keys[pygame.K_k]:
            msg.values[1] = 2000

        # BACKWARD -- j
        elif self.keys[pygame.K_j]:
            msg.values[1] = 1000

        else:
            msg.values[1] = 1500

        # CCW -- d
        if self.keys[pygame.K_d]:
            msg.values[3] = 2000

        # CW -- f
        elif self.keys[pygame.K_f]:
            msg.values[3] = 1000

        else:
            msg.values[3] = 1500

        # THRUST LOWER -- s  //  THRUST HIGHER -- a
        if self.keys[pygame.K_a] or self.keys[pygame.K_s]:
            self.debounce(self.thrust_debouncing, self.thrust_start_time, self.THRUST_DEBOUNCE_THRESHOLD, self.increment_thrust)
        else:
            self.thrust_debouncing[0] = False

        # Always send an thrust command -- we don't want to drop like a brick!
        msg.values[2] = self.thrust_command
        
        # Send arm commands
        if self.keys[pygame.K_t]:
            self.debounce(self.arm_debouncing, self.arm_start_time, self.ARM_DEBOUNCE_THRESHOLD, self.toggle_arm)
        else:
            self.arm_debouncing[0] = False

        msg.values[4] = self.armed
        
        # Send RC override commands
        if self.keys[pygame.K_r]:
            self.debounce(self.RC_override_debouncing, self.RC_override_start_time, self.RC_OVERRIDE_DEBOUNCE_THRESHOLD, self.toggle_RC_override)
        else:
            self.RC_override_debouncing[0] = False
        
        msg.values[5] = self.RC_override

        # Pad the message with remaining channels.
        msg.values[6] = 1500
        msg.values[7] = 1500

        # Publish commands
        self.rc_pub.publish(msg)

        # Update the display with the current commands
        self.update_display(msg)

        # process event queue and throttle the while loop
        pygame.event.pump()


    def update_display(self, msg):
        self.display_help()

        msgText = "roll: {}, pitch: {}, yaw: {}, thrust: {}".format(msg.values[0], msg.values[1], msg.values[3], msg.values[2])

        status_info = "armed: {}, RC_override: {}".format(msg.values[4], msg.values[5])
        self.render(msgText, (0,140))
        self.render(status_info, (100,160))
        
        pygame.display.flip()


    def display_help(self):
        self.screen.fill((0,0,0))

        LINE=20

        self.render("vimfly keybindings:", (0,0))
        self.render("- a: higher thrust", (0,1*LINE)); self.render("- h: Roll Left", (250,1*LINE))
        self.render("- s: lower thrust", (0,2*LINE)); self.render("- j: Pitch Backward", (250,2*LINE))
        self.render("- d: yaw CCW", (0,3*LINE)); self.render("- k: Pitch Forward", (250,3*LINE))
        self.render("- f: yaw CW", (0,4*LINE)); self.render("- l: Roll Right", (250,4*LINE))
        self.render("- t: arm/disarm", (0,5*LINE)); self.render("- r: RC override", (250,5*LINE))


    def render(self, text, loc):
        txt = self.font.render(text, 1, (255,255,255))
        self.screen.blit(txt, loc)

    def debounce(self, debounced, debounce_start_time, threshold, key_action):

        if not debounced[0]:
            debounced[0] = True
            debounce_start_time = self.node.get_clock().now()

        if (self.node.get_clock().now() - debounce_start_time).nanoseconds / 1e9 > threshold:
            
            # The key has been debounced once, start the process over!
            debounced[0] = False

            key_action()

    def increment_thrust(self):
        # Increment the commanded altitude
        if self.keys[pygame.K_a] and self.thrust_command < 2000:
            self.thrust_command += self.thrust_step

        elif self.keys[pygame.K_s] and self.thrust_command > 1000:
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

if __name__ == '__main__':
    rclpy.init()
    teleop = VimFly()
